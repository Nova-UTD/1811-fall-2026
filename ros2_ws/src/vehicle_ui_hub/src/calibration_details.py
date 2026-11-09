# Show calibration matrices and the TF frames actually received from ROS.
import math
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets
from hub_widgets import AnimatedStatusLabel
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfGraph(QtWidgets.QWidget):
    # Draw the received TF graph directly, without spawning Graphviz or a new
    # process. The list remains available for keyboard navigation and details.
    selected = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setObjectName('tfGraph')
        self._nodes, self._links, self._signature = {}, [], None
        self._selected, self._highlights, self._stale = '', set(), set()
        self.setMinimumSize(300, 180)

    def set_frames(self, edges, selected, highlights):
        now = time.monotonic()
        stale = {child for child, edge in edges.items() if not edge[1] and now - edge[2] > 5}
        signature = (tuple(sorted((child, edge[0], edge[1]) for child, edge in edges.items())),
                     selected, tuple(sorted(highlights)), tuple(sorted(stale)))
        if signature == self._signature:
            return
        self._signature, self._selected, self._highlights, self._stale = signature, selected, set(highlights), stale
        children, frames = {}, set()
        for child, edge in edges.items():
            children.setdefault(edge[0], []).append(child)
            frames.update((child, edge[0]))
        for values in children.values():
            values.sort()
        self._nodes, self._links = {}, []
        visited, next_y = set(), 22.0
        roots = sorted(frame for frame in frames if frame not in edges)
        # Cycles have no root; visit remaining frames once after normal roots.
        for root in roots + sorted(frames):
            if root in visited:
                continue
            stack = [(root, 0, False)]
            while stack:
                frame, depth, complete = stack.pop()
                if not complete:
                    if frame in visited:
                        continue
                    visited.add(frame)
                    stack.append((frame, depth, True))
                    stack.extend((child, depth + 1, False) for child in reversed(children.get(frame, ())) if child not in visited)
                    continue
                child_nodes = [self._nodes[child] for child in children.get(frame, ()) if child in self._nodes]
                if child_nodes:
                    y = sum(rect.center().y() for rect in child_nodes) / len(child_nodes) - 18
                else:
                    y, next_y = next_y, next_y + 54
                self._nodes[frame] = QtCore.QRectF(14 + depth * 202, y, 164, 36)
        for child, edge in edges.items():
            if child in self._nodes and edge[0] in self._nodes:
                self._links.append((edge[0], child))
        width = max((rect.right() for rect in self._nodes.values()), default=286)
        height = max((rect.bottom() for rect in self._nodes.values()), default=160)
        self.setFixedSize(max(300, int(width + 18)), max(180, int(height + 22)))
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        white = bool(QtWidgets.qApp.property('vehicleHubWhiteTheme'))
        painter.fillRect(self.rect(), QtGui.QColor('#f7f7f7' if white else '#090909'))
        painter.setPen(QtGui.QPen(QtGui.QColor('#a3a8ac' if white else '#565d63'), 1.2))
        for parent, child in self._links:
            start = QtCore.QPointF(self._nodes[parent].right(), self._nodes[parent].center().y())
            end = QtCore.QPointF(self._nodes[child].left(), self._nodes[child].center().y())
            path = QtGui.QPainterPath(start)
            middle = (start.x() + end.x()) / 2
            path.cubicTo(middle, start.y(), middle, end.y(), end.x(), end.y())
            painter.drawPath(path)
        for frame, rect in self._nodes.items():
            if not event.rect().intersects(rect.toAlignedRect().adjusted(-4, -4, 4, 4)):
                continue
            selected, highlight = frame == self._selected, frame in self._highlights
            painter.setBrush(QtGui.QColor('#dbe9f5' if white and selected else '#183248' if selected else
                                          '#ffffff' if white else '#17191b'))
            painter.setPen(QtGui.QPen(QtGui.QColor('#3278ae' if white and highlight else '#75b9eb' if highlight else
                                                  '#c7cbd0' if white else '#555b60'), 1))
            painter.drawRoundedRect(rect, 5, 5)
            painter.setPen(QtGui.QColor('#a36b2a' if white and frame in self._stale else
                                        '#e1a86b' if frame in self._stale else '#202124' if white else '#e7e9ea'))
            text = painter.fontMetrics().elidedText(frame, QtCore.Qt.ElideMiddle, int(rect.width()) - 16)
            painter.drawText(rect.adjusted(8, 0, -8, 0), QtCore.Qt.AlignVCenter, text)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and any(rect.contains(event.pos()) for rect in self._nodes.values()):
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            for frame, rect in self._nodes.items():
                if rect.contains(event.pos()):
                    self.selected.emit(frame)
                    event.accept()
                    return
        super().mouseReleaseEvent(event)


class CalibrationDetails(QtWidgets.QWidget):
    def __init__(self, node, preview=False, parent=None):
        super().__init__(parent)
        self.node, self.preview = node, preview
        self.setObjectName('calibrationDetails')
        self._lock = threading.Lock()
        self._edges, self._subscriptions = {}, []
        self._generation, self._idle = 0, True
        self._cameras, self._connected, self._selected_tf = {}, False, ''
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 8, 0, 0)
        layout.setSpacing(8)
        title = QtWidgets.QLabel('Calibration details')
        title.setObjectName('sectionTitle')
        layout.addWidget(title)
        self._camera = QtWidgets.QComboBox()
        self._camera.setObjectName('detailCamera')
        for name in ('front', 'left', 'rear', 'right'):
            self._camera.addItem(name.capitalize(), name)
        layout.addWidget(self._camera)
        self._source = AnimatedStatusLabel('Waiting for data')
        self._source.setObjectName('transformSource')
        self._source.setWordWrap(True)
        layout.addWidget(self._source)
        self._quality = QtWidgets.QLabel('No quality data')
        self._quality.setObjectName('calibrationQuality')
        self._quality.setWordWrap(True)
        layout.addWidget(self._quality)
        self._cloud_direction = QtWidgets.QLabel('Camera cloud → LiDAR')
        self._cloud_direction.setWordWrap(True)
        layout.addWidget(self._cloud_direction)
        self._cloud_matrix = self._matrix_label('cloudMatrix')
        layout.addWidget(self._cloud_matrix)
        self._mount_direction = QtWidgets.QLabel('Camera body → base_link')
        self._mount_direction.setWordWrap(True)
        layout.addWidget(self._mount_direction)
        self._mount_matrix = self._matrix_label('mountMatrix')
        layout.addWidget(self._mount_matrix)
        units = QtWidgets.QLabel('Translation in meters')
        units.setObjectName('statusHint')
        layout.addWidget(units)
        tree_title = QtWidgets.QLabel('TF tree')
        tree_title.setObjectName('sectionTitle')
        layout.addWidget(tree_title)
        self._tree_info = AnimatedStatusLabel('Waiting for /tf and /tf_static')
        self._tree_info.setObjectName('statusHint')
        self._tree_info.setWordWrap(True)
        layout.addWidget(self._tree_info)
        self._graph = TfGraph()
        self._graph_area = QtWidgets.QScrollArea()
        self._graph_area.setObjectName('tfGraphArea')
        self._graph_area.setWidget(self._graph)
        self._graph_area.setWidgetResizable(False)
        self._graph_area.setMinimumHeight(190)
        self._graph_area.setMaximumHeight(260)
        layout.addWidget(self._graph_area)
        self._tree = QtWidgets.QTreeWidget()
        self._tree.setObjectName('tfTree')
        self._tree.setHeaderLabels(['Frame', 'Source'])
        self._tree.setMinimumHeight(220)
        self._tree.setMaximumHeight(300)
        self._tree.setColumnWidth(0, 230)
        layout.addWidget(self._tree)
        self._tf_direction = QtWidgets.QLabel('Select a frame')
        self._tf_direction.setObjectName('tfDirection')
        self._tf_direction.setWordWrap(True)
        layout.addWidget(self._tf_direction)
        self._tf_matrix = QtWidgets.QLabel('—')
        self._tf_matrix.setObjectName('tfMatrix')
        self._tf_matrix.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self._tf_matrix)
        self._tree.currentItemChanged.connect(self._select_tf)
        self._graph.selected.connect(self._select_graph_frame)
        self._camera.currentIndexChanged.connect(lambda _: self.refresh())
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(lambda: self.refresh() if self.isVisible() else None)
        self.set_idle(False)

    @staticmethod
    def _matrix_label(name):
        label = QtWidgets.QLabel('—')
        label.setObjectName(name)
        label.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        return label

    def set_idle(self, idle):
        idle = bool(idle)
        if self._idle == idle:
            return
        with self._lock:
            self._idle = idle
            self._generation += 1
            generation = self._generation
            if idle:
                self._edges.clear()
        if idle:
            self._timer.stop()
            if self.node:
                for subscription in self._subscriptions:
                    self.node.destroy_subscription(subscription)
            self._subscriptions.clear()
            return
        if self.node:
            dynamic = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._subscriptions = [
                self.node.create_subscription(TFMessage, '/tf',
                                              lambda msg: self._receive_tf(msg, False, generation), dynamic),
                self.node.create_subscription(TFMessage, '/tf_static',
                                              lambda msg: self._receive_tf(msg, True, generation), static),
            ]
        self._timer.start()

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh()

    def set_status(self, cameras, connected=True):
        self._cameras, self._connected = cameras, bool(connected)
        if self.isVisible():
            self.refresh()

    def select_camera(self, camera):
        index = self._camera.findData(camera)
        if index >= 0:
            self._camera.setCurrentIndex(index)

    def refresh_theme(self):
        self._graph.update()
        if self.isVisible():
            self.refresh()

    def _receive_tf(self, message, is_static, generation):
        now = time.monotonic()
        with self._lock:
            if generation != self._generation or self._idle:
                return
            for transform in message.transforms:
                parent, child = transform.header.frame_id, transform.child_frame_id
                if parent and child and parent != child:
                    self._edges[child] = (parent, is_static, now, transform.transform)

    @staticmethod
    def format_matrix(matrix):
        if not isinstance(matrix, list) or len(matrix) != 4:
            return '—'
        lines = []
        for row in matrix:
            if not isinstance(row, list) or len(row) != 4:
                return '—'
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                   for value in row):
                return '—'
            lines.append('  '.join(f'{value:8.4f}' for value in row))
        return '\n'.join(lines)

    @classmethod
    def format_tf_matrix(cls, transform):
        # TF transforms are child-to-parent; translations are in meters.
        q, t = transform.rotation, transform.translation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not math.isfinite(norm) or norm < 1e-12 or not all(math.isfinite(v) for v in (t.x, t.y, t.z)):
            return '—'
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        return cls.format_matrix([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
            [0, 0, 0, 1],
        ])

    def _select_tf(self, current, previous):
        self._selected_tf = current.text(0) if current else ''
        self._update_selected()
        if self.isVisible():
            self.refresh()

    def _select_graph_frame(self, frame):
        self._selected_tf = frame
        self.refresh()
        items = self._tree.findItems(frame, QtCore.Qt.MatchExactly | QtCore.Qt.MatchRecursive, 0)
        if items:
            self._tree.scrollToItem(items[0])

    def _update_selected(self):
        with self._lock:
            edge = self._edges.get(self._selected_tf)
            is_root = bool(self._selected_tf and any(value[0] == self._selected_tf for value in self._edges.values()))
        if not edge:
            self._tf_direction.setText(f'{self._selected_tf} · root' if is_root else 'Select a frame')
            self._tf_matrix.setText('—')
            return
        parent, is_static, received, transform = edge
        label = 'static TF' if is_static else ('stale TF' if time.monotonic() - received > 5 else 'live TF')
        self._tf_direction.setText(f'{self._selected_tf} → {parent} · {label}')
        self._tf_matrix.setText(self.format_tf_matrix(transform))

    def refresh(self):
        name = self._camera.currentData()
        camera = self._cameras.get(name, {}) if self._connected else {}
        camera = camera if isinstance(camera, dict) else {}
        transform = camera.get('transform', {})
        transform = transform if isinstance(transform, dict) else {}
        labels = {'proposal': 'Proposal · not applied', 'review': 'Review snapshot · not applied',
                  'applying': 'Applying · not yet confirmed', 'accepted': 'Accepted calibration',
                  'previous': 'Previous accepted calibration'}
        self._source.setText(labels.get(transform.get('source'),
                                        'Waiting for estimate' if self._connected else 'Calibration offline'))
        quality = transform.get('quality', {})
        quality = quality if isinstance(quality, dict) else {}
        metrics = []
        for key, title, suffix, digits in (
                ('icp_fitness', 'ICP fitness', '', 3), ('icp_rmse_m', 'RMSE', ' m', 3),
                ('icp_matches', 'matches', '', 0), ('ransac_fitness', 'RANSAC fitness', '', 3),
                ('ransac_matches', 'RANSAC matches', '', 0)):
            value = quality.get(key)
            if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
                formatted = f'{value:.3f}' if digits else str(int(value))
                metrics.append(f'{title} {formatted}{suffix}')
        self._quality.setText('  ·  '.join(metrics) if metrics else 'No quality data')
        cloud = transform.get('cloud_frame', 'camera cloud')
        lidar = transform.get('lidar_frame', 'LiDAR')
        body = transform.get('body_frame', f'zed_{name}_camera_link')
        self._cloud_direction.setText(f'{cloud} → {lidar}')
        self._mount_direction.setText(f'{body} → base_link')
        self._cloud_matrix.setText(self.format_matrix(transform.get('cloud_to_lidar')))
        self._mount_matrix.setText(self.format_matrix(transform.get('mount_to_base')))
        with self._lock:
            edges = dict(self._edges)
        expanded = {}
        iterator = QtWidgets.QTreeWidgetItemIterator(self._tree)
        while iterator.value():
            item = iterator.value()
            expanded[item.text(0)] = item.isExpanded()
            iterator += 1
        scroll = self._tree.verticalScrollBar().value()
        blocker = QtCore.QSignalBlocker(self._tree)
        self._tree.clear()
        if not edges:
            self._tree_info.set_message('No live TF' if self.preview else 'Waiting for TF')
            self._graph.set_frames({}, '', set())
            self._update_selected()
            return
        self._tree_info.set_message('')
        children, frames = {}, set()
        for child, edge in edges.items():
            children.setdefault(edge[0], []).append(child)
            frames.update((child, edge[0]))
        visited = set()
        now = time.monotonic()
        app = QtWidgets.QApplication.instance()
        white = bool(app and app.property('vehicleHubWhiteTheme'))
        highlights = {'base_link', body, cloud, lidar, transform.get('preview_frame', f'hub_zed_{name}_cloud')}
        highlights = {frame for frame in highlights if isinstance(frame, str)}
        self._graph.set_frames(edges, self._selected_tf, highlights)

        def add(frame, parent):
            if frame in visited:
                return
            visited.add(frame)
            item = QtWidgets.QTreeWidgetItem(parent if parent else self._tree)
            item.setText(0, frame)
            if frame == self._selected_tf:
                self._tree.setCurrentItem(item)
            edge = edges.get(frame)
            if edge:
                stale = not edge[1] and now - edge[2] > 5
                item.setText(1, 'Static' if edge[1] else ('Stale' if stale else 'Live'))
                if stale:
                    item.setForeground(1, QtGui.QColor('#895314' if white else '#d7a36a'))
            if frame in highlights:
                item.setForeground(0, QtGui.QColor('#1c6399' if white else '#7bb7f4'))
                font = item.font(0)
                font.setBold(True)
                item.setFont(0, font)
            for child in sorted(children.get(frame, ())):
                add(child, item)
            item.setExpanded(expanded.get(frame, True))

        for frame in sorted(frames):
            if frame not in edges:
                add(frame, None)
        # Invalid TF cycles have no root; show them once without recursing forever.
        for frame in sorted(frames):
            if frame not in visited:
                add(frame, None)
        self._tree.verticalScrollBar().setValue(scroll)
        del blocker
        self._update_selected()
