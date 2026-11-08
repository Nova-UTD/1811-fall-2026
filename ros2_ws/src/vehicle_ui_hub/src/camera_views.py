# Camera tiles, the recording frame, and deliberate drag-to-swap behavior.
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


NAMES = ('Front', 'Left', 'Rear', 'Right', 'Panorama')
COLORS = ('#68c5e7', '#78d3a7', '#f0ad69', '#b49ae8', '#89aeff')
TOPICS = tuple(f'/zed_{name.lower()}/zed_node/rgb/color/rect/image' for name in NAMES[:4]) + (
    '/panorama/image',
)


def _theme(black, white):
    app = QtWidgets.QApplication.instance()
    return QtGui.QColor(white if app and app.property('vehicleHubWhiteTheme') else black)


class DragHandle(QtWidgets.QPushButton):
    # The rest of the tile stays interactive; only this handle starts a swap.
    def __init__(self, name, parent=None):
        super().__init__(parent)
        self.setObjectName('dragHandle' + name)
        self.setAccessibleName(f'Drag {name} to the main view')
        self.setToolTip('Drag to main view')
        self.setFixedSize(40, 40)
        self.setCursor(QtCore.Qt.OpenHandCursor)
        self.setAttribute(QtCore.Qt.WA_AcceptTouchEvents)
        self.handler = None
        self._pressed = False
        self._dragging = False
        self._start = self._last = QtCore.QPoint()

    def _begin(self, position):
        self._pressed, self._dragging = True, False
        self._start = self._last = position
        self.setCursor(QtCore.Qt.ClosedHandCursor)
        self.update()

    def _move(self, position):
        if not self._pressed:
            return
        self._last = position
        if not self._dragging and (position - self._start).manhattanLength() >= QtWidgets.QApplication.startDragDistance():
            self._dragging = True
            if self.handler:
                self.handler('begin', position)
        if self._dragging and self.handler:
            self.handler('move', position)

    def _end(self, position, canceled=False):
        if not self._pressed:
            return
        dragged = self._dragging
        self._pressed = self._dragging = False
        self.setCursor(QtCore.Qt.OpenHandCursor)
        self.update()
        if dragged and self.handler:
            self.handler('cancel' if canceled else 'drop', position)

    def event(self, event):
        if event.type() == QtCore.QEvent.TouchCancel:
            self._end(self._last, True)
            event.accept()
            return True
        if event.type() in (QtCore.QEvent.TouchBegin, QtCore.QEvent.TouchUpdate, QtCore.QEvent.TouchEnd):
            points = event.touchPoints()
            position = points[0].screenPos().toPoint() if points else self._last
            if len(points) > 1:
                self._end(position, True)
            elif event.type() == QtCore.QEvent.TouchBegin and points:
                self._begin(position)
            elif event.type() == QtCore.QEvent.TouchUpdate:
                self._move(position)
            elif event.type() == QtCore.QEvent.TouchEnd:
                self._end(position)
            event.accept()
            return True
        return super().event(event)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._begin(event.globalPos())
        event.accept()

    def mouseMoveEvent(self, event):
        if self._pressed and event.buttons() & QtCore.Qt.LeftButton:
            self._move(event.globalPos())
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._end(event.globalPos())
        event.accept()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 150 if self._pressed else 85), 1))
        painter.setBrush(QtGui.QColor(0, 0, 0, 220 if self._pressed else 165))
        painter.drawRoundedRect(self.rect().adjusted(1, 1, -1, -1), 5, 5)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor('#e2e4e6'))
        for x in (16, 23):
            for y in (13, 20, 27):
                painter.drawEllipse(QtCore.QPointF(x, y), 1.6, 1.6)


class RecordingViewport(QtWidgets.QFrame):
    def __init__(self, viewport, parent=None):
        super().__init__(parent)
        self.setObjectName('viewFrame')
        self._layout = QtWidgets.QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(0)
        self._banner = QtWidgets.QFrame()
        self._banner.setObjectName('recordingBanner')
        self._banner.setFixedHeight(26)
        self._banner.setStyleSheet('QFrame#recordingBanner { background: #000000; border: 0; }')
        banner_layout = QtWidgets.QHBoxLayout(self._banner)
        banner_layout.setContentsMargins(10, 2, 10, 2)
        self._title = QtWidgets.QLabel('Point cloud')
        self._title.setStyleSheet('color: #e1e4e6; font-size: 12px; background: transparent; border: 0;')
        self._title.hide()
        banner_layout.addWidget(self._title)
        banner_layout.addStretch()
        self._badge = QtWidgets.QLabel('<span style="color:#d86970">●</span>  REC')
        self._badge.setObjectName('recordingBadge')
        self._badge.setTextFormat(QtCore.Qt.RichText)
        self._badge.setAccessibleName('Recording in progress')
        self._badge.setStyleSheet('color: #e1e3e5; background: transparent; border: 0; '
                                  'padding: 0 2px; font-size: 11px; font-weight: 600;')
        self._badge.hide()
        banner_layout.addWidget(self._badge)
        self.drag_handle = DragHandle('PointCloud')
        self.drag_handle.hide()
        banner_layout.addWidget(self.drag_handle)
        self._banner.hide()
        self._layout.addWidget(self._banner)
        self._layout.addWidget(viewport, 1)
        self._compact = self._recording = False

    def set_recording(self, recording):
        recording = bool(recording)
        if self._recording == recording:
            return
        self._recording = recording
        # A quiet, one-pixel frame stays outside RViz's native OpenGL window.
        outline = 1 if recording else 0
        self._layout.setContentsMargins(outline, outline, outline, outline)
        self.setStyleSheet('QFrame#viewFrame { background: #854348; }' if recording else '')
        self._badge.setVisible(recording)
        self._banner.setVisible(recording or self._compact)

    def set_compact(self, compact):
        compact = bool(compact)
        if self._compact == compact:
            return
        self._compact = compact
        self._banner.setFixedHeight(46 if compact else 26)
        self._title.setVisible(compact)
        self.drag_handle.setVisible(compact)
        self._banner.setVisible(self._recording or compact)
        self.setMinimumHeight(112 if compact else 0)


class CameraTile(QtWidgets.QWidget):
    def __init__(self, name, accent, interactive=True, parent=None):
        super().__init__(parent)
        self.setObjectName('cameraFeed' + name)
        self.setMinimumHeight(112)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.name, self.accent, self.interactive = name, QtGui.QColor(accent), interactive
        self.main = False
        self.activate = None
        self.drag_handle = None
        self._image, self._error, self._rate = QtGui.QImage(), '', None
        self._received = 0.0
        self._pressed_at = QtCore.QPoint()
        self._hover, self._reveal = 0.0, 1.0
        if interactive:
            self.setCursor(QtCore.Qt.PointingHandCursor)
            self.setFocusPolicy(QtCore.Qt.StrongFocus)
            self.setAccessibleName(f'Expand {name} camera feed')
            self.drag_handle = DragHandle(name, self)
        self._hover_animation = self._animation(200, self._set_hover)
        self._reveal_animation = self._animation(300, self._set_reveal)

    def _animation(self, duration, handler):
        animation = QtCore.QVariantAnimation(self)
        animation.setDuration(duration)
        animation.setEasingCurve(QtCore.QEasingCurve.OutCubic)
        animation.valueChanged.connect(handler)
        return animation

    def _set_hover(self, value):
        self._hover = value
        self.update()

    def _set_reveal(self, value):
        self._reveal = value
        self.update()

    def set_main(self, main):
        self.main = bool(main)
        if self.drag_handle:
            self.drag_handle.setVisible(not main)
        self.setCursor(QtCore.Qt.ArrowCursor if main else QtCore.Qt.PointingHandCursor)
        self.update()

    def set_rate(self, rate):
        self._rate = rate
        self.setAccessibleDescription(f'Received {rate:.1f} Hz' if rate is not None else 'Rate unavailable')
        self.update()

    def is_live(self):
        return bool(self._received and time.monotonic() - self._received < 3 and not self._image.isNull())

    def current_image(self):
        return self._image if self.is_live() else QtGui.QImage()

    def set_frame(self, image):
        was_live = self.is_live()
        self._image, self._error, self._received = image, '', time.monotonic()
        if not was_live:
            self._reveal_animation.stop()
            self._reveal_animation.setStartValue(0.0)
            self._reveal_animation.setEndValue(1.0)
            self._reveal_animation.start()
        self.update()

    def clear_frame(self):
        self._image, self._error, self._received = QtGui.QImage(), '', 0.0
        self.update()

    def set_error(self, error):
        self._image, self._error, self._received = QtGui.QImage(), error, time.monotonic()
        self.update()

    def event(self, event):
        if self.interactive and not self.main and event.type() in (QtCore.QEvent.Enter, QtCore.QEvent.Leave):
            self._hover_animation.stop()
            self._hover_animation.setStartValue(self._hover)
            self._hover_animation.setEndValue(1.0 if event.type() == QtCore.QEvent.Enter else 0.0)
            self._hover_animation.start()
        return super().event(event)

    def mousePressEvent(self, event):
        if self.interactive and not self.main and event.button() == QtCore.Qt.LeftButton:
            self._pressed_at = event.pos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if (self.interactive and not self.main and event.button() == QtCore.Qt.LeftButton and
                (event.pos() - self._pressed_at).manhattanLength() < 12 and self.activate):
            self.activate()
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if self.interactive and not self.main and event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Space) and self.activate:
            self.activate()
            event.accept()
            return
        super().keyPressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.drag_handle:
            self.drag_handle.move(self.width() - self.drag_handle.width() - 6,
                                  self.height() - self.drag_handle.height() - 6)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        bounds = QtCore.QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        clip = QtGui.QPainterPath()
        clip.addRoundedRect(bounds, 5, 5)
        painter.setClipPath(clip)
        live = self.is_live()
        painter.fillRect(self.rect(), QtGui.QColor('#080808') if live else _theme('#080808', '#f3f3f3'))
        if live:
            size = self._image.size().scaled(self.rect().size(), QtCore.Qt.KeepAspectRatio)
            target = QtCore.QRect(QtCore.QPoint(self.rect().center().x() - size.width() // 2,
                                               self.rect().center().y() - size.height() // 2), size)
            painter.setOpacity(self._reveal)
            painter.drawImage(target, self._image)
            painter.setOpacity(1.0)
            top = QtGui.QLinearGradient(0, 0, 0, 44)
            top.setColorAt(0, QtGui.QColor(0, 0, 0, 210))
            top.setColorAt(1, QtGui.QColor(0, 0, 0, 0))
            painter.fillRect(QtCore.QRect(0, 0, self.width(), 44), top)
        else:
            painter.setPen(_theme('#898989', '#606467'))
            font = painter.font()
            font.setPixelSize(13)
            painter.setFont(font)
            invalid = bool(self._error and self._received and time.monotonic() - self._received < 3)
            message = self._error if invalid else ('No recent image' if self._received else 'Waiting for image')
            painter.drawText(self.rect().adjusted(12, 30, -12, 0), QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, message)
        app = QtWidgets.QApplication.instance()
        white = bool(app and app.property('vehicleHubWhiteTheme'))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(self.accent.darker(160) if not live and white else self.accent)
        painter.drawEllipse(QtCore.QPointF(14, 19), 3, 3)
        font = painter.font()
        font.setPixelSize(13)
        font.setWeight(QtGui.QFont.Medium)
        painter.setFont(font)
        painter.setPen(QtGui.QColor('#eff3f4') if live else _theme('#eff3f4', '#252525'))
        painter.drawText(QtCore.QRect(23, 6, self.width() - 100, 26), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, self.name)
        if live:
            state = f'{self._rate:.1f} Hz' if self._rate is not None else 'Live'
            painter.setPen(QtGui.QColor('#dddddd'))
            painter.drawText(QtCore.QRect(self.width() - 82, 6, 70, 26), QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, state)
        painter.setClipping(False)
        painter.setBrush(QtCore.Qt.NoBrush)
        shade = round((200 if white else 47) + (-45 if white else 36) * self._hover)
        border = _theme('#a5a5a5', '#666666') if self.hasFocus() else QtGui.QColor(shade, shade, shade)
        painter.setPen(QtGui.QPen(border, 1))
        painter.drawRoundedRect(bounds, 5, 5)


class ReceiveRate:
    def __init__(self):
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self._count, self._last, self._sampled, self._value = 0, 0.0, time.monotonic(), None

    def record(self):
        with self._lock:
            self._last = time.monotonic()
            self._count += 1

    def sample(self):
        with self._lock:
            now = time.monotonic()
            elapsed = now - self._sampled
            if elapsed >= 0.5:
                self._value = self._count / elapsed if self._last and now - self._last <= 3 else None
                self._count, self._sampled = 0, now
            return self._value


class CameraGallery(QtWidgets.QWidget):
    _frame_ready = QtCore.pyqtSignal(int, int, QtGui.QImage, str)

    def __init__(self, node, preview=False, parent=None):
        super().__init__(parent)
        self.node, self.preview = node, preview
        self.setObjectName('cameraGallery')
        self._lock = threading.Lock()
        self._group = MutuallyExclusiveCallbackGroup() if node else None
        self._active, self._side_visible, self._surround_visible = False, True, True
        self._generation, self._main_index, self._focus_index, self._restore_slot = 0, -1, -1, -1
        self._pending = [False] * 5
        self._last_frame_ns = [0] * 5
        self._subscriptions = [None] * 5
        self._rates = [ReceiveRate() for _ in NAMES]
        self._tiles, self._slots = [], []
        self._main_view = self._point_cloud = self._main_changed = None
        self._focus_dialog = self._focus_tile = self._focus_title = None
        self._frame_ready.connect(self._apply_frame, QtCore.Qt.QueuedConnection)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)
        heading = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel('Views')
        title.setObjectName('galleryTitle')
        heading.addWidget(title)
        heading.addStretch()
        self._live_count = QtWidgets.QLabel('0 / 5 LIVE')
        self._live_count.setObjectName('feedCount')
        self._live_count.setVisible(not preview)
        heading.addWidget(self._live_count)
        layout.addLayout(heading)
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(8)
        for index, (name, color, topic) in enumerate(zip(NAMES, COLORS, TOPICS)):
            tile = CameraTile(name, color)
            tile.setToolTip(topic)
            tile.activate = lambda index=index: self._open_focus(index)
            tile.drag_handle.handler = lambda phase, position, tile=tile: self._handle_drag(tile, phase, position)
            slot = QtWidgets.QFrame()
            slot.setObjectName('viewSlot' + name)
            slot_layout = QtWidgets.QVBoxLayout(slot)
            slot_layout.setContentsMargins(0, 0, 0, 0)
            slot_layout.addWidget(tile)
            if index < 4:
                grid.addWidget(slot, 1 + index // 2, index % 2)
            else:
                grid.addWidget(slot, 0, 0, 1, 2)
            self._tiles.append(tile)
            self._slots.append(slot)
        for row in range(3):
            grid.setRowStretch(row, 1)
        layout.addLayout(grid, 1)
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._refresh_rates)

    def set_main_view(self, container, point_cloud, on_changed=None):
        self._main_view, self._point_cloud, self._main_changed = container, point_cloud, on_changed
        point_cloud.drag_handle.handler = lambda phase, position: self._handle_drag(point_cloud, phase, position)

    def has_main_camera(self):
        with self._lock:
            return self._main_index >= 0

    def _slot_of(self, view):
        for index, slot in enumerate(self._slots):
            if slot.layout().count() and slot.layout().itemAt(0).widget() is view:
                return index
        return -1

    def _highlight(self, active):
        if not self._main_view or not self._main_view.layout():
            return
        margin = 3 if active else 0
        self._main_view.layout().setContentsMargins(margin, margin, margin, margin)
        self._main_view.setStyleSheet('QFrame#mainViewSlot { background: #8ab4ce; }' if active else '')

    def _handle_drag(self, source, phase, position):
        if not self._main_view or not self._surround_visible:
            return
        over = self._main_view.isVisible() and self._main_view.rect().contains(self._main_view.mapFromGlobal(position))
        if phase in ('begin', 'move'):
            self._highlight(over)
        else:
            self._highlight(False)
            if phase == 'drop' and over:
                self._swap_slot(self._slot_of(source))

    def _swap_slot(self, index):
        if not self._main_view or not self._point_cloud or not 0 <= index < len(self._slots):
            return
        main_layout, side_layout = self._main_view.layout(), self._slots[index].layout()
        if not main_layout or not side_layout or not main_layout.count() or not side_layout.count():
            return
        main, side = main_layout.itemAt(0).widget(), side_layout.itemAt(0).widget()
        if not main or not side or main is side:
            return
        # Reparent the existing widgets rather than creating more subscriptions.
        main_layout.removeWidget(main)
        side_layout.removeWidget(side)
        main_layout.addWidget(side)
        side_layout.addWidget(main)
        side.show()
        main.show()
        selected = -1
        for tile_index, tile in enumerate(self._tiles):
            tile.set_main(tile is side)
            if tile is side:
                selected = tile_index
        with self._lock:
            self._main_index = selected
        self._point_cloud.set_compact(side is not self._point_cloud)
        if self._main_changed:
            self._main_changed(side is self._point_cloud)

    def set_surround_visible(self, visible):
        if self._surround_visible == visible:
            return
        self._surround_visible = visible
        self._highlight(False)
        if not visible:
            self._restore_slot = self._slot_of(self._point_cloud)
            if self._restore_slot >= 0:
                self._swap_slot(self._restore_slot)
        elif self._restore_slot >= 0:
            index, self._restore_slot = self._restore_slot, -1
            self._swap_slot(index)

    def _refresh_indicators(self):
        if not self.preview:
            self._live_count.setText(f'{sum(tile.is_live() for tile in self._tiles)} / 5 live')

    def _refresh_rates(self):
        if not self._active:
            return
        for index, tile in enumerate(self._tiles):
            rate = self._rates[index].sample()
            tile.set_rate(rate)
            if self._focus_index == index and self._focus_dialog and self._focus_dialog.isVisible():
                self._focus_tile.set_rate(rate)
        self._refresh_indicators()

    def _open_focus(self, index):
        if self._focus_dialog is None:
            self._focus_dialog = QtWidgets.QDialog(self)
            self._focus_dialog.setObjectName('cameraFocusDialog')
            self._focus_dialog.setWindowTitle('Camera view')
            self._focus_dialog.resize(1000, 650)
            self._focus_dialog.setMinimumSize(620, 380)
            layout = QtWidgets.QVBoxLayout(self._focus_dialog)
            layout.setContentsMargins(24, 20, 24, 24)
            layout.setSpacing(14)
            heading = QtWidgets.QHBoxLayout()
            self._focus_title = QtWidgets.QLabel()
            self._focus_title.setObjectName('galleryTitle')
            heading.addWidget(self._focus_title)
            heading.addStretch()
            close = QtWidgets.QPushButton('Close')
            close.setObjectName('windowButton')
            close.clicked.connect(self._focus_dialog.accept)
            heading.addWidget(close)
            layout.addLayout(heading)
            self._focus_tile = CameraTile('Camera', '#89aeff', interactive=False)
            self._focus_tile.setObjectName('cameraFocusFeed')
            layout.addWidget(self._focus_tile, 1)
            self._focus_dialog.finished.connect(self._close_focus)
        with self._lock:
            self._focus_index = index
        self._focus_title.setText(NAMES[index])
        self._focus_tile.name, self._focus_tile.accent = NAMES[index], QtGui.QColor(COLORS[index])
        self._focus_tile.clear_frame()
        image = self._tiles[index].current_image()
        if not image.isNull():
            self._focus_tile.set_frame(image)
        host = self.window()
        self._focus_dialog.resize(min(1100, max(620, host.width() - 80)), min(720, max(380, host.height() - 80)))
        self._focus_dialog.open()
        self._focus_dialog.raise_()

    def _close_focus(self, result):
        with self._lock:
            self._focus_index = -1
        self._focus_tile.clear_frame()

    def set_active(self, active, side_visible=True, preserve_frames=False):
        active, side_visible = bool(active), bool(side_visible)
        with self._lock:
            if self._active == active and self._side_visible == side_visible:
                return
            self._active, self._side_visible = active, side_visible
            self._generation += 1
            generation = self._generation
            selected = self._main_index
            self._last_frame_ns = [0] * 5
        if active:
            self._timer.start()
        else:
            self._timer.stop()
        if not active and self._focus_dialog and self._focus_dialog.isVisible():
            self._focus_dialog.reject()
        for index, tile in enumerate(self._tiles):
            if self.node and self._subscriptions[index] is not None:
                self.node.destroy_subscription(self._subscriptions[index])
            self._subscriptions[index] = None
            if not preserve_frames:
                tile.clear_frame()
            tile.set_rate(None)
            self._rates[index].reset()
            if active and self.node and (side_visible or selected == index):
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                self._subscriptions[index] = self.node.create_subscription(
                    Image, TOPICS[index],
                    lambda message, index=index, generation=generation: self._receive(index, generation, message),
                    qos, callback_group=self._group)
        self._refresh_indicators()

    def clear_inactive_frames(self):
        # Keep the last images while their panel slides out, then free them.
        # A quick return to Surround must not clear newly resumed feeds.
        with self._lock:
            if self._active:
                return
        for tile in self._tiles:
            tile.clear_frame()
        self._refresh_indicators()

    @staticmethod
    def decode_image(message, bounds):
        formats = {
            'rgb8': (3, QtGui.QImage.Format_RGB888),
            'bgr8': (3, QtGui.QImage.Format_BGR888),
            'rgba8': (4, QtGui.QImage.Format_RGBA8888),
            'bgra8': (4, QtGui.QImage.Format_ARGB32),
            'mono8': (1, QtGui.QImage.Format_Grayscale8),
            '8UC1': (1, QtGui.QImage.Format_Grayscale8),
        }
        encoding = formats.get(message.encoding)
        if (not encoding or not message.width or not message.height or bounds.isEmpty() or
                max(message.width, message.height, message.step) > 2 ** 31 - 1 or
                message.width * encoding[0] > message.step or message.step * message.height > len(message.data)):
            return QtGui.QImage()
        image = QtGui.QImage(memoryview(message.data), message.width, message.height, message.step, encoding[1])
        # The ROS message can be released after the callback, so own the pixels.
        return image.scaled(bounds, QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation).copy()

    def _receive(self, index, generation, message):
        with self._lock:
            if not self._active or generation != self._generation:
                return
        self._rates[index].record()
        now = time.monotonic_ns()
        with self._lock:
            if (not self._active or generation != self._generation or
                    now - self._last_frame_ns[index] < 100_000_000 or self._pending[index]):
                return
            self._last_frame_ns[index] = now
            self._pending[index] = True
            focused = self._focus_index == index or self._main_index == index
        bounds = (QtCore.QSize(1600, 500) if index == 4 else QtCore.QSize(1280, 720)) if focused else (
            QtCore.QSize(960, 300) if index == 4 else QtCore.QSize(480, 270))
        try:
            image = self.decode_image(message, bounds)
        except (TypeError, ValueError, OverflowError):
            image = QtGui.QImage()
        error = f'Unsupported or invalid image\n{message.encoding}'
        self._frame_ready.emit(index, generation, image, error)

    @QtCore.pyqtSlot(int, int, QtGui.QImage, str)
    def _apply_frame(self, index, generation, image, error):
        with self._lock:
            self._pending[index] = False
            active = self._active and generation == self._generation
            focus = self._focus_index == index
        if not active:
            return
        tile = self._tiles[index]
        was_live = tile.is_live()
        tile.set_error(error) if image.isNull() else tile.set_frame(image)
        if focus and self._focus_dialog and self._focus_dialog.isVisible():
            self._focus_tile.set_error(error) if image.isNull() else self._focus_tile.set_frame(image)
        if was_live != tile.is_live():
            self._refresh_indicators()
