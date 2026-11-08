# Discover ROS topics and record selected topics without leaving orphaned bags.
import os
import signal
import sys


if __name__ == '__main__' and len(sys.argv) >= 4 and sys.argv[1] == '--record-guard':
    # This short-lived helper runs in a fresh process, before execing ros2. It
    # can safely set Linux's parent-death signal without forking Python threads.
    if sys.platform.startswith('linux'):
        import ctypes
        parent = int(sys.argv[2])
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(1, signal.SIGINT, 0, 0, 0) == 0 and os.getppid() != parent:
            os.kill(os.getpid(), signal.SIGINT)
    os.execvp(sys.argv[3], sys.argv[3:])

import datetime
import re
import threading
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def _label(text, name):
    label = QtWidgets.QLabel(text)
    label.setObjectName(name)
    return label


def _theme(black, white):
    app = QtWidgets.QApplication.instance()
    return white if app and app.property('vehicleHubWhiteTheme') else black


def _last_line(output):
    lines = output.decode(errors='replace').strip().splitlines()
    return lines[-1].strip()[:350] if lines else ''


class BagNameDialog(QtWidgets.QDialog):
    # A small keyboard makes the optional name usable on a touchscreen even
    # when the desktop has no system keyboard configured.
    def __init__(self, value, parent=None):
        super().__init__(parent)
        self.setObjectName('bagNameDialog')
        self.setWindowTitle('Bag name')
        self.resize(640, 330)
        self.setStyleSheet(
            'QDialog#bagNameDialog { background: ' + _theme('#101010', '#ffffff') + '; }'
            'QLabel { color: ' + _theme('#eeeeee', '#202124') + '; font-size: 15px; }'
            'QLineEdit { color: ' + _theme('#eeeeee', '#202124') + '; background: ' +
            _theme('#1a1a1a', '#f3f3f3') + '; border: 1px solid ' + _theme('#555555', '#aaaaaa') +
            '; border-radius: 5px; padding: 8px; }'
            'QPushButton { color: ' + _theme('#eeeeee', '#202124') + '; background: ' +
            _theme('#262626', '#ededed') + '; border: 1px solid ' + _theme('#484848', '#c7c7c7') +
            '; border-radius: 4px; }'
            'QPushButton:pressed, QPushButton:checked { background: ' + _theme('#555555', '#d3d3d3') + '; }'
            'QPushButton#bagNameDone { color: #ffffff; background: #3468d4; border-color: #3468d4; }'
            'QPushButton#bagNameDone:pressed { background: #2854b3; }')
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(_label('Bag name', 'sectionTitle'))
        self.name_input = QtWidgets.QLineEdit(value)
        self.name_input.setObjectName('bagNameInput')
        self.name_input.setMaxLength(120)
        self.name_input.setMinimumHeight(42)
        layout.addWidget(self.name_input)
        letters = []
        for characters in ('1234567890', 'qwertyuiop', 'asdfghjkl', 'zxcvbnm'):
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(4)
            for character in characters:
                key = QtWidgets.QPushButton(character)
                key.setObjectName('bagNameKey')
                key.setMinimumHeight(40)
                key.setFocusPolicy(QtCore.Qt.NoFocus)
                key.clicked.connect(lambda checked=False, key=key: self.name_input.insert(key.text()))
                row.addWidget(key, 1)
                if character.isalpha():
                    letters.append(key)
            layout.addLayout(row)
        actions = QtWidgets.QHBoxLayout()
        shift = QtWidgets.QPushButton('Shift')
        shift.setCheckable(True)

        def set_shift(upper):
            for key in letters:
                key.setText(key.text().upper() if upper else key.text().lower())
        shift.toggled.connect(set_shift)
        for label, callback, stretch in (
                ('Shift', None, 1), ('-', lambda: self.name_input.insert('-'), 1),
                ('_', lambda: self.name_input.insert('_'), 1), ('Space', lambda: self.name_input.insert(' '), 3),
                ('Back', self.name_input.backspace, 1)):
            button = shift if label == 'Shift' else QtWidgets.QPushButton(label)
            button.setObjectName('bagNameKey')
            button.setMinimumHeight(40)
            button.setFocusPolicy(QtCore.Qt.NoFocus)
            if callback:
                button.clicked.connect(callback)
            actions.addWidget(button, stretch)
        layout.addLayout(actions)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Ok)
        done = buttons.button(QtWidgets.QDialogButtonBox.Ok)
        done.setText('Done')
        done.setObjectName('bagNameDone')
        for button in buttons.buttons():
            button.setIcon(QtGui.QIcon())
            button.setMinimumWidth(82)
            button.setMinimumHeight(42)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self.name_input.returnPressed.connect(self.accept)
        self.name_input.setFocus()


class RosbagRecorder(QtWidgets.QWidget):
    def __init__(self, node, preview=False, parent=None):
        super().__init__(parent)
        self.node, self.preview = node, preview
        self.setObjectName('recordingPanel')
        self._state, self._active, self._directory = 'idle', False, self._default_directory()
        self._selected, self._discovered = set(), {}
        self._bag_path, self._recording_topics, self._output = '', [], bytearray()
        self._request_id, self._stop_requested = 0, False
        self._probe_stop_message, self._probe_stop_error = '', False
        self._recording_started = 0.0
        self._status_callback = self._stopped_callback = None
        self._last_header, self._last_recording = '', False
        self._rate_subscription = None
        self._monitored_topic = self._monitored_type = ''
        self._rate_lock = threading.Lock()
        self._rate_count, self._rate_generation, self._rate_started = 0, 0, 0.0
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(12)
        heading = QtWidgets.QHBoxLayout()
        heading.addWidget(_label('Record bag', 'drawerTitle'))
        heading.addStretch()
        self.close_button = QtWidgets.QPushButton('×')
        self.close_button.setObjectName('closeButton')
        self.close_button.setAccessibleName('Close recording panel')
        self.close_button.setFixedSize(44, 44)
        heading.addWidget(self.close_button)
        layout.addLayout(heading)
        self._status_card = QtWidgets.QFrame()
        self._status_card.setObjectName('recordingStatusCard')
        status_layout = QtWidgets.QHBoxLayout(self._status_card)
        status_layout.setContentsMargins(14, 10, 14, 10)
        status_layout.setSpacing(10)
        self._status_dot = QtWidgets.QFrame()
        self._status_dot.setObjectName('recordingStatusDot')
        self._status_dot.setFixedSize(10, 10)
        status_layout.addWidget(self._status_dot)
        self._status_label = _label('Preview only' if preview else 'Ready', 'recordingState')
        status_layout.addWidget(self._status_label, 1)
        layout.addWidget(self._status_card)
        layout.addWidget(_label('Save to', 'sectionTitle'))
        location = QtWidgets.QHBoxLayout()
        self._directory_label = _label(self._directory, 'recordingDirectory')
        self._directory_label.setWordWrap(True)
        self._directory_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        location.addWidget(self._directory_label, 1)
        self._choose = QtWidgets.QPushButton('Change')
        self._choose.setObjectName('secondaryButton')
        self._choose.setMinimumHeight(42)
        self._choose.clicked.connect(self._choose_directory)
        location.addWidget(self._choose)
        layout.addLayout(location)
        note = _label('Use a mounted folder to keep recordings.', 'statusHint')
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addWidget(_label('Bag name', 'sectionTitle'))
        name_row = QtWidgets.QHBoxLayout()
        self._name = QtWidgets.QLineEdit()
        self._name.setObjectName('recordingName')
        self._name.setPlaceholderText('Optional')
        self._name.setMaxLength(120)
        self._name.setClearButtonEnabled(True)
        self._name.setMinimumHeight(42)
        name_row.addWidget(self._name, 1)
        self._name_keyboard = QtWidgets.QPushButton('Keyboard')
        self._name_keyboard.setObjectName('secondaryButton')
        self._name_keyboard.setMinimumHeight(42)
        self._name_keyboard.clicked.connect(self._edit_name_on_screen)
        name_row.addWidget(self._name_keyboard)
        layout.addLayout(name_row)
        layout.addWidget(_label('Topics', 'sectionTitle'))
        filters = QtWidgets.QHBoxLayout()
        self._category = QtWidgets.QComboBox()
        self._category.setObjectName('recordingCategory')
        self._category.addItems(['All topics', 'LiDAR', 'Cameras', 'IMU', 'Transforms'])
        self._category.setMinimumHeight(42)
        self._search = QtWidgets.QLineEdit()
        self._search.setObjectName('recordingSearch')
        self._search.setPlaceholderText('Search topics')
        self._search.setClearButtonEnabled(True)
        self._search.setMinimumHeight(42)
        filters.addWidget(self._category)
        filters.addWidget(self._search, 1)
        layout.addLayout(filters)
        self._category.currentIndexChanged.connect(self._apply_filter)
        self._search.textChanged.connect(self._apply_filter)
        selection = QtWidgets.QHBoxLayout()
        self._select_visible = QtWidgets.QPushButton('Select visible')
        self._select_visible.setObjectName('textButton')
        self._clear = QtWidgets.QPushButton('Clear')
        self._clear.setObjectName('textButton')
        self._selection_label = _label('0 selected', 'statusHint')
        selection.addWidget(self._select_visible)
        selection.addWidget(self._clear)
        selection.addStretch()
        selection.addWidget(self._selection_label)
        layout.addLayout(selection)
        self._topics = QtWidgets.QTreeWidget()
        self._topics.setObjectName('recordingTopics')
        self._topics.setColumnCount(3)
        self._topics.setHeaderLabels(['Topic', 'Type', 'Pubs'])
        self._topics.setRootIsDecorated(False)
        self._topics.setAlternatingRowColors(False)
        self._topics.setMinimumHeight(260)
        self._topics.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self._topics.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self._topics.header().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        layout.addWidget(self._topics)
        self._topics.itemChanged.connect(self._selection_changed)
        self._topics.currentItemChanged.connect(lambda *_: self._monitor_selected_topic())
        self._topic_detail = _label('', 'recordingTopicDetail')
        self._topic_detail.hide()
        self._topic_detail.setWordWrap(True)
        self._topic_detail.setTextFormat(QtCore.Qt.PlainText)
        self._topic_detail.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self._topic_detail)
        self._select_visible.clicked.connect(lambda: self._set_checks(True))
        self._clear.clicked.connect(lambda: self._set_checks(False))
        note = _label('Waiting: no publisher. Include /tf and /tf_static for transforms. Images and clouds use more space.',
                      'statusHint')
        note.setWordWrap(True)
        layout.addWidget(note)
        controls = QtWidgets.QHBoxLayout()
        self._start = QtWidgets.QPushButton('Start')
        self._start.setObjectName('recordingStart')
        self._start.setMinimumHeight(52)
        self._stop = QtWidgets.QPushButton('Stop')
        self._stop.setObjectName('recordingStop')
        self._stop.setMinimumHeight(52)
        controls.addWidget(self._start)
        controls.addWidget(self._stop)
        layout.addLayout(controls)
        self._start.clicked.connect(self.start_recording)
        self._stop.clicked.connect(self.stop_recording)
        self._message = _label('' if preview else 'Select topics to start.', 'recordingMessage')
        self._message.setWordWrap(True)
        self._message.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self._message)
        layout.addStretch()
        self._discovery_timer = QtCore.QTimer(self)
        self._discovery_timer.setInterval(2000)
        self._discovery_timer.timeout.connect(self._refresh_topics)
        self._rate_timer = QtCore.QTimer(self)
        self._rate_timer.setInterval(1000)
        self._rate_timer.timeout.connect(self._update_topic_rate)
        self._status_timer = QtCore.QTimer(self)
        self._status_timer.setInterval(250)
        self._status_timer.timeout.connect(self._tick_status)
        self._status_timer.start()
        self._probe, self._recorder = QtCore.QProcess(self), QtCore.QProcess(self)
        for process in (self._probe, self._recorder):
            process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self._probe.errorOccurred.connect(self._probe_error)
        self._probe.finished.connect(self._probe_finished)
        self._recorder.started.connect(self._recorder_started)
        self._recorder.readyReadStandardOutput.connect(self._read_output)
        self._recorder.errorOccurred.connect(self._recorder_error)
        self._recorder.finished.connect(self._recorder_finished)
        self._update_controls()
        self._refresh_topics()

    @staticmethod
    def _default_directory():
        if Path('/vehicle_1811/ros2_ws').is_dir():
            return '/vehicle_1811/recordings'
        root = Path.cwd()
        for candidate in (root, *list(root.parents)[:7]):
            if (candidate / 'ros2_ws/src/vehicle_ui_hub').is_dir():
                return str(candidate / 'recordings')
        return str(Path.home() / 'recordings')

    def _choose_directory(self):
        initial = self._directory if os.path.isdir(self._directory) else os.path.dirname(self._directory)
        selected = QtWidgets.QFileDialog.getExistingDirectory(self, 'Choose a recording folder', initial)
        if selected:
            self.set_directory(selected)

    def _edit_name_on_screen(self):
        dialog = BagNameDialog(self._name.text(), self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            self._name.setText(dialog.name_input.text())

    def set_directory(self, directory):
        if not self.is_busy() and directory:
            self._directory = directory
            self._directory_label.setText(directory)
            self._directory_label.setToolTip(directory)

    def is_busy(self):
        return self._state != 'idle'

    def set_status_callback(self, callback):
        self._status_callback = callback
        if callback:
            callback(self._last_header, self._state == 'recording')

    def stop_and_then(self, callback):
        if not self.is_busy():
            QtCore.QTimer.singleShot(0, callback)
            return
        self._stopped_callback = callback
        self.stop_recording()

    def set_active(self, active):
        self._active = bool(active)
        if active:
            self._refresh_topics()
            self._discovery_timer.start()
        else:
            self._discovery_timer.stop()
        self._monitor_selected_topic()

    def set_idle(self, idle):
        if idle:
            self.set_active(False)
            self._status_timer.stop()
        else:
            self._status_timer.start()
            self._update_status()

    def refresh_theme(self):
        self._rebuild_topics()
        self._update_status()
        self._message.setStyleSheet('color: ' + _theme('#f4bd73', '#895314') + ';'
                                    if self._message.property('recordingError') else '')

    def _selection_changed(self, item, column):
        if column:
            return
        name = item.data(0, QtCore.Qt.UserRole)
        if item.checkState(0) == QtCore.Qt.Checked:
            self._selected.add(name)
        else:
            self._selected.discard(name)
        self._update_controls()

    def _set_checks(self, checked):
        for index in range(self._topics.topLevelItemCount()):
            item = self._topics.topLevelItem(index)
            if not checked or not item.isHidden():
                item.setCheckState(0, QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked)

    def _refresh_topics(self):
        topics = {'/ouster/points': ('sensor_msgs/msg/PointCloud2', 0),
                  '/ouster/imu': ('sensor_msgs/msg/Imu', 0), '/panorama/image': ('sensor_msgs/msg/Image', 0),
                  '/tf': ('tf2_msgs/msg/TFMessage', 0), '/tf_static': ('tf2_msgs/msg/TFMessage', 0),
                  '/odometry': ('nav_msgs/msg/Odometry', 0)}
        for side in ('front', 'left', 'rear', 'right'):
            prefix = f'/zed_{side}/zed_node/'
            topics[prefix + 'rgb/color/rect/image'] = ('sensor_msgs/msg/Image', 0)
            topics[prefix + 'point_cloud/cloud_registered'] = ('sensor_msgs/msg/PointCloud2', 0)
        if self.preview:
            topics = {name: (kind, 1) for name, (kind, _) in topics.items()}
        elif self.node:
            try:
                for name, types in self.node.get_topic_names_and_types():
                    topics[name] = (', '.join(types), self.node.count_publishers(name))
            except Exception as error:
                self._message.setText(f'Could not refresh topics: {error}')
        for name in self._selected:
            topics.setdefault(name, ('', 0))
        if topics != self._discovered or not self._topics.topLevelItemCount():
            self._discovered = topics
            self._rebuild_topics()
            self._monitor_selected_topic()

    def _rebuild_topics(self):
        scroll = self._topics.verticalScrollBar().value()
        current = self._topics.currentItem()
        current = current.data(0, QtCore.Qt.UserRole) if current else ''
        blocker = QtCore.QSignalBlocker(self._topics)
        self._topics.setUpdatesEnabled(False)
        self._topics.clear()
        for name, (kind, publishers) in sorted(self._discovered.items()):
            item = QtWidgets.QTreeWidgetItem(self._topics)
            item.setText(0, name)
            item.setData(0, QtCore.Qt.UserRole, name)
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(0, QtCore.Qt.Checked if name in self._selected else QtCore.Qt.Unchecked)
            item.setSizeHint(0, QtCore.QSize(0, 38))
            item.setText(1, kind.rsplit('/', 1)[-1])
            item.setToolTip(0, name + '\n' + kind)
            item.setToolTip(1, kind)
            item.setText(2, str(publishers) if publishers else 'Waiting')
            item.setForeground(2, QtGui.QColor(_theme('#8ac9aa', '#176d45') if publishers else _theme('#8b98a5', '#5b6164')))
            if name == current:
                self._topics.setCurrentItem(item)
        self._apply_filter()
        self._topics.setUpdatesEnabled(True)
        self._topics.verticalScrollBar().setValue(scroll)
        del blocker
        self._update_controls()

    def _apply_filter(self, *args):
        query, category = self._search.text().strip().lower(), self._category.currentIndex()
        for index in range(self._topics.topLevelItemCount()):
            item = self._topics.topLevelItem(index)
            name = item.data(0, QtCore.Qt.UserRole)
            kind = self._discovered.get(name, ('', 0))[0]
            matches = (category == 0 or
                       category == 1 and (name.startswith('/ouster/') or name.startswith('/lidar')) or
                       category == 2 and (name.startswith('/zed_') or name.startswith('/panorama/')) or
                       category == 3 and ('/Imu' in kind or 'imu' in name.lower()) or
                       category == 4 and name in ('/tf', '/tf_static'))
            item.setHidden(not matches or (query not in name.lower() and query not in kind.lower()))

    def _monitor_selected_topic(self):
        item = self._topics.currentItem() if self._active else None
        name = item.data(0, QtCore.Qt.UserRole) if item else ''
        kind, publishers = self._discovered.get(name, ('', 0))
        can_monitor = (self._state == 'idle' and not self.preview and self.node and name and publishers and kind and ',' not in kind)
        if can_monitor and self._rate_subscription and name == self._monitored_topic and kind == self._monitored_type:
            return
        self._rate_timer.stop()
        with self._rate_lock:
            self._rate_generation += 1
            generation = self._rate_generation
            self._rate_count = 0
        if self._rate_subscription and self.node:
            self.node.destroy_subscription(self._rate_subscription)
        self._rate_subscription = None
        self._monitored_topic, self._monitored_type = name, kind
        if not name:
            self._topic_detail.hide()
            return
        self._topic_detail.setText(f'{name}\n{kind}\nRate: —')
        self._topic_detail.show()
        if not can_monitor:
            return
        try:
            def receive(_message):
                with self._rate_lock:
                    if generation == self._rate_generation:
                        self._rate_count += 1
            # Raw serialized messages avoid deserializing large sensor streams.
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._rate_subscription = self.node.create_subscription(get_message(kind), name, receive, qos, raw=True)
            self._rate_started = time.monotonic()
            self._rate_timer.start()
        except Exception as error:
            self._topic_detail.setText(f'{name}\n{kind}\nRate unavailable')
            self.node.get_logger().warning(f'Could not monitor {name}: {error}')

    def _update_topic_rate(self):
        with self._rate_lock:
            count, self._rate_count = self._rate_count, 0
        now, previous = time.monotonic(), self._rate_started
        self._rate_started = now
        rate = f'{count / (now - previous):.1f} Hz' if count and now > previous else '—'
        self._topic_detail.setText(f'{self._monitored_topic}\n{self._monitored_type}\nRate: {rate}')

    def _update_controls(self):
        idle = self._state == 'idle'
        self._selection_label.setText(f'{len(self._selected)} selected')
        for widget in (self._choose, self._name, self._name_keyboard, self._category, self._search,
                       self._topics, self._select_visible):
            widget.setEnabled(idle)
        self._clear.setEnabled(idle and bool(self._selected))
        self._start.setEnabled(idle and not self.preview and bool(self._selected))
        self._stop.setEnabled(not idle and self._state != 'stopping')
        self._monitor_selected_topic()
        self._update_status()

    def _tick_status(self):
        if self._state == 'starting' and os.path.exists(self._bag_path):
            self._state, self._recording_started = 'recording', time.monotonic()
            self._message.setText('Bag: ' + self._bag_path)
            self._update_controls()
        self._update_status()

    def _update_status(self):
        if self._state == 'idle':
            text = 'Preview only' if self.preview else 'Ready'
        elif self._state in ('checking', 'starting'):
            text = 'Starting…'
        elif self._state == 'recording':
            seconds = int(time.monotonic() - self._recording_started)
            text = f'Recording  {seconds // 60:02d}:{seconds % 60:02d}'
        else:
            text = 'Saving…'
        recording = self._state == 'recording'
        self._status_label.setText(text)
        self._status_label.setStyleSheet('color: ' + _theme('#ff8282', '#b22230') + ';' if recording else '')
        self._status_card.setStyleSheet(
            'QFrame#recordingStatusCard { background: ' + _theme('#230b0d', '#fff1f2') +
            '; border: 1px solid ' + _theme('#713037', '#e6b7bb') + '; border-radius: 6px; }' if recording else '')
        self._status_dot.setStyleSheet('background: ' + ('#f0444c' if recording else _theme('#777777', '#8a8a8a')) +
                                       '; border: 0; border-radius: 5px;')
        header = text if self.is_busy() else ''
        if header != self._last_header or recording != self._last_recording:
            self._last_header, self._last_recording = header, recording
            if self._status_callback:
                self._status_callback(header, recording)

    def start_recording(self):
        if self.is_busy() or self.preview or not self._selected:
            return
        name = self._name.text().strip()
        # The name is one directory component, never a path; preserve existing
        # bags and reject control characters or names too long for bag files.
        if (name in ('.', '..') or '/' in name or '\\' in name or
                any(ord(character) < 32 or ord(character) == 127 for character in name) or
                len(os.fsencode(name)) > 200):
            self._finish_idle('Use a shorter name without slashes or control characters.', True)
            return
        try:
            Path(self._directory).mkdir(parents=True, exist_ok=True)
        except OSError:
            self._finish_idle('Could not create recording folder: ' + self._directory, True)
            return
        name = name or 'bag_' + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')[:-3]
        path, suffix = Path(self._directory) / name, 1
        while os.path.lexists(path):
            path = Path(self._directory) / f'{name}_{suffix}'
            suffix += 1
        self._bag_path, self._recording_topics = str(path), sorted(self._selected)
        self._output.clear()
        self._stop_requested, self._probe_stop_message, self._probe_stop_error = False, '', False
        self._state = 'checking'
        self._request_id += 1
        request = self._request_id
        self._message.setText('Preparing ' + self._bag_path)
        self._message.setProperty('recordingError', False)
        self._message.setStyleSheet('')
        self._update_controls()
        self._probe.start('ros2', ['bag', 'record', '--help'])

        def timeout():
            if request == self._request_id and self._state == 'checking' and self._probe.state() != QtCore.QProcess.NotRunning:
                self._probe_stop_message, self._probe_stop_error = 'Timed out checking ros2 bag record.', True
                self._state = 'stopping'
                self._update_controls()
                self._probe.kill()
        QtCore.QTimer.singleShot(10000, timeout)

    def _probe_error(self, error):
        if error == QtCore.QProcess.FailedToStart and self._state == 'stopping':
            self._finish_idle(self._probe_stop_message, self._probe_stop_error)
        elif error == QtCore.QProcess.FailedToStart and self._state == 'checking':
            self._finish_idle('Could not start ros2. Check that rosbag2 is installed and ROS is sourced.', True)

    def _probe_finished(self, code, status):
        help_text = bytes(self._probe.readAll())
        if self._state == 'stopping':
            self._finish_idle(self._probe_stop_message, self._probe_stop_error)
        elif self._state == 'checking':
            if status != QtCore.QProcess.NormalExit or code != 0:
                self._finish_idle('Could not inspect ros2 bag record. ' + _last_line(help_text), True)
            else:
                self._launch_recorder(bool(re.search(r'(?:^|\s)--topics(?:\s|,|$)', help_text.decode(errors='replace'))))

    def _launch_recorder(self, option_topics):
        args = ['ros2', 'bag', 'record', '-o', self._bag_path, '--include-hidden-topics']
        if option_topics:
            args.append('--topics')
        args.extend(self._recording_topics)
        self._state = 'starting'
        self._update_controls()
        self._recorder.start(sys.executable, [__file__, '--record-guard', str(os.getpid()), *args])

    def _recorder_started(self):
        if self._stop_requested and self._recorder.processId() > 0:
            os.kill(self._recorder.processId(), signal.SIGINT)

    def _read_output(self):
        self._output.extend(bytes(self._recorder.readAll()))
        if len(self._output) > 8192:
            del self._output[:-8192]

    def _recorder_error(self, error):
        if error == QtCore.QProcess.FailedToStart and self._state != 'idle':
            self._finish_idle('Could not start ros2 bag record. Check that rosbag2 is installed.', True)

    def _recorder_finished(self, code, status):
        self._read_output()
        if self._state == 'idle':
            return
        if status == QtCore.QProcess.NormalExit and code == 0 and (Path(self._bag_path) / 'metadata.yaml').exists():
            self._finish_idle('Saved: ' + self._bag_path)
        elif self._stop_requested and not os.path.exists(self._bag_path):
            self._finish_idle('Recording canceled before the bag was created.')
        else:
            self._finish_idle(f'Recorder stopped unexpectedly (exit {code}). {_last_line(self._output)}', True)

    def stop_recording(self):
        if self._state in ('idle', 'stopping'):
            return
        self._stop_requested = True
        checking = self._state == 'checking'
        self._state = 'stopping'
        self._message.setText('Canceling…' if checking else '')
        self._update_controls()
        if checking:
            self._probe_stop_message, self._probe_stop_error = 'Recording canceled.', False
            if self._probe.state() != QtCore.QProcess.NotRunning:
                self._probe.kill()
            else:
                self._finish_idle(self._probe_stop_message)
        elif self._recorder.processId() > 0:
            try:
                os.kill(self._recorder.processId(), signal.SIGINT)
            except ProcessLookupError:
                pass

    def _finish_idle(self, message, error=False):
        self._request_id += 1
        self._state = 'idle'
        self._message.setText(message)
        self._message.setProperty('recordingError', error)
        self._message.setStyleSheet('color: ' + _theme('#f4bd73', '#895314') + ';' if error else '')
        self._update_controls()
        if self._stopped_callback:
            callback, self._stopped_callback = self._stopped_callback, None
            QtCore.QTimer.singleShot(0, callback)
