# Display current vehicle commands, estimated motion, sensor activity and diagnostics.
import html
import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from PyQt5 import QtCore, QtWidgets
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from vehicle_msgs.msg import VehicleCommand, VehicleState

from camera_views import ReceiveRate


def _label(text, name):
    label = QtWidgets.QLabel(text)
    label.setObjectName(name)
    return label


def _finite(value):
    return value if math.isfinite(value) else None


class VehicleStatus(QtCore.QObject):
    def __init__(self, node, preview=False, parent=None):
        super().__init__(parent)
        self.node, self.preview = node, preview
        self._lock = threading.Lock()
        self._generation, self._idle = 0, False
        self._subscriptions = {}
        self._rates = {name: ReceiveRate() for name in ('mode', 'command', 'feedback', 'odom', 'gamepad', 'diagnostics')}
        self._values = {}
        self._last = {}
        self._diagnostics = {}
        self._previous_odom = None
        self._sensor_status, self._sensor_connected = {}, False
        self.panel = self._make_panel()
        if preview:
            self._values.update(mode='MANUAL', speed=2.8, x=3.4, y=-0.7, yaw=18.0,
                                throttle=0.18, steering=-0.06, brake=0.0)
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self.refresh)
        self._subscribe()
        self._timer.start()
        self.refresh()

    @staticmethod
    def _metric(title, name):
        card = QtWidgets.QFrame()
        card.setObjectName('metricCard')
        layout = QtWidgets.QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(5)
        layout.addWidget(_label(title, 'metricCaption'))
        value = _label('—', name)
        layout.addWidget(value)
        return card, value

    @staticmethod
    def _signal_row(name, object_name, topic):
        row = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(row)
        layout.setContentsMargins(0, 3, 0, 3)
        layout.addWidget(_label(name, 'signalName'))
        layout.addStretch()
        value = _label('Unavailable', object_name)
        value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        value.setToolTip(topic)
        layout.addWidget(value)
        return row, value

    def _make_panel(self):
        panel = QtWidgets.QFrame()
        panel.setObjectName('vehicleStatusPanel')
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(14)
        heading = QtWidgets.QHBoxLayout()
        heading.addWidget(_label('Vehicle status', 'drawerTitle'))
        heading.addStretch()
        self.close_button = QtWidgets.QPushButton('×')
        self.close_button.setObjectName('closeButton')
        self.close_button.setFixedSize(44, 44)
        self.close_button.setAccessibleName('Close vehicle status')
        heading.addWidget(self.close_button)
        layout.addLayout(heading)
        metrics = QtWidgets.QGridLayout()
        metrics.setSpacing(10)
        for index, (title, key, name) in enumerate((
                ('Drive mode', 'mode', 'vehicleMode'), ('LiDAR speed estimate', 'speed', 'vehicleSpeed'),
                ('Reported battery', 'battery', 'vehicleBattery'), ('Odometry position', 'position', 'vehiclePosition'))):
            card, value = self._metric(title, name)
            setattr(self, '_' + key, value)
            metrics.addWidget(card, index // 2, index % 2)
        layout.addLayout(metrics)
        self._heading = _label('X, Y in odom · Heading —', 'statusHint')
        self._heading.setWordWrap(True)
        layout.addWidget(self._heading)
        layout.addSpacing(6)
        layout.addWidget(_label('Drive commands', 'sectionTitle'))
        commands = QtWidgets.QGridLayout()
        commands.setSpacing(8)
        for index, (title, key, name) in enumerate((
                ('Throttle', 'throttle', 'commandThrottle'), ('Steering', 'steering', 'commandSteering'),
                ('Brake', 'brake', 'commandBrake'))):
            card, value = self._metric(title, name)
            setattr(self, '_' + key, value)
            commands.addWidget(card, 0, index)
        layout.addLayout(commands)
        note = _label('Commands, not measured positions.', 'statusHint')
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addSpacing(6)
        layout.addWidget(_label('Activity', 'sectionTitle'))
        self._stream_labels = {}
        for title, key, name, topic in (
                ('Vehicle commands', 'command', 'commandStatus', '/vehicle_command'),
                ('LiDAR odometry', 'odom', 'odometryStatus', '/odometry'),
                ('Gamepad', 'gamepad', 'gamepadStatus', '/joy'),
                ('Vehicle state', 'feedback', 'feedbackStatus', '/vehicle_state'),
                ('Drive mode', 'mode', 'modeStatus', '/guardian/mode'),
                ('Diagnostics', 'diagnostics', 'diagnosticStream', '/diagnostics')):
            row, value = self._signal_row(title, name, topic)
            self._stream_labels[key] = value
            layout.addWidget(row)
        self._sensors = {}
        for key, title in (('lidar', 'LiDAR'), ('front', 'Front camera'), ('left', 'Left camera'),
                           ('rear', 'Rear camera'), ('right', 'Right camera')):
            topic = '/ouster/points' if key == 'lidar' else f'/zed_{key}/zed_node/point_cloud/cloud_registered'
            row, value = self._signal_row(title, 'sensor_' + key, topic)
            self._sensors[key] = value
            layout.addWidget(row)
        layout.addSpacing(6)
        layout.addWidget(_label('Diagnostics', 'sectionTitle'))
        self._diagnostic_status = _label('No recent diagnostics', 'diagnosticStatus')
        self._diagnostic_status.setTextFormat(QtCore.Qt.RichText)
        self._diagnostic_status.setWordWrap(True)
        layout.addWidget(self._diagnostic_status)
        layout.addStretch()
        return panel

    def _subscribe(self):
        if self.preview or not self.node:
            return
        with self._lock:
            self._generation += 1
            generation = self._generation
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for key, message, topic, qos in (
                ('mode', String, '/guardian/mode', latched),
                ('command', VehicleCommand, '/vehicle_command', reliable),
                ('feedback', VehicleState, '/vehicle_state', reliable),
                ('odom', Odometry, '/odometry', sensor),
                ('gamepad', Joy, '/joy', sensor),
                ('diagnostics', DiagnosticArray, '/diagnostics', sensor)):
            self._subscriptions[key] = self.node.create_subscription(
                message, topic, lambda msg, key=key, generation=generation: self._receive(key, generation, msg), qos)

    def _receive(self, key, generation, message):
        # Keep only the latest values for the next 250 ms refresh. This avoids
        # queuing one Qt event for every incoming ROS message.
        now = time.monotonic()
        with self._lock:
            if generation != self._generation or self._idle:
                return
            self._rates[key].record()
            self._last[key] = now
            if key == 'mode':
                self._values['mode'] = message.data
            elif key == 'command':
                self._values.update(throttle=_finite(message.throttle), steering=_finite(message.steer),
                                    brake=_finite(message.brake))
            elif key == 'feedback':
                value = _finite(message.battery_v)
                self._values['battery'] = value if value is not None and value > 0 else None
            elif key == 'odom':
                self._receive_odom(message)
            elif key == 'diagnostics':
                for item in message.status:
                    # Humble exposes this ROS byte field as bytes, while newer
                    # generated message bindings may expose it as an integer.
                    level = item.level[0] if isinstance(item.level, (bytes, bytearray)) else int(item.level)
                    self._diagnostics[item.name] = (level, item.message, now)

    def _receive_odom(self, message):
        x = _finite(message.pose.pose.position.x)
        y = _finite(message.pose.pose.position.y)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        frame, child = message.header.frame_id, message.child_frame_id
        speed = None
        if self._previous_odom:
            old_x, old_y, old_stamp, old_frame, old_child = self._previous_odom
            delta = stamp - old_stamp
            if (x is not None and y is not None and old_x is not None and old_y is not None and
                    0.01 < delta <= 2.0 and frame == old_frame and child == old_child):
                speed = _finite(math.hypot(x - old_x, y - old_y) / delta * 2.236936292)
        self._previous_odom = (x, y, stamp, frame, child)
        q = message.pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        yaw = None
        if math.isfinite(norm) and norm > 1e-9:
            qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
            yaw = _finite(math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))))
        self._values.update(x=x, y=y, speed=speed, yaw=yaw)

    def set_idle(self, idle):
        idle = bool(idle)
        if self._idle == idle:
            return
        with self._lock:
            self._idle = idle
            if idle:
                self._generation += 1
                if not self.preview:
                    self._values.clear()
                    self._last.clear()
                    self._diagnostics.clear()
                    self._previous_odom = None
                    for rate in self._rates.values():
                        rate.reset()
        if idle:
            self._timer.stop()
            if self.node:
                for subscription in self._subscriptions.values():
                    self.node.destroy_subscription(subscription)
            self._subscriptions.clear()
            return
        self._subscribe()
        self.refresh()
        self._timer.start()

    def set_sensors(self, status, connected=True):
        self._sensor_status = status if isinstance(status, dict) else {}
        self._sensor_connected = bool(connected)
        self.refresh()

    @staticmethod
    def _stream(label, available, missing='No recent data', rate=None):
        text = (f'{rate:.1f} Hz' if rate is not None else 'Receiving') if available else missing
        app = QtWidgets.QApplication.instance()
        white = bool(app and app.property('vehicleHubWhiteTheme'))
        if label.text() == text and label.property('streamStyled') and label.property('streamWhiteTheme') == white:
            return
        color = ('#176d45' if white else '#8ac9aa') if available else ('#5b6164' if white else '#a1a1a1')
        label.setText(text)
        label.setStyleSheet('color: ' + color + ';')
        label.setProperty('streamStyled', True)
        label.setProperty('streamWhiteTheme', white)

    def refresh(self):
        now = time.monotonic()
        with self._lock:
            values, last = dict(self._values), dict(self._last)
            self._diagnostics = {name: value for name, value in self._diagnostics.items() if now - value[2] <= 10}
            diagnostics = dict(self._diagnostics)
        fresh = lambda key, limit: self.preview or (key in last and now - last[key] <= limit)
        command_fresh, odom_fresh = fresh('command', 1), fresh('odom', 2)
        feedback_fresh, gamepad_fresh = fresh('feedback', 3), fresh('gamepad', 1.5)
        mode_available = bool(values.get('mode')) and (self.preview or (
            self.node and self._subscriptions.get('mode') and
            self.node.count_publishers(self._subscriptions['mode'].topic_name) > 0 and command_fresh))
        mode = values.get('mode', '—') if mode_available else '—'
        speed_value = values.get('speed')
        speed = f'{speed_value:.1f}' if odom_fresh and speed_value is not None else '—'
        self._mode.setText(mode)
        self._speed.setText(speed if speed == '—' else speed + ' mph')
        battery = values.get('battery')
        self._battery.setText(f'{battery:.1f} V' if feedback_fresh and battery is not None else '—')
        x, y, yaw = values.get('x'), values.get('y'), values.get('yaw')
        self._position.setText(f'{x:.1f}, {y:.1f} m' if odom_fresh and x is not None and y is not None else '—')
        self._heading.setText(f'X, Y in odom · Heading {yaw:.0f}°' if odom_fresh and yaw is not None else
                              'X, Y in odom · Heading —')
        for key in ('throttle', 'steering', 'brake'):
            value = values.get(key)
            getattr(self, '_' + key).setText(f'{value * 100:.0f} %' if command_fresh and value is not None else '—')
        available = {'command': command_fresh, 'odom': odom_fresh, 'gamepad': gamepad_fresh,
                     'feedback': feedback_fresh, 'mode': mode_available}
        for key in ('command', 'odom', 'gamepad', 'feedback', 'mode'):
            self._stream(self._stream_labels[key], available[key], rate=self._rates[key].sample())
        diagnostic_rate = self._rates['diagnostics'].sample()
        self._stream(self._stream_labels['diagnostics'], diagnostic_rate is not None, rate=diagnostic_rate)
        cameras = self._sensor_status.get('cameras', {})
        cameras = cameras if isinstance(cameras, dict) else {}
        for key, label in self._sensors.items():
            if not self._sensor_connected:
                self._stream(label, False, 'Unavailable')
                continue
            sensor = self._sensor_status if key == 'lidar' else cameras.get(key, {})
            sensor = sensor if isinstance(sensor, dict) else {}
            online = sensor.get('lidar_online', False) if key == 'lidar' else sensor.get('sensor_online', False)
            rate = sensor.get('lidar_hz') if key == 'lidar' else sensor.get('cloud_hz')
            rate = rate if isinstance(rate, (int, float)) and not isinstance(rate, bool) else None
            self._stream(label, bool(online), rate=rate)
        priority = {2: 3, 1: 2, 3: 1, 0: 0}
        recent = sorted(diagnostics.items(), key=lambda item: (-priority.get(item[1][0], 0), item[0]))[:4]
        app = QtWidgets.QApplication.instance()
        white = bool(app and app.property('vehicleHubWhiteTheme'))
        lines = []
        for name, (level, message, _) in recent:
            state = 'OK' if level == 0 else 'Warning' if level == 1 else 'Error' if level == 2 else 'Stale'
            color = ('#176d45' if white else '#8ac9aa') if level == 0 else (
                ('#b22230' if white else '#ec9292') if level == 2 else ('#895314' if white else '#e5bd82'))
            description = ' — ' + html.escape(message[:100]) if message else ''
            lines.append(f'<span style="color:{color}">{state}</span> · {html.escape(name)}{description}')
        self._diagnostic_status.setText('<br>'.join(lines) if lines else 'No recent diagnostics')
