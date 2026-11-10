# Show physical network ports, bounded tcpdump output and approximate weather.
import json
import math
import os
import shutil
import signal
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtNetwork, QtWidgets
from hub_widgets import AnimatedStatusLabel


def _label(text, name):
    label = QtWidgets.QLabel(text)
    label.setObjectName(name)
    label.setTextFormat(QtCore.Qt.PlainText)
    return label


def _read(path):
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as stream:
            return stream.read(256).strip()
    except OSError:
        return ''


def _conditions(code):
    if code == 0:
        return 'Clear'
    if code in (1, 2):
        return 'Partly cloudy'
    if code == 3:
        return 'Cloudy'
    if code in (45, 48):
        return 'Fog'
    if 51 <= code <= 57:
        return 'Drizzle'
    if 61 <= code <= 67:
        return 'Rain'
    if 71 <= code <= 77 or 85 <= code <= 86:
        return 'Snow'
    if 80 <= code <= 82:
        return 'Showers'
    if 95 <= code <= 99:
        return 'Thunderstorms'
    return 'Conditions unavailable'


class NetworkStatus(QtWidgets.QWidget):
    def __init__(self, preview=False, parent=None, sysfs_root='/sys/class/net',
                 enable_external_checks=True, manager=None, enable_probe=True):
        super().__init__(parent)
        self.preview, self._external_checks = preview, enable_external_checks and not preview
        self._probe_enabled, self._sysfs_root = enable_probe, Path(sysfs_root)
        self._active = self._idle = self._probing = self._stopping = False
        self._ports = []
        self._pending, self._errors = bytearray(), bytearray()
        self._dropped = False
        self._weather_reply, self._weather_attempt = None, 0.0
        self._internet_callback = self._weather_callback = None
        self._internet_text = self._internet_detail = self._weather_text = self._weather_detail = ''
        self.setObjectName('networkPanel')
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(12)
        heading = QtWidgets.QHBoxLayout()
        heading.addWidget(_label('Network', 'drawerTitle'))
        heading.addStretch()
        self.close_button = QtWidgets.QPushButton('×')
        self.close_button.setObjectName('closeButton')
        self.close_button.setAccessibleName('Close network settings')
        self.close_button.setFixedSize(44, 44)
        heading.addWidget(self.close_button)
        layout.addLayout(heading)
        internet_row = QtWidgets.QHBoxLayout()
        internet_row.addWidget(_label('Internet', 'sectionTitle'))
        internet_row.addStretch()
        self._internet = AnimatedStatusLabel('Checking…')
        self._internet.setObjectName('networkInternet')
        self._internet.setTextFormat(QtCore.Qt.PlainText)
        internet_row.addWidget(self._internet)
        layout.addLayout(internet_row)
        self._weather = AnimatedStatusLabel('Weather unavailable')
        self._weather.setObjectName('networkWeather')
        self._weather.setTextFormat(QtCore.Qt.PlainText)
        self._weather.setWordWrap(True)
        layout.addWidget(self._weather)
        layout.addSpacing(6)
        layout.addWidget(_label('Ports', 'sectionTitle'))
        self._tree = QtWidgets.QTreeWidget()
        self._tree.setObjectName('networkPorts')
        self._tree.setColumnCount(3)
        self._tree.setHeaderLabels(['Port', 'Link', 'IPv4'])
        self._tree.setRootIsDecorated(False)
        self._tree.setMinimumHeight(145)
        self._tree.setMaximumHeight(220)
        self._tree.header().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self._tree.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self._tree.header().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self._tree)
        self._port_detail = _label('Select a port for details.', 'networkPortDetail')
        self._port_detail.setWordWrap(True)
        self._port_detail.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self._port_detail)
        layout.addSpacing(4)
        layout.addWidget(_label('Packet capture', 'sectionTitle'))
        capture_row = QtWidgets.QHBoxLayout()
        self._capture_port = QtWidgets.QComboBox()
        self._capture_port.setObjectName('networkCapturePort')
        self._capture_port.setMinimumHeight(44)
        capture_row.addWidget(self._capture_port, 1)
        self._capture_button = QtWidgets.QPushButton('Start')
        self._capture_button.setObjectName('networkCaptureButton')
        self._capture_button.setMinimumHeight(44)
        capture_row.addWidget(self._capture_button)
        layout.addLayout(capture_row)
        self._capture_status = AnimatedStatusLabel('Up to 5,000 packets per capture.')
        self._capture_status.setObjectName('networkCaptureStatus')
        self._capture_status.setTextFormat(QtCore.Qt.PlainText)
        self._capture_status.setWordWrap(True)
        layout.addWidget(self._capture_status)
        self._output = QtWidgets.QPlainTextEdit()
        self._output.setObjectName('networkCaptureOutput')
        self._output.setReadOnly(True)
        self._output.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self._output.document().setMaximumBlockCount(400)
        self._output.setMinimumHeight(180)
        self._output.setPlaceholderText('Packet summaries appear here.')
        layout.addWidget(self._output)
        layout.addStretch()
        self._capture = QtCore.QProcess(self)
        self._tree.currentItemChanged.connect(self._selected_port)
        self._capture_port.currentIndexChanged.connect(self._selected_capture_port)
        self._capture_button.clicked.connect(self._toggle_capture)
        self._capture.readyReadStandardOutput.connect(self._read_capture)
        self._capture.readyReadStandardError.connect(self._read_capture_error)
        self._capture.errorOccurred.connect(self._capture_error)
        self._capture.finished.connect(self._finish_capture)
        self._output_timer = QtCore.QTimer(self)
        self._output_timer.setInterval(250)
        self._output_timer.timeout.connect(self._flush_capture)
        self._output_timer.start()
        self._port_timer = QtCore.QTimer(self)
        self._port_timer.setInterval(2000)
        self._port_timer.timeout.connect(self.refresh_ports)
        self._probe = QtNetwork.QTcpSocket(self)
        self._probe_timer = QtCore.QTimer(self)
        self._probe_timer.setInterval(30000)
        self._probe_timer.timeout.connect(self._start_probe)
        self._probe_timeout = QtCore.QTimer(self)
        self._probe_timeout.setSingleShot(True)
        self._probe_timeout.timeout.connect(lambda: self._finish_probe(False))
        self._probe.connected.connect(lambda: self._finish_probe(True))
        self._probe.errorOccurred.connect(lambda _: self._finish_probe(False))
        self._network = manager or QtNetwork.QNetworkAccessManager(self)
        self._weather_timer = QtCore.QTimer(self)
        self._weather_timer.setInterval(15 * 60 * 1000)
        self._weather_timer.timeout.connect(self._refresh_weather)
        self.refresh_ports()
        if self._external_checks:
            self._set_internet('Checking…', 'Checking TCP connectivity to 1.1.1.1:443.')
            self._set_weather('Weather —', 'Finding an approximate location from the internet connection.')
            if self._probe_enabled:
                self._probe_timer.start()
            self._weather_timer.start()
            QtCore.QTimer.singleShot(0, self._start_external_checks)
        else:
            self._set_internet('—', 'Internet checks are disabled in preview.' if preview else 'Internet check disabled.')
            self._set_weather('Weather —', 'Weather is unavailable in preview.')

    def _start_external_checks(self):
        if not self._idle:
            self._start_probe()
            self._refresh_weather()

    def set_internet_callback(self, callback):
        self._internet_callback = callback
        if callback:
            callback(self._internet_text, self._internet_detail)

    def set_weather_callback(self, callback):
        self._weather_callback = callback
        if callback:
            callback(self._weather_text, self._weather_detail)

    def set_active(self, active):
        self._active = bool(active)
        if self._active and not self._idle:
            self.refresh_ports()
            self._port_timer.start()
        else:
            self._port_timer.stop()
            self.stop_capture()
            if self._capture.state() == QtCore.QProcess.NotRunning:
                self._capture_button.setEnabled(False)

    def set_idle(self, idle):
        if self._idle == bool(idle):
            return
        self._idle = bool(idle)
        if idle:
            self.set_active(False)
            for timer in (self._output_timer, self._probe_timer, self._probe_timeout, self._weather_timer):
                timer.stop()
            self._probing = False
            self._probe.abort()
            if self._weather_reply:
                reply, self._weather_reply = self._weather_reply, None
                reply.abort()
                reply.deleteLater()
            return
        self._output_timer.start()
        if self._external_checks:
            self._set_internet('Checking…', 'Checking TCP connectivity to 1.1.1.1:443.')
            self._set_weather('Weather —', 'Refreshing the weather after idle.')
            if self._probe_enabled:
                self._probe_timer.start()
            self._weather_timer.start()
            QtCore.QTimer.singleShot(0, self._start_external_checks)

    def shutdown(self):
        # Let tcpdump handle SIGINT before Qt destroys its process object. A
        # stuck capture has a short deadline so it cannot block hub shutdown.
        self.set_idle(True)
        if self._capture.state() != QtCore.QProcess.NotRunning and not self._capture.waitForFinished(2000):
            self._capture.kill()
            self._capture.waitForFinished(1000)

    def refresh_ports(self, force=False):
        addresses = {}
        for interface in QtNetwork.QNetworkInterface.allInterfaces():
            for entry in interface.addressEntries():
                ip = entry.ip()
                if ip.protocol() == QtNetwork.QAbstractSocket.IPv4Protocol and not ip.isLoopback():
                    addresses.setdefault(interface.name(), []).append(ip.toString())
        ports = []
        try:
            names = sorted(path.name for path in self._sysfs_root.iterdir() if path.is_dir())
        except OSError:
            names = []
        for name in names:
            base = self._sysfs_root / name
            if not (base / 'device').exists() or _read(base / 'type') != '1':
                continue
            try:
                flags = int(_read(base / 'flags'), 0)
            except ValueError:
                flags = None
            carrier, operstate = _read(base / 'carrier'), _read(base / 'operstate')
            if flags is not None and not flags & 1:
                link = 'Disabled'
            elif carrier == '1' or not carrier and operstate == 'up':
                link = 'Connected'
            elif carrier == '0' or operstate == 'down':
                link = 'Disconnected'
            else:
                link = 'Unknown'
            speed = _read(base / 'speed')
            speed = f'{speed} Mb/s' if speed.isdigit() and int(speed) > 0 else 'Speed unavailable'
            mac = _read(base / 'address') or '—'
            counts = []
            for key in ('rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped'):
                number = _read(base / 'statistics' / key)
                counts.append(number if number.isdigit() else '—')
            counters = f'RX errors {counts[0]} · TX errors {counts[1]}\nRX dropped {counts[2]} · TX dropped {counts[3]}'
            ports.append((name, link, ', '.join(addresses.get(name, [])) or '—', speed, mac, counters))
        if not force and ports == self._ports:
            if self._capture.state() == QtCore.QProcess.NotRunning:
                self._capture_button.setEnabled(not self.preview and self._active and self._capture_port.count() > 0)
            self._show_port_detail()
            return
        selected = self._tree.currentItem().text(0) if self._tree.currentItem() else ''
        capture_name = self._capture_port.currentText()
        self._ports = ports
        tree_blocker, combo_blocker = QtCore.QSignalBlocker(self._tree), QtCore.QSignalBlocker(self._capture_port)
        self._tree.clear()
        self._capture_port.clear()
        for name, link, address, _, _, _ in ports:
            item = QtWidgets.QTreeWidgetItem(self._tree, [name, link, address])
            item.setToolTip(2, address)
            app = QtWidgets.QApplication.instance()
            white = bool(app and app.property('vehicleHubWhiteTheme'))
            item.setForeground(1, QtGui.QColor(('#176d45' if white else '#8ac9aa') if link == 'Connected' else
                                               ('#5b6164' if white else '#aaaaaa')))
            item.setSizeHint(0, QtCore.QSize(0, 40))
            self._capture_port.addItem(name)
            if name == selected:
                self._tree.setCurrentItem(item)
        if self._capture.state() != QtCore.QProcess.NotRunning and capture_name and self._capture_port.findText(capture_name) < 0:
            self._capture_port.addItem(capture_name)
        index = self._capture_port.findText(capture_name)
        if index < 0:
            index = self._capture_port.findText('enp4s0')
        if index < 0 and self._capture_port.count():
            index = 0
        if index >= 0:
            self._capture_port.setCurrentIndex(index)
        if not self._tree.currentItem() and self._tree.topLevelItemCount():
            matches = self._tree.findItems(capture_name, QtCore.Qt.MatchExactly, 0)
            self._tree.setCurrentItem(matches[0] if matches else self._tree.topLevelItem(0))
        self._capture_button.setEnabled(not self.preview and self._active and self._capture_port.count() > 0 and
                                        self._capture.state() == QtCore.QProcess.NotRunning)
        if self._capture.state() != QtCore.QProcess.NotRunning:
            self._capture_button.setEnabled(not self._stopping)
        del tree_blocker, combo_blocker
        self._show_port_detail()

    def _selected_port(self, current, previous):
        self._show_port_detail()
        if current and self._capture.state() == QtCore.QProcess.NotRunning:
            index = self._capture_port.findText(current.text(0))
            if index >= 0:
                self._capture_port.setCurrentIndex(index)

    def _selected_capture_port(self, index):
        if index >= 0 and self._capture.state() == QtCore.QProcess.NotRunning:
            items = self._tree.findItems(self._capture_port.itemText(index), QtCore.Qt.MatchExactly, 0)
            if items:
                self._tree.setCurrentItem(items[0])

    def _show_port_detail(self):
        item = self._tree.currentItem()
        if not item:
            self._port_detail.setText('No physical network ports found.')
            return
        for name, _, address, speed, mac, counters in self._ports:
            if name == item.text(0):
                self._port_detail.setText(f'{name} · {speed}\nIPv4: {address}\nMAC: {mac}\n{counters}')
                return

    def _start_probe(self):
        if self._idle or not self._external_checks or not self._probe_enabled or self._probing:
            return
        self._probing = True
        self._probe.abort()
        self._probe_timeout.start(4000)
        self._probe.connectToHost(QtNetwork.QHostAddress('1.1.1.1'), 443)

    def _finish_probe(self, reachable):
        if not self._probing:
            return
        self._probing = False
        self._probe_timeout.stop()
        self._probe.abort()
        self._set_internet('Reachable' if reachable else 'Unverified',
                           'TCP connection to 1.1.1.1:443 succeeded.' if reachable else
                           'Could not reach 1.1.1.1:443. A firewall can also block this check.')
        if reachable and self._weather_text == 'Weather —' and time.monotonic() - self._weather_attempt > 300:
            self._refresh_weather()

    def _set_internet(self, text, detail):
        self._internet_text, self._internet_detail = text, detail
        self._internet.setText(text)
        self._internet.setToolTip(detail)
        if self._internet_callback:
            self._internet_callback(text, detail)

    def _get_json(self, url, finished):
        request = QtNetwork.QNetworkRequest(QtCore.QUrl(url))
        request.setHeader(QtNetwork.QNetworkRequest.UserAgentHeader, '1811-vehicle-hub/1.0')
        request.setAttribute(QtNetwork.QNetworkRequest.RedirectPolicyAttribute, QtNetwork.QNetworkRequest.NoLessSafeRedirectPolicy)
        reply = self._network.get(request)
        self._weather_reply = reply
        timer = QtCore.QTimer(reply)
        timer.setSingleShot(True)
        timer.timeout.connect(reply.abort)
        timer.start(8000)
        reply.readyRead.connect(lambda: reply.abort() if reply.bytesAvailable() > 65536 else None)

        def complete():
            if self._weather_reply is not reply:
                return
            self._weather_reply = None
            ok = reply.error() == QtNetwork.QNetworkReply.NoError and reply.attribute(
                QtNetwork.QNetworkRequest.HttpStatusCodeAttribute) == 200
            try:
                data = json.loads(bytes(reply.readAll()))
                if not isinstance(data, dict):
                    data = {}
            except (ValueError, UnicodeDecodeError):
                data = {}
            reply.deleteLater()
            finished(ok, data)
        reply.finished.connect(complete)

    def _refresh_weather(self):
        if self._idle or not self._external_checks or self._weather_reply:
            return
        self._weather_attempt = time.monotonic()

        def located(ok, data):
            lat, lon = data.get('latitude'), data.get('longitude')
            valid = lambda value: isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            if not ok or not data.get('success') or not valid(lat) or not valid(lon) or abs(lat) > 90 or abs(lon) > 180:
                self._set_weather('Weather —', 'Approximate IP location is unavailable.')
                return
            city = ' '.join(str(data.get('city', '')).split())[:60]
            self._request_weather(lat, lon, city)
        self._get_json('https://ipwho.is/', located)

    def _request_weather(self, latitude, longitude, city):
        url = QtCore.QUrl('https://api.open-meteo.com/v1/forecast')
        query = QtCore.QUrlQuery()
        for key, value in (('latitude', f'{latitude:.3f}'), ('longitude', f'{longitude:.3f}'),
                           ('current', 'temperature_2m,weather_code'), ('temperature_unit', 'fahrenheit'),
                           ('forecast_days', '1')):
            query.addQueryItem(key, value)
        url.setQuery(query)

        def received(ok, data):
            current = data.get('current', {})
            current = current if isinstance(current, dict) else {}
            temperature = current.get('temperature_2m')
            if (not ok or isinstance(temperature, bool) or not isinstance(temperature, (int, float)) or
                    not math.isfinite(temperature) or not -150 <= temperature <= 160):
                self._set_weather('Weather —', 'Weather data is unavailable.')
                return
            # Qt rounds halves away from zero, unlike Python's default rounding.
            rounded = math.floor(temperature + 0.5) if temperature >= 0 else math.ceil(temperature - 0.5)
            location = f'Near {city} (IP estimate)' if city else 'Approximate IP location'
            code = current.get('weather_code')
            conditions = _conditions(int(code)) if isinstance(code, (int, float)) and math.isfinite(code) else 'Conditions unavailable'
            self._set_weather(f'{rounded}°F', f'{location} · {conditions} · Open-Meteo\n'
                              'IP location can be wrong on VPN or mobile networks.')
        self._get_json(url, received)

    def _set_weather(self, text, detail):
        self._weather_text, self._weather_detail = text, detail
        self._weather.setText(text + ('' if text == 'Weather —' else ' · ' + detail.split('\n', 1)[0]))
        self._weather.setToolTip(detail)
        if self._weather_callback:
            self._weather_callback(text, detail)

    def _toggle_capture(self):
        if self._capture.state() == QtCore.QProcess.NotRunning:
            self.start_capture()
        else:
            self.stop_capture()

    def start_capture(self):
        if not self._active or self.preview or self._capture.state() != QtCore.QProcess.NotRunning:
            return
        name = self._capture_port.currentText()
        if not any(port[0] == name for port in self._ports) or not (self._sysfs_root / name / 'device').exists():
            self._capture_status.setText('Port is no longer available.')
            self.refresh_ports()
            return
        program = shutil.which('tcpdump')
        if not program:
            self._capture_status.setText('tcpdump is not installed. Rebuild the container image.')
            return
        self._pending.clear()
        self._errors.clear()
        self._dropped = self._stopping = False
        self._output.clear()
        self._capture_status.setText(f'Capturing on {name} · up to 5000 packets.')
        self._capture_port.setEnabled(False)
        self._capture_button.setText('Stop')
        self._capture.start(program, ['-n', '-l', '-p', '-s', '96', '-c', '5000', '-i', name])

    def stop_capture(self):
        if self._capture.state() == QtCore.QProcess.NotRunning or self._stopping:
            return
        self._stopping = True
        self._capture_button.setEnabled(False)
        self._capture_status.setText('Stopping capture…')
        if self._capture.processId() > 0:
            try:
                os.kill(self._capture.processId(), signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            self._capture.terminate()
        QtCore.QTimer.singleShot(2000, self._kill_stalled_capture)

    def _kill_stalled_capture(self):
        if self._stopping and self._capture.state() != QtCore.QProcess.NotRunning:
            self._capture.kill()

    def _read_capture(self):
        self._pending.extend(bytes(self._capture.readAllStandardOutput()))
        if len(self._pending) > 65536:
            del self._pending[:-65536]
            newline = self._pending.find(b'\n')
            if newline >= 0:
                del self._pending[:newline + 1]
            self._dropped = True

    def _read_capture_error(self):
        self._errors.extend(bytes(self._capture.readAllStandardError()))
        if len(self._errors) > 8192:
            del self._errors[:-8192]

    def _flush_capture(self):
        if not self._pending and not self._dropped:
            return
        cursor = self._output.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        if self._dropped:
            cursor.insertText('[Display skipped packets during a burst]\n')
        cursor.insertText(self._pending.decode('utf-8', errors='replace'))
        self._output.setTextCursor(cursor)
        self._pending.clear()
        self._dropped = False

    def _capture_error(self, error):
        if error == QtCore.QProcess.FailedToStart:
            self._capture_status.setText('Could not start tcpdump.')
            self._capture_button.setText('Start')
            self._capture_button.setEnabled(self._active and self._capture_port.count() > 0 and not self.preview)
            self._capture_port.setEnabled(True)

    def _finish_capture(self, code, status):
        self._read_capture()
        self._read_capture_error()
        self._flush_capture()
        error = self._errors.decode('utf-8', errors='replace').strip()
        if not self._stopping and (status != QtCore.QProcess.NormalExit or code != 0):
            self._capture_status.setText('Capture failed. Check tcpdump and capture permissions.')
        else:
            self._capture_status.setText('Capture stopped.' if self._stopping else 'Capture complete.')
        if error:
            self._output.appendPlainText(error[-2000:])
        self._stopping = False
        self._capture_button.setText('Start')
        self._capture_port.setEnabled(True)
        self.refresh_ports(True)
