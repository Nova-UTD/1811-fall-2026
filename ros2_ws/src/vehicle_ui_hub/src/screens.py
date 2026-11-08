# Startup and idle screens, including the startup helper's readiness handshake.
import math
import os
import sys
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets


class IdleScreen(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('idleScreen')
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent)
        self.setStyleSheet('''
            QWidget#idleScreen { background: #000000; }
            QLabel#idleTitle { color: #b7b9bb; font-size: 23px; font-weight: 500; }
            QLabel#idleHint { color: #686c70; font-size: 13px; }
            QPushButton { color: #c7c9cb; background: #151617; border: 1px solid #37393b;
              border-radius: 5px; padding: 0 22px; min-height: 48px; font-size: 14px; }
            QPushButton:hover { background: #242628; }
            QPushButton:pressed { background: #323436; }
        ''')
        self.wake_callback = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(14)
        layout.addStretch()
        moon = QtGui.QPixmap(64, 64)
        moon.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(moon)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        shape, cutout = QtGui.QPainterPath(), QtGui.QPainterPath()
        shape.addEllipse(QtCore.QRectF(9, 9, 44, 44))
        cutout.addEllipse(QtCore.QRectF(24, 1, 42, 42))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor('#aeb2b5'))
        painter.drawPath(shape.subtracted(cutout))
        painter.end()
        icon = QtWidgets.QLabel()
        icon.setPixmap(moon)
        icon.setAlignment(QtCore.Qt.AlignCenter)
        icon.setFixedHeight(64)
        layout.addWidget(icon)
        title = QtWidgets.QLabel('Idling')
        title.setObjectName('idleTitle')
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setFixedHeight(32)
        layout.addWidget(title)
        hint = QtWidgets.QLabel('Vehicle stack is still running')
        hint.setObjectName('idleHint')
        hint.setAlignment(QtCore.Qt.AlignCenter)
        hint.setFixedHeight(22)
        layout.addWidget(hint)
        layout.addSpacing(12)
        self._buttons = QtWidgets.QStackedWidget()
        self._buttons.setFixedSize(248, 50)
        wake = QtWidgets.QPushButton('Wake up')
        wake.setObjectName('idleWake')
        self._buttons.addWidget(wake)
        confirmation = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(confirmation)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        cancel = QtWidgets.QPushButton('Cancel')
        cancel.setObjectName('idleCancel')
        confirm = QtWidgets.QPushButton('Confirm wake')
        confirm.setObjectName('idleConfirm')
        row.addWidget(cancel)
        row.addWidget(confirm)
        self._buttons.addWidget(confirmation)
        layout.addWidget(self._buttons, 0, QtCore.Qt.AlignHCenter)
        layout.addStretch()
        wake.clicked.connect(lambda: self._buttons.setCurrentIndex(1))
        cancel.clicked.connect(lambda: self._buttons.setCurrentIndex(0))
        confirm.clicked.connect(self._wake)

    def _wake(self):
        if self.wake_callback:
            self.wake_callback()

    def paintEvent(self, event):
        # Always paint black, including when the surrounding hub uses light mode.
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtCore.Qt.black)

    def prepare(self):
        self._buttons.setCurrentIndex(0)


class StartupScreen(QtWidgets.QWidget):
    def __init__(self, logo_path=None):
        flags = QtCore.Qt.SplashScreen | QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint
        super().__init__(None, flags)
        self.setObjectName('startupScreen')
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent)
        if logo_path is None:
            logo_path = Path(__file__).resolve().parents[1] / 'assets' / 'startup.png'
            if not logo_path.is_file():
                from ament_index_python.packages import get_package_share_directory
                logo_path = Path(get_package_share_directory('vehicle_ui_hub')) / 'assets' / 'startup.png'
        self._logo = QtGui.QPixmap(os.fspath(logo_path))
        self._status = 'STARTING'
        self.skip_callback = None
        self._skip = QtWidgets.QPushButton('Open hub', self)
        self._skip.setFixedSize(130, 44)
        self._skip.setStyleSheet('QPushButton { color: #dddddd; background: #1a1a1a; border: 1px solid #555555; '
                                 'border-radius: 5px; } QPushButton:pressed { background: #333333; }')
        self._skip.clicked.connect(self._skip_wait)
        self._skip.hide()
        QtCore.QTimer.singleShot(15000, self._skip.show)
        self._started = time.monotonic()
        self._exit_progress = 0.0
        self._refresh = QtCore.QTimer(self)
        self._refresh.setInterval(33)
        self._refresh.timeout.connect(self.update)
        self._refresh.start()
        self._exit = QtCore.QVariantAnimation(self)
        self._exit.setStartValue(0.0)
        self._exit.setEndValue(1.0)
        self._exit.setDuration(600)
        self._exit.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self._exit.valueChanged.connect(self._advance_exit)

    def has_logo(self):
        return not self._logo.isNull()

    def set_status(self, status):
        self._status = status or 'STARTING'
        self.update()

    def _skip_wait(self):
        if self.skip_callback:
            self.skip_callback()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._skip.move(max(10, self.width() - self._skip.width() - 24), max(10, self.height() - 78))

    def _advance_exit(self, value):
        self._exit_progress = value
        if QtGui.QGuiApplication.platformName() != 'offscreen':
            self.setWindowOpacity(1.0 - value)
        self.update()

    def dismiss(self, on_finished=None):
        if self._exit.state() == QtCore.QAbstractAnimation.Running:
            return
        def finish():
            self.close()
            if on_finished:
                on_finished()
        self._exit.finished.connect(finish)
        self._exit.start()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtCore.Qt.black)
        elapsed = time.monotonic() - self._started
        pulse = 1.0 + 0.014 * math.sin(elapsed * 2.3)
        side = min(self.width() * 0.78, self.height() * 0.82) * pulse * (1.0 + 0.35 * self._exit_progress)
        center = QtCore.QPointF(self.width() / 2.0, self.height() * 0.46)
        target = QtCore.QRectF(center.x() - side / 2.0, center.y() - side / 2.0, side, side)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        if self.has_logo():
            painter.drawPixmap(target, self._logo, QtCore.QRectF(self._logo.rect()))
        else:
            painter.setPen(QtGui.QColor('#ef7419'))
            fallback = painter.font()
            fallback.setPixelSize(36)
            fallback.setBold(True)
            painter.setFont(fallback)
            painter.drawText(target, QtCore.Qt.AlignCenter, 'NOVA')
        painter.setOpacity(1.0 - self._exit_progress)
        caption = painter.font()
        caption.setPixelSize(12)
        caption.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 2.0)
        painter.setFont(caption)
        painter.setPen(QtGui.QColor('#777b80'))
        caption_y = self.height() * 0.83
        painter.drawText(QtCore.QRectF(10, caption_y, self.width() - 20, 24), QtCore.Qt.AlignCenter, self._status)
        painter.setPen(QtCore.Qt.NoPen)
        active = int(elapsed * 1000 / 260) % 3
        for index in range(3):
            painter.setBrush(QtGui.QColor('#ef7419' if index == active else '#343434'))
            painter.drawRect(QtCore.QRectF(self.width() / 2.0 - 11.0 + index * 9.0, caption_y + 34.0, 4.0, 4.0))
        caption.setPixelSize(11)
        caption.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 0.4)
        painter.setFont(caption)
        painter.setPen(QtGui.QColor('#a8aaad'))
        painter.drawText(QtCore.QRectF(12, self.height() - 35, self.width() - 24, 20), QtCore.Qt.AlignCenter,
                         'Powered by NOVA @ UTD  ·  Blackwall Feature #2')


class StartupProcess:
    def __init__(self, entrypoint, session=None):
        self._entrypoint = os.fspath(entrypoint)
        self._process = QtCore.QProcess()
        self._session = Path(session) if session else None
        self._external = self._skipped = False
        self._visible_at = None
        self._finishing = False
        self._last_status = ''

    def start(self, windowed, window_size):
        # ./g can start the same screen on the host before Docker builds. Fall
        # back to the normal helper when that optional early screen is absent.
        if self._session:
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                if (self._session / 'visible').is_file() or (self._session / 'skip').is_file():
                    self._external = True
                    self._visible_at = time.monotonic()
                    return
                QtCore.QThread.msleep(25)
            # A delayed or failed host helper must not later cover the hub.
            try:
                (self._session / 'abandon').touch()
            except OSError:
                pass
        args = [self._entrypoint, '--startup-splash']
        if windowed:
            args += ['--windowed', '--preview-size', f'{window_size.width()}x{window_size.height()}']
        self._process.setProcessChannelMode(QtCore.QProcess.ForwardedErrorChannel)
        self._process.start(sys.executable, args)
        # Wait for the helper to paint once before RViz blocks this Qt thread.
        output = b''
        deadline = time.monotonic() + 4
        if self._process.waitForStarted(2000):
            while time.monotonic() < deadline and b'VISIBLE\n' not in output:
                output += bytes(self._process.readAllStandardOutput())
                if b'VISIBLE\n' in output:
                    break
                remaining = max(0, int((deadline - time.monotonic()) * 1000))
                if not remaining or not self._process.waitForReadyRead(remaining):
                    output += bytes(self._process.readAllStandardOutput())
                    break
        if b'VISIBLE\n' not in output:
            print('[hub] Startup screen unavailable; continuing without it.', file=sys.stderr)
            self._process.closeWriteChannel()
            return
        self._visible_at = time.monotonic()
        self._process.readyReadStandardOutput.connect(self._read_output)

    def _read_output(self):
        if b'SKIP\n' in bytes(self._process.readAllStandardOutput()):
            self._skipped = True

    def available(self):
        if self._visible_at is None:
            return False
        if not self._external:
            return self._process.state() != QtCore.QProcess.NotRunning
        try:
            pid = int((self._session / 'pid').read_text(encoding='ascii'))
            if pid < 2:
                return False
            os.kill(pid, 0)
            return True
        except PermissionError:
            return True
        except (OSError, ValueError):
            return False

    def skipped(self):
        if self._external and (self._session / 'skip').exists():
            try:
                (self._session / 'skip_ack').touch()
            except OSError:
                pass
            return True
        return self._skipped

    def set_status(self, status):
        if status == self._last_status:
            return
        self._last_status = status
        if self._external:
            try:
                (self._session / 'status').write_text(status, encoding='utf-8')
            except OSError as error:
                print(f'[hub] Could not update startup status: {error}', file=sys.stderr)
        elif self._process.state() == QtCore.QProcess.Running:
            self._process.write(('STATUS ' + status.replace('\n', ' ') + '\n').encode())

    def finish(self):
        if self._finishing or self._visible_at is None:
            return
        self._finishing = True
        remaining = max(0, int(700 - (time.monotonic() - self._visible_at) * 1000))
        QtCore.QTimer.singleShot(remaining, self._send_ready)

    def _send_ready(self):
        if self._external:
            try:
                (self._session / 'ready').touch()
            except OSError as error:
                print(f'[hub] Could not finish startup screen: {error}', file=sys.stderr)
        elif self._process.state() == QtCore.QProcess.Running:
            self._process.write(b'READY\n')
            self._process.closeWriteChannel()

    def close(self):
        if self._external:
            if not self._finishing and not self.skipped():
                try:
                    (self._session / 'error').touch()
                except OSError:
                    pass
            return
        if self._process.state() != QtCore.QProcess.NotRunning:
            self._process.closeWriteChannel()
            if not self._process.waitForFinished(900):
                self._process.kill()
                self._process.waitForFinished(500)


def run_startup_screen(app, windowed, window_size):
    screen = StartupScreen()
    if windowed:
        screen.resize(window_size if window_size.isValid() else QtCore.QSize(1440, 900))
        display = QtGui.QGuiApplication.primaryScreen()
        if display:
            screen.move(display.availableGeometry().center() - screen.rect().center())
        screen.show()
    else:
        display = QtGui.QGuiApplication.primaryScreen()
        if display:
            screen.setGeometry(display.geometry())
        screen.showFullScreen()
    screen.repaint()
    app.processEvents()
    # Keep the helper above the hub even on displays without a window manager.
    raise_timer = QtCore.QTimer(screen)
    raise_timer.setInterval(250)
    raise_timer.timeout.connect(screen.raise_)
    if QtGui.QGuiApplication.platformName() != 'offscreen':
        raise_timer.start()

    def skip():
        raise_timer.stop()
        print('SKIP', flush=True)
        screen.dismiss(app.quit)
    screen.skip_callback = skip
    notifier = QtCore.QSocketNotifier(sys.stdin.fileno(), QtCore.QSocketNotifier.Read, screen)
    pending = bytearray()

    def read_input():
        data = os.read(sys.stdin.fileno(), 64)
        if not data:
            notifier.setEnabled(False)
            screen.close()
            app.quit()
        else:
            pending.extend(data)
            while b'\n' in pending:
                line, _, remainder = pending.partition(b'\n')
                pending[:] = remainder
                if line == b'READY':
                    notifier.setEnabled(False)
                    raise_timer.stop()
                    screen.dismiss(app.quit)
                    break
                if line.startswith(b'STATUS '):
                    screen.set_status(line[7:].decode('utf-8', errors='replace'))

    notifier.activated.connect(read_input)
    print('VISIBLE', flush=True)
    return app.exec_()


def _cleanup_session(session):
    for name in ('visible', 'pid', 'status', 'ready', 'skip', 'skip_ack', 'error', 'abandon'):
        try:
            (session / name).unlink(missing_ok=True)
        except OSError:
            pass
    try:
        session.rmdir()
    except OSError:
        pass


def run_host_startup_screen(app, session, parent_pid):
    # The host helper starts before Docker/build. Only its own session files
    # are touched, and it exits when the launcher dies or the hub is ready.
    session = Path(session)
    app.setQuitOnLastWindowClosed(False)
    if (session / 'abandon').exists():
        _cleanup_session(session)
        return 0
    screen = StartupScreen()
    display = QtGui.QGuiApplication.primaryScreen()
    if display:
        screen.setGeometry(display.geometry())
    screen.showFullScreen()
    screen.repaint()
    app.processEvents()
    if (session / 'abandon').exists():
        screen.close()
        _cleanup_session(session)
        return 0
    (session / 'pid').write_text(str(os.getpid()), encoding='ascii')
    (session / 'visible').touch()

    def skip():
        if (session / 'error').exists():
            screen.dismiss(app.quit)
        else:
            (session / 'skip').touch()
            screen.dismiss()
    screen.skip_callback = skip
    timer = QtCore.QTimer(screen)
    timer.setInterval(200)

    def poll():
        try:
            os.kill(parent_pid, 0)
        except ProcessLookupError:
            screen.close()
            app.quit()
            return
        except PermissionError:
            pass
        if (session / 'abandon').exists():
            screen.close()
            app.quit()
        elif (session / 'skip').exists():
            if (session / 'skip_ack').exists():
                timer.stop()
                app.quit()
        elif (session / 'ready').exists():
            timer.stop()
            screen.dismiss(app.quit)
        elif (session / 'error').exists():
            screen.set_status('Startup stopped · Check the terminal')
            screen._skip.setText('Close')
            screen._skip.show()
        else:
            try:
                screen.set_status((session / 'status').read_text(encoding='utf-8').strip())
            except OSError:
                pass
        if screen.isVisible() and QtGui.QGuiApplication.platformName() != 'offscreen' and not (session / 'ready').exists():
            screen.raise_()
    timer.timeout.connect(poll)
    timer.start()
    result = app.exec_()
    _cleanup_session(session)
    return result


if __name__ == '__main__' and len(sys.argv) == 4 and sys.argv[1] == '--host-splash':
    app = QtWidgets.QApplication(sys.argv[:1])
    sys.exit(run_host_startup_screen(app, sys.argv[2], int(sys.argv[3])))
