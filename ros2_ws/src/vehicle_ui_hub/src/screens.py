# Startup and idle screens, including the startup helper's readiness handshake.
import math
import os
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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
            logo_path = Path(get_package_share_directory('vehicle_ui_hub')) / 'assets' / 'startup.png'
        self._logo = QtGui.QPixmap(os.fspath(logo_path))
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
        painter.drawText(QtCore.QRectF(0, caption_y, self.width(), 24), QtCore.Qt.AlignCenter, 'STARTING')
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
    def __init__(self, entrypoint):
        self._entrypoint = os.fspath(entrypoint)
        self._process = QtCore.QProcess()
        self._visible_at = None
        self._finishing = False

    def start(self, windowed, window_size):
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

    def finish(self):
        if self._finishing or self._visible_at is None:
            return
        self._finishing = True
        remaining = max(0, int(700 - (time.monotonic() - self._visible_at) * 1000))
        QtCore.QTimer.singleShot(remaining, self._send_ready)

    def _send_ready(self):
        if self._process.state() == QtCore.QProcess.Running:
            self._process.write(b'READY\n')
            self._process.closeWriteChannel()

    def close(self):
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
        screen.showFullScreen()
    screen.repaint()
    app.processEvents()
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
            if b'READY\n' in pending:
                notifier.setEnabled(False)
                screen.dismiss(app.quit)

    notifier.activated.connect(read_input)
    print('VISIBLE', flush=True)
    return app.exec_()
