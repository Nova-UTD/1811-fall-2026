# Developer-tool panels. Update operations remain in the checked vehicle script.
import os
import re
import signal
import sys
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
from hub_widgets import AnimatedStatusLabel


def _label(text, name):
    label = QtWidgets.QLabel(text)
    label.setObjectName(name)
    return label


def _status_label(text, name):
    label = AnimatedStatusLabel(text)
    label.setObjectName(name)
    return label


def _jetson_sync_enabled():
    enabled = os.environ.get('NOVA_UI_JETSON_UPDATE', 'false').lower() == 'true'
    cameras = os.environ.get('NOVA_UI_JETSON_CAMERAS', 'true').lower()
    registration = os.environ.get('NOVA_UI_JETSON_REGISTRATION', 'true').lower()
    return enabled and (cameras != 'false' or registration == 'true')


class SoftwareUpdate(QtWidgets.QWidget):
    def __init__(self, preview=False, parent=None):
        super().__init__(parent)
        self.setObjectName('updatePanel')
        self._preview, self._installed, self._target = preview, False, ''
        self._close_callback = None
        self.can_restart = None
        self._verifying = self._checking_jetson = self._restart_pending = False
        self._requested_target = ''
        root = os.environ.get('NOVA_1811_STACK_ROOT', '')
        self._script = str(Path(root) / 'scripts' / 'vehicle.sh') if root else ''
        token = os.environ.get('NOVA_UI_SUPERVISOR_TOKEN', '')
        self._supervisor = (Path(root) / '.nova-ui-startup' / token
                            if root and re.fullmatch(r'run\.[A-Za-z0-9]{6,}', token) else None)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(16)
        heading = QtWidgets.QHBoxLayout()
        heading.addWidget(_label('Updates', 'drawerTitle'))
        heading.addStretch()
        self.close_button = QtWidgets.QPushButton('×')
        self.close_button.setObjectName('closeButton')
        self.close_button.setAccessibleName('Back to developer tools')
        self.close_button.setFixedSize(44, 44)
        heading.addWidget(self.close_button)
        layout.addLayout(heading)
        layout.addSpacing(18)
        self._title = _status_label('Ready', 'updateStatus')
        self._title.setWordWrap(True)
        layout.addWidget(self._title)
        self._detail = _status_label('Update and restart the stack.', 'updateDetail')
        self._detail.setWordWrap(True)
        layout.addWidget(self._detail)
        layout.addSpacing(12)
        self._action = QtWidgets.QPushButton('Check for updates')
        self._action.setObjectName('updateAction')
        self._action.setMinimumHeight(52)
        layout.addWidget(self._action)
        self._changes = _status_label('', 'updateChanges')
        self._changes.setTextFormat(QtCore.Qt.PlainText)
        self._changes.setWordWrap(True)
        self._changes.hide()
        layout.addWidget(self._changes)
        layout.addStretch()
        if preview:
            self._message('Preview only', 'Start the stack to check for updates.')
            self._action.setEnabled(False)
        elif not self._script or not os.path.exists(self._script):
            self._message('Software not verified', 'Start with ./g to enable updates.')
            self._action.setEnabled(False)
        self._action.clicked.connect(lambda: self._run(bool(self._target)))
        self._process = QtCore.QProcess(self)
        self._process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self._process.errorOccurred.connect(self._process_error)
        self._process.finished.connect(self._complete)
        self._restart_timer = QtCore.QTimer(self)
        self._restart_timer.setInterval(150)
        self._restart_timer.timeout.connect(self._check_restart)
        result = os.environ.get('NOVA_UI_UPDATE_RESULT', '')
        if not preview and result:
            self._show_result(result)

    def is_busy(self):
        return self._restart_pending or self._process.state() != QtCore.QProcess.NotRunning

    def finish_then(self, callback):
        if self.is_busy():
            self._close_callback = callback
        else:
            QtCore.QTimer.singleShot(0, callback)

    def _run(self, apply):
        if self.is_busy() or self._preview or self._installed or not self._script or (apply and not self._target):
            return
        if apply and self.can_restart:
            reason = self.can_restart()
            if reason:
                self._message('Update paused', reason)
                return
        self._verifying = apply
        self._requested_target = self._target if apply else ''
        self._action.setEnabled(False)
        if not apply:
            self._changes.set_message('')
        self._message('Preparing…' if apply else 'Checking…', '')
        # Check again before asking the host to stop anything. The host applies
        # the exact checked revision after the ROS stack has shut down.
        self._process.start('setsid', ['--wait', 'bash', self._script, 'check-update'])

    def _process_error(self, error):
        if error != QtCore.QProcess.FailedToStart:
            return
        self._message('Update unavailable', 'Could not start the updater.')
        self._verifying = self._checking_jetson = False
        self._target = ''
        self._action.setText('Check for updates')
        self._action.setEnabled(not self._preview and not self._installed and bool(self._script))
        self._release_close()

    def _message(self, title, detail):
        self._title.setText(title)
        self._detail.set_message(detail)

    def _complete(self, exit_code, status):
        # Commit messages are untrusted input, so display them as plain text.
        output = bytes(self._process.readAll()).decode('utf-8', errors='replace')
        lines = output.split('\n')
        fields = lines[0].split() if lines else []
        result = fields[0] if fields else ''
        if self._checking_jetson:
            self._checking_jetson = False
            requested, self._requested_target = self._requested_target, ''
            if status == QtCore.QProcess.NormalExit and exit_code == 0 and result in ('JETSON_READY', 'JETSON_SKIPPED'):
                reason = self.can_restart() if self.can_restart else ''
                if not reason:
                    self._request_restart(requested)
                    return
                self._message('Update paused', reason)
            else:
                messages = {
                    'JETSON_DIRTY': 'Jetson has local changes. Save them before updating.',
                    'JETSON_DIVERGED': 'Jetson repository history needs attention.',
                }
                self._message('Update unavailable', messages.get(result, 'Check the Jetson connection and Git access.'))
                if len(lines) > 1:
                    print('\n'.join(lines[1:]).strip(), file=sys.stderr)
            self._target = ''
            self._action.setText('Check for updates')
            self._action.setEnabled(True)
            self._release_close()
            return
        verifying, requested = self._verifying, self._requested_target
        self._verifying = False
        self._requested_target = ''
        if verifying and status == QtCore.QProcess.NormalExit and exit_code == 0:
            if result == 'AVAILABLE' and len(fields) == 4 and fields[1] == requested and fields[3] == 'READY':
                # Jetson cameras may still be running. This only checks its Git
                # state; the actual checkout happens on the next ./g launch.
                self._checking_jetson = True
                self._requested_target = requested
                self._message('Preparing…', 'Checking the Jetson.')
                self._process.start('setsid', ['--wait', 'bash', self._script, 'check-jetson-update', requested])
                return
            if result == 'AVAILABLE':
                result = fields[3] if len(fields) == 4 and fields[3] in ('DIRTY', 'SUBMODULE', 'RESTART') else 'STALE'
            elif result in ('CURRENT', 'AHEAD'):
                result = 'STALE'
        self._target = ''
        if result not in ('AVAILABLE', 'UPDATED'):
            self._changes.set_message('')
        self._action.setText('Check for updates')
        self._action.setEnabled(not self._preview and not self._installed and bool(self._script))
        if status != QtCore.QProcess.NormalExit or exit_code != 0:
            self._message('Update unavailable', 'Check the checkout before retrying.')
        elif (result == 'AVAILABLE' and len(fields) == 4 and
              re.fullmatch(r'(?:[0-9a-fA-F]{40}|[0-9a-fA-F]{64})', fields[1])):
            branch = os.environ.get('NOVA_1811_STACK_BRANCH', '')
            branch_pattern = re.compile(r'(?<![\w./-])' + re.escape(branch) + r'(?![\w./-])', re.IGNORECASE) if branch else None
            subjects = []
            for line in lines[1:]:
                subject = re.sub(r'[\x00-\x1f\x7f]', '', line[:160])
                if branch_pattern:
                    subject = branch_pattern.sub('[branch]', subject)
                if subject.strip():
                    subjects.append('•  ' + subject.strip())
                if len(subjects) == 20:
                    break
            self._changes.set_message('\n'.join(subjects) if subjects else 'No commit messages.')
            if fields[3] == 'READY':
                if self._supervisor and (self._supervisor / 'supervisor_pid').is_file():
                    self._target = fields[1]
                    self._message('Update available', 'The stack will restart.')
                    self._action.setText('Update now')
                else:
                    self._message('Update available', 'Start with ./g to enable automatic updates.')
            elif fields[3] == 'DIRTY':
                self._message('Update available', 'Commit local changes before updating.')
            elif fields[3] == 'SUBMODULE':
                self._message('Update available', 'This update needs manual Git and submodule steps.')
            else:
                self._message('Update available', 'Restart from a clean checkout before updating.')
        elif result in ('CURRENT', 'AHEAD'):
            self._message('Up to date', '')
        else:
            messages = {
                'DIRTY': ('Update not installed', 'Commit local changes before updating.'),
                'RESTART': ('Software not verified', 'Restart with ./g from a clean checkout.'),
                'UNVERIFIED': ('Software not verified', 'Restart with ./g from a clean checkout.'),
                'STALE': ('Update changed', 'Check again before installing.'),
                'DIVERGED': ('Update unavailable', 'Repository history needs attention.'),
                'NO_REMOTE': ('Update unavailable', 'No update source is configured.'),
                'NETWORK': ('Update unavailable', 'Check the network and Git credentials.'),
                'PERMISSION': ('Update unavailable', 'Check checkout permissions.'),
                'STOP_FAILED': ('Update unavailable', 'Could not stop the running stack.'),
                'UPDATE_FAILED': ('Update not installed', 'Check the checkout and try again.'),
                'SUBMODULE': ('Update not installed', 'This update needs manual Git and submodule steps.'),
            }
            self._message(*messages.get(result, ('Update unavailable', 'Check the checkout before retrying.')))
        self._release_close()

    def _request_restart(self, target):
        # Only a session made by ./g may restart the stack. The request is
        # atomic so the host never reads a partly written revision.
        try:
            if not self._supervisor or not self._supervisor.is_dir():
                raise OSError('No running launcher')
            pid = int((self._supervisor / 'supervisor_pid').read_text(encoding='ascii'))
            if pid < 2:
                raise ValueError('Invalid launcher PID')
            for name in ('error', 'accepted'):
                (self._supervisor / name).unlink(missing_ok=True)
            temporary = self._supervisor / 'request.tmp'
            temporary.write_text(target + '\n', encoding='ascii')
            os.replace(temporary, self._supervisor / 'request')
            os.kill(pid, signal.SIGUSR1)
        except (OSError, ValueError) as error:
            print(f'[hub] Could not request update restart: {error}', flush=True)
            self._target = ''
            self._action.setText('Check for updates')
            self._action.setEnabled(True)
            self._message('Update unavailable', 'Start with ./g and try again.')
            self._release_close()
            return
        self._restart_pending = True
        self._restart_deadline = time.monotonic() + 5
        self._message('Restarting…', 'Preparing both computers.' if _jetson_sync_enabled() else 'Preparing the Karbon.')
        self._restart_timer.start()

    def _check_restart(self):
        if (self._supervisor / 'accepted').exists():
            self._restart_pending = False
            self._restart_timer.stop()
            self._release_close()
        elif (self._supervisor / 'error').exists() or time.monotonic() >= self._restart_deadline:
            self._restart_pending = False
            self._restart_timer.stop()
            self._target = ''
            self._action.setText('Check for updates')
            self._action.setEnabled(True)
            self._message('Update unavailable', 'Could not restart the stack. Check the terminal.')
            self._release_close()

    def _show_result(self, result):
        messages = {
            'UPDATED': ('Update installed', 'Jetson sync runs during startup.' if _jetson_sync_enabled() else
                        'Jetson sync is disabled.'),
            'DIRTY': ('Update not installed', 'Commit local changes before updating.'),
            'STALE': ('Update changed', 'Check again before installing.'),
            'DIVERGED': ('Update unavailable', 'Repository history needs attention.'),
            'NETWORK': ('Update unavailable', 'Check the network and Git credentials.'),
            'PERMISSION': ('Update unavailable', 'Check checkout permissions.'),
            'SUBMODULE': ('Update not installed', 'This update needs manual Git and submodule steps.'),
        }
        self._message(*messages.get(result, ('Update not installed', 'Check the terminal before retrying.')))

    def _release_close(self):
        if self._close_callback:
            callback, self._close_callback = self._close_callback, None
            QtCore.QTimer.singleShot(0, callback)
