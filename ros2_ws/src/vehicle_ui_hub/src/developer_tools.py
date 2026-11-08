# Developer-tool panels. Update operations remain in the checked vehicle script.
import os
import re
from pathlib import Path

from PyQt5 import QtCore, QtWidgets


def _label(text, name):
    label = QtWidgets.QLabel(text)
    label.setObjectName(name)
    return label


class SoftwareUpdate(QtWidgets.QWidget):
    def __init__(self, preview=False, parent=None):
        super().__init__(parent)
        self.setObjectName('updatePanel')
        self._preview, self._installed, self._target = preview, False, ''
        self._close_callback = None
        root = os.environ.get('NOVA_1811_STACK_ROOT', '')
        self._script = str(Path(root) / 'scripts' / 'vehicle.sh') if root else ''
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
        self._title = _label('Ready', 'updateStatus')
        self._title.setWordWrap(True)
        layout.addWidget(self._title)
        self._detail = _label('Updates this computer. Restart the stack after installing.', 'updateDetail')
        self._detail.setWordWrap(True)
        layout.addWidget(self._detail)
        layout.addSpacing(12)
        self._action = QtWidgets.QPushButton('Check for updates')
        self._action.setObjectName('updateAction')
        self._action.setMinimumHeight(52)
        layout.addWidget(self._action)
        self._changes = _label('', 'updateChanges')
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

    def is_busy(self):
        return self._process.state() != QtCore.QProcess.NotRunning

    def finish_then(self, callback):
        if self.is_busy():
            self._close_callback = callback
        else:
            QtCore.QTimer.singleShot(0, callback)

    def _run(self, apply):
        if self.is_busy() or self._preview or self._installed or not self._script or (apply and not self._target):
            return
        self._action.setEnabled(False)
        if not apply:
            self._changes.hide()
            self._changes.clear()
        self._message('Installing…' if apply else 'Checking…', 'Keep the stack running.' if apply else '')
        args = [self._script, 'apply-update' if apply else 'check-update']
        if apply:
            args.append(self._target)
        # An update must finish even if Ctrl-C stops the launch terminal. The
        # helper rechecks the exact revision before applying any update.
        self._process.start('setsid', ['--wait', 'bash', *args])

    def _process_error(self, error):
        if error != QtCore.QProcess.FailedToStart:
            return
        self._message('Update unavailable', 'Could not start the updater.')
        self._target = ''
        self._action.setText('Check for updates')
        self._action.setEnabled(not self._preview and not self._installed and bool(self._script))
        self._release_close()

    def _message(self, title, detail):
        self._title.setText(title)
        self._detail.setText(detail)
        self._detail.setVisible(bool(detail))

    def _complete(self, exit_code, status):
        # Commit messages are untrusted input, so display them as plain text.
        output = bytes(self._process.readAll()).decode('utf-8', errors='replace')
        lines = output.split('\n')
        fields = lines[0].split() if lines else []
        result = fields[0] if fields else ''
        self._target = ''
        if result not in ('AVAILABLE', 'UPDATED'):
            self._changes.hide()
            self._changes.clear()
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
            self._changes.setText('\n'.join(subjects) if subjects else 'No commit messages.')
            self._changes.show()
            if fields[3] == 'READY':
                self._target = fields[1]
                self._message('Update available', 'Restart the stack after installing.')
                self._action.setText('Update now')
            elif fields[3] == 'DIRTY':
                self._message('Update available', 'Commit local changes before updating.')
            else:
                self._message('Update available', 'Restart from a clean checkout before updating.')
        elif result == 'UPDATED':
            self._installed = True
            self._action.setEnabled(False)
            self._message('Update installed', 'Restart the stack to use it.')
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
            }
            self._message(*messages.get(result, ('Update unavailable', 'Check the checkout before retrying.')))
        self._release_close()

    def _release_close(self):
        if self._close_callback:
            callback, self._close_callback = self._close_callback, None
            QtCore.QTimer.singleShot(0, callback)
