"""Exercise vehicle launch, Jetson synchronization, and floor overlay without hardware."""

import importlib.util
import json
import os
import pty
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch


ROOT = REPO = Path(__file__).resolve().parents[1]
START_SCRIPT = ROOT / 'scripts/vehicle.sh'
UPDATER = ROOT / 'ros2_ws/src/obc_bringup/scripts/sync_jetson_repo.sh'
SSH_WRAPPER = UPDATER.with_name('jetson_cameras_ssh.sh')
spec = importlib.util.spec_from_file_location('floor_overlay', ROOT / 'scripts/prepare_zed_floor_overlay.py')
floor_overlay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(floor_overlay)


class VehicleCommandTest(unittest.TestCase):
    """Check the grouped command's help and unknown-command handling."""

    def test_help_lists_the_available_commands(self):
        result = subprocess.run(['bash', str(START_SCRIPT), '--help'],
                                text=True, capture_output=True, check=True)
        for command in ('dev', 'full-stack', 'cameras', 'setup-ssh', 'setup-link-jetson',
                        'setup-link-karbon', 'check-link', 'survey-link', 'setup-machine', 'fetch-radar'):
            self.assertIn(command, result.stdout)

    def test_unknown_command_fails_without_running_a_setup(self):
        result = subprocess.run(['bash', str(START_SCRIPT), 'does-not-exist'],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('Unknown vehicle command', result.stderr)


class SoftwareUpdateTest(unittest.TestCase):
    """Check update safeguards using temporary local Git repositories."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='vehicle updates ')
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.origin, self.source, self.vehicle = root / 'origin.git', root / 'source', root / 'vehicle checkout'
        self.env = os.environ.copy()
        self.env.update({
            'GIT_CONFIG_NOSYSTEM': '1', 'GIT_CONFIG_GLOBAL': '/dev/null', 'GIT_TERMINAL_PROMPT': '0',
            'GIT_AUTHOR_NAME': 'Test', 'GIT_AUTHOR_EMAIL': 'test@example.invalid',
            'GIT_COMMITTER_NAME': 'Test', 'GIT_COMMITTER_EMAIL': 'test@example.invalid',
        })
        self.git('init', '--bare', '--initial-branch=main', str(self.origin))
        self.git('clone', str(self.origin), str(self.source))
        (self.source / 'scripts').mkdir()
        shutil.copy2(START_SCRIPT, self.source / 'scripts/vehicle.sh')
        (self.source / 'version.txt').write_text('initial\n')
        (self.source / '.gitignore').write_text('ignored.txt\n')
        self.git('-C', str(self.source), 'add', '.')
        self.git('-C', str(self.source), 'commit', '-m', 'initial')
        self.git('-C', str(self.source), 'push', '-u', 'origin', 'main')
        self.git('-C', str(self.source), 'switch', '-c', 'blackwall')
        self.git('-C', str(self.source), 'push', '-u', 'origin', 'blackwall')
        self.git('clone', '--branch', 'blackwall', str(self.origin), str(self.vehicle))
        self.snapshot()

    def git(self, *args):
        return subprocess.run(['git', *args], env=self.env, check=True,
                              text=True, capture_output=True).stdout.strip()

    def snapshot(self, clean='1'):
        self.env.update({
            'NOVA_1811_STACK_ROOT': str(self.vehicle.resolve()),
            'NOVA_1811_STACK_BRANCH': self.git('-C', str(self.vehicle), 'branch', '--show-current'),
            'NOVA_1811_STACK_REVISION': self.git('-C', str(self.vehicle), 'rev-parse', 'HEAD'),
            'NOVA_1811_STACK_CLEAN': clean,
        })

    def push_change(self, content='updated'):
        (self.source / 'version.txt').write_text(content + '\n')
        self.git('-C', str(self.source), 'add', 'version.txt')
        self.git('-C', str(self.source), 'commit', '-m', content)
        self.git('-C', str(self.source), 'push')
        return self.git('-C', str(self.source), 'rev-parse', 'HEAD')

    def update(self, action, *args):
        return self.update_output(action, *args).splitlines()[0]

    def update_output(self, action, *args):
        return subprocess.run(['bash', str(self.vehicle / 'scripts/vehicle.sh'), action, *args],
                              env=self.env, text=True, capture_output=True, timeout=15, check=True).stdout.strip()

    def head(self):
        return self.git('-C', str(self.vehicle), 'rev-parse', 'HEAD')

    def test_checks_and_fast_forwards_only_the_running_branch(self):
        original = self.head()
        self.git('-C', str(self.source), 'switch', 'main')
        self.push_change('main-only')
        self.assertEqual(self.update('check-update'), 'CURRENT')
        self.git('-C', str(self.source), 'switch', 'blackwall')
        updater = self.source / 'scripts/vehicle.sh'
        updater.write_text(updater.read_text() + '\n# updated updater\n')
        self.git('-C', str(self.source), 'add', 'scripts/vehicle.sh')
        target = self.push_change('blackwall-update')
        tracking_before = self.git('-C', str(self.vehicle), 'rev-parse', 'refs/remotes/origin/blackwall')
        self.assertEqual(self.update('check-update'), f'AVAILABLE {target} 1 READY')
        self.assertEqual(self.update_output('check-update').splitlines()[1:], ['blackwall-update'])
        self.assertEqual(self.head(), original)
        self.assertEqual(self.git('-C', str(self.vehicle), 'rev-parse', 'refs/remotes/origin/blackwall'), tracking_before)
        self.assertEqual(self.git('-C', str(self.vehicle), 'for-each-ref', '--format=%(refname)', 'refs/nova-ui'), '')
        self.assertEqual(self.update('apply-update', target), 'UPDATED')
        self.assertEqual(self.head(), target)
        self.assertEqual(self.git('-C', str(self.vehicle), 'branch', '--show-current'), 'blackwall')
        self.assertEqual((self.vehicle / 'version.txt').read_text(), 'blackwall-update\n')
        self.assertIn('# updated updater', (self.vehicle / 'scripts/vehicle.sh').read_text())
        self.assertEqual(self.git('-C', str(self.vehicle), 'for-each-ref', '--format=%(refname)', 'refs/nova-ui'), '')

    def test_local_changes_and_dirty_start_are_not_overwritten(self):
        target = self.push_change()
        original = self.head()
        for name in ('version.txt', 'new-untracked.txt'):
            with self.subTest(name=name):
                path = self.vehicle / name
                previous = path.read_text() if path.exists() else None
                path.write_text('local work\n')
                self.assertEqual(self.update('check-update'), f'AVAILABLE {target} 1 DIRTY')
                self.assertEqual(self.update('apply-update', target), 'DIRTY')
                self.assertEqual(path.read_text(), 'local work\n')
                self.assertEqual(self.head(), original)
                if previous is None: path.unlink()
                else: path.write_text(previous)
        self.snapshot(clean='0')
        self.assertEqual(self.update('check-update'), f'AVAILABLE {target} 1 RESTART')
        self.assertEqual(self.update('apply-update', target), 'RESTART')

    def test_changed_or_detached_checkout_is_not_updated(self):
        target = self.push_change()
        self.git('-C', str(self.vehicle), 'switch', 'main')
        self.assertEqual(self.update('check-update'), 'UNVERIFIED')
        self.assertEqual(self.update('apply-update', target), 'UNVERIFIED')
        self.git('-C', str(self.vehicle), 'switch', '--detach')
        self.assertEqual(self.update('check-update'), 'UNVERIFIED')

    def test_remote_changes_after_check_require_another_check(self):
        first = self.push_change('first')
        original = self.head()
        self.assertEqual(self.update('check-update'), f'AVAILABLE {first} 1 READY')
        second = self.push_change('second')
        self.assertEqual(self.update('apply-update', first), 'STALE')
        self.assertEqual(self.head(), original)
        self.assertEqual(self.update('check-update'), f'AVAILABLE {second} 2 READY')

    def test_diverged_history_is_not_updated(self):
        self.push_change('remote')
        (self.vehicle / 'local.txt').write_text('local\n')
        self.git('-C', str(self.vehicle), 'add', 'local.txt')
        self.git('-C', str(self.vehicle), 'commit', '-m', 'local')
        self.snapshot()
        self.assertEqual(self.update('check-update'), 'DIVERGED')

    def test_locally_ahead_history_is_not_updated(self):
        (self.vehicle / 'local.txt').write_text('local\n')
        self.git('-C', str(self.vehicle), 'add', 'local.txt')
        self.git('-C', str(self.vehicle), 'commit', '-m', 'local')
        self.snapshot()
        self.assertEqual(self.update('check-update'), 'AHEAD')

    def test_ignored_files_are_not_overwritten(self):
        local = self.vehicle / 'ignored.txt'
        local.write_text('important local data\n')
        (self.source / 'ignored.txt').write_text('remote version\n')
        self.git('-C', str(self.source), 'add', '-f', 'ignored.txt')
        self.git('-C', str(self.source), 'commit', '-m', 'add ignored path')
        self.git('-C', str(self.source), 'push')
        target = self.git('-C', str(self.source), 'rev-parse', 'HEAD')
        original = self.head()
        self.assertEqual(self.update('check-update'), f'AVAILABLE {target} 1 READY')
        self.assertEqual(self.update('apply-update', target), 'UPDATE_FAILED')
        self.assertEqual(local.read_text(), 'important local data\n')
        self.assertEqual(self.head(), original)

    def test_missing_snapshot_or_unreachable_remote_fails_closed(self):
        self.env.pop('NOVA_1811_STACK_BRANCH')
        self.assertEqual(self.update('check-update'), 'UNVERIFIED')
        self.snapshot()
        self.git('-C', str(self.vehicle), 'remote', 'set-url', 'origin', str(self.origin / 'missing'))
        self.assertEqual(self.update('check-update'), 'NETWORK')

    def test_changed_submodule_is_not_silently_left_at_the_old_revision(self):
        original = self.head()
        self.git('-C', str(self.source), 'update-index', '--add', '--cacheinfo',
                 f'160000,{original},component')
        self.git('-C', str(self.source), 'commit', '-m', 'update dependency')
        self.git('-C', str(self.source), 'push')
        target = self.git('-C', str(self.source), 'rev-parse', 'HEAD')
        self.assertEqual(self.update('check-update'), f'AVAILABLE {target} 1 SUBMODULE')
        self.assertEqual(self.update('apply-update', target), 'SUBMODULE')
        self.assertEqual(self.head(), original)


class GUpdateRestartTest(unittest.TestCase):
    # Exercise the real host supervisor and Git updater with a fake ROS launch.
    # The origin, both checkouts, and every launched process are temporary.
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='vehicle restart ')
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.origin, self.seed, self.vehicle = root / 'origin.git', root / 'seed', root / 'vehicle'
        self.bin = root / 'bin'
        self.bin.mkdir()
        self.events = root / 'events.jsonl'
        self.env = os.environ.copy()
        self.env.pop('DISPLAY', None)
        self.env.update({
            'PATH': f'{self.bin}:{self.env["PATH"]}', 'HOME': str(root),
            'NOVA_TEST_ROOT': str(self.vehicle), 'NOVA_TEST_EVENTS': str(self.events),
            'GIT_CONFIG_NOSYSTEM': '1', 'GIT_CONFIG_GLOBAL': '/dev/null',
            'GIT_AUTHOR_NAME': 'Test', 'GIT_AUTHOR_EMAIL': 'test@example.invalid',
            'GIT_COMMITTER_NAME': 'Test', 'GIT_COMMITTER_EMAIL': 'test@example.invalid',
        })
        self.git('init', '--bare', '--initial-branch=main', str(self.origin))
        self.git('clone', str(self.origin), str(self.seed))
        (self.seed / 'scripts').mkdir()
        for path in ('g', 'scripts/vehicle.sh', 'Dockerfile', 'docker-compose.yml', '.gitignore'):
            shutil.copy2(REPO / path, self.seed / path)
        (self.seed / 'version.txt').write_text('original\n')
        self.git('-C', str(self.seed), 'add', '.')
        self.git('-C', str(self.seed), 'commit', '-m', 'initial')
        self.git('-C', str(self.seed), 'push', '-u', 'origin', 'main')
        self.git('clone', str(self.origin), str(self.vehicle))
        self.stub('flock', '#!/bin/sh\nshift\nexec "$@"\n')
        self.stub('colcon', '#!/bin/sh\nmkdir -p install\n: > install/setup.bash\n')
        self.stub('docker', '''#!/usr/bin/env python3
import os, sys
args = sys.argv[1:]
if args[:4] == ['compose', 'config', '--images', 'dev']:
    print('test-image')
    sys.exit(0)
if args[:2] == ['image', 'inspect']:
    sys.exit(1)
if args[:2] == ['compose', 'build'] or args[:3] == ['compose', 'up', '-d']:
    sys.exit(0)
if args[:2] != ['compose', 'exec']:
    sys.exit(2)
if args[2] != '-T' and not os.isatty(0):
    print('cannot attach stdin to a TTY-enabled container because stdin is not a terminal', file=sys.stderr)
    sys.exit(1)
index, env = 2, os.environ.copy()
while args[index].startswith('-'):
    if args[index] == '-e':
        key, value = args[index + 1].split('=', 1)
        env[key] = os.environ['NOVA_TEST_ROOT'] if key == 'NOVA_1811_STACK_ROOT' else value
        index += 2
    else:
        index += 1
if args[index] != 'dev':
    sys.exit(2)
command = args[index + 1:]
command = [os.path.join(os.environ['NOVA_TEST_ROOT'], 'scripts/vehicle.sh')
           if value == '/vehicle_1811/scripts/vehicle.sh' else value for value in command]
if command[:2] == ['bash', '-ic']:
    command[1] = '-c'
os.execvpe(command[0], command, env)
''')
        self.stub('ros2', '''#!/usr/bin/env python3
import json, os, signal, sys, time
stopping = False
def stop(*args):
    global stopping
    stopping = True
signal.signal(signal.SIGINT, stop)
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as events:
    events.write(json.dumps(['launch', *sys.argv[1:]]) + '\\n')
while not stopping:
    time.sleep(0.02)
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as events:
    events.write(json.dumps(['stopped']) + '\\n')
''')

    def stub(self, name, content):
        path = self.bin / name
        path.write_text(content)
        path.chmod(0o755)

    def git(self, *args):
        return subprocess.run(['git', *args], env=self.env, check=True,
                              text=True, capture_output=True).stdout.strip()

    def wait_for(self, predicate, process, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if process.poll() is not None:
                self.fail(f'Launcher exited early: {process.stdout.read()}')
            time.sleep(0.03)
        self.fail('Timed out waiting for the launch supervisor')

    def recorded(self):
        return [json.loads(line) for line in self.events.read_text().splitlines()] if self.events.exists() else []

    def exercise_restart(self, dirty=False):
        process = subprocess.Popen([str(self.vehicle / 'g'), 'hub:=true', 'port:=/dev/test'],
                                   cwd=self.vehicle, env=self.env, text=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            self.wait_for(lambda: len(self.recorded()) >= 1, process)
            original = self.git('-C', str(self.vehicle), 'rev-parse', 'HEAD')
            (self.seed / 'version.txt').write_text('updated\n')
            self.git('-C', str(self.seed), 'commit', '-am', 'update sensors')
            self.git('-C', str(self.seed), 'push')
            target = self.git('-C', str(self.seed), 'rev-parse', 'HEAD')
            sessions = list((self.vehicle / '.nova-ui-startup').glob('run.*'))
            self.assertEqual(len(sessions), 1)
            session = sessions[0]
            self.assertEqual((session / 'revision').read_text().strip(), original)
            if dirty:
                # Work created after the UI check must still be protected.
                (self.vehicle / 'version.txt').write_text('local work\n')
            (session / 'request').write_text(target + '\n')
            os.kill(int((session / 'supervisor_pid').read_text()), signal.SIGUSR1)
            self.wait_for(lambda: len([e for e in self.recorded() if e[0] == 'launch']) == 2, process)
            events = self.recorded()
            self.assertEqual([event[0] for event in events[:3]], ['launch', 'stopped', 'launch'])
            expected = original if dirty else target
            self.assertEqual(self.git('-C', str(self.vehicle), 'rev-parse', 'HEAD'), expected)
            self.assertEqual((self.vehicle / 'version.txt').read_text(), 'local work\n' if dirty else 'updated\n')
            self.assertIn('jetson_update:=true', events[2])
            self.assertIn('jetson_revision:=' + expected, events[2])
            self.assertIn('hub_update_result:=' + ('DIRTY' if dirty else 'UPDATED'), events[2])
        finally:
            if process.poll() is None:
                process.terminate()
            try:
                process.communicate(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate()

    def test_update_stops_launch_fast_forwards_and_restarts_g(self):
        self.exercise_restart()

    def test_new_local_work_is_preserved_and_stack_restarts(self):
        self.exercise_restart(dirty=True)


class GStartupTest(unittest.TestCase):
    """Check host launch options and build snapshots with stubbed commands."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "scripts").mkdir()
        (self.root / "bin").mkdir()
        for name in ("g", "scripts/vehicle.sh", "Dockerfile", "docker-compose.yml"):
            shutil.copy2(REPO / name, self.root / name)

        self.events = self.root / "events.jsonl"
        self.env = os.environ.copy()
        self.env.update({
            "PATH": f"{self.root / 'bin'}:{self.env['PATH']}",
            "DISPLAY": ":test",
            "NOVA_TEST_ROOT": str(self.root),
            "NOVA_TEST_EVENTS": str(self.events),
        })
        # The dev command normally prepares ~/.ssh; never touch the real home in tests.
        self.stub("mkdir", "#!/bin/sh\nexit 0\n")
        self.stub("chmod", "#!/bin/sh\nexit 0\n")
        self.stub("flock", "#!/bin/sh\nshift\nexec \"$@\"\n")
        self.stub("git", '''#!/usr/bin/env python3
import os, sys
args = sys.argv[1:]
if args == ['symbolic-ref', '--quiet', '--short', 'HEAD']:
    print('other/test' if os.environ.get('NOVA_TEST_BRANCH_AFTER_FILE') and
          os.path.exists(os.environ['NOVA_TEST_BRANCH_AFTER_FILE']) else 'feature/test')
elif args == ['rev-parse', '--verify', 'HEAD']:
    print('a' * 40)
elif args == ['rev-parse', '--show-toplevel']:
    print(os.environ['NOVA_TEST_ROOT'])
elif args[:2] == ['status', '--porcelain']:
    print(os.environ.get('NOVA_TEST_GIT_CHANGES', ''), end='')
else:
    sys.exit(2)
''')
        self.stub("xhost", '''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['xhost', *sys.argv[1:]]) + '\\n')
sys.exit(int(os.environ.get('NOVA_TEST_XHOST_EXIT', '0')))
''')
        self.stub("colcon", '''#!/usr/bin/env python3
import json, os, pathlib, sys
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['colcon', *sys.argv[1:]]) + '\\n')
if os.environ.get('NOVA_TEST_BUILD_FAIL'):
    sys.exit(9)
pathlib.Path('install').mkdir(exist_ok=True)
pathlib.Path('install/setup.bash').write_text('export NOVA_TEST_SOURCED=1\\n')
if os.environ.get('NOVA_TEST_BRANCH_AFTER_FILE'):
    pathlib.Path(os.environ['NOVA_TEST_BRANCH_AFTER_FILE']).write_text('changed')
''')
        self.stub("ros2", '''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['ros2', os.environ.get('NOVA_TEST_SOURCED'), *sys.argv[1:]]) + '\\n')
if os.environ.get('NOVA_TEST_STACK_ENV'):
    with open(os.environ['NOVA_TEST_STACK_ENV'], 'w') as f:
        json.dump({name: os.environ.get('NOVA_1811_STACK_' + name) for name in
                   ('ROOT', 'BRANCH', 'REVISION', 'CLEAN')}, f)
''')
        self.stub("docker", '''#!/usr/bin/env python3
import json, os, pathlib, sys
args = sys.argv[1:]
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['docker', *args[:3]]) + '\\n')
image_label = pathlib.Path(os.environ['NOVA_TEST_ROOT']) / 'image-label'
if args == ['compose', 'config', '--images', 'dev']:
    print('vehicle_1811')
    sys.exit(0)
if args[:2] == ['image', 'inspect']:
    if not image_label.exists():
        sys.exit(1)
    print(image_label.read_text())
    sys.exit(0)
if args[:2] == ['compose', 'build']:
    if os.environ.get('NOVA_TEST_IMAGE_BUILD_FAIL'):
        sys.exit(8)
    build_arg = args[args.index('--build-arg') + 1]
    image_label.write_text(build_arg.split('=', 1)[1])
    sys.exit(0)
if args[:3] == ['compose', 'up', '-d']:
    sys.exit(0)
if args[:2] != ['compose', 'exec']:
    sys.exit(2)
index = 2
no_tty = args[index] == '-T'
if no_tty:
    index += 1
elif not os.isatty(0):
    print('cannot attach stdin to a TTY-enabled container because stdin is not a terminal', file=sys.stderr)
    sys.exit(1)
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['compose_exec', no_tty, os.isatty(0), *args[index:index + 3]]) + '\\n')
if args[index:index + 3] != ['dev', 'bash', '-ic']:
    sys.exit(2)
if os.environ.get('NOVA_TEST_DEV_SHELL'):
    sys.exit(0)
script, command = args[index + 3], args[index + 4:]
command = [os.path.join(os.environ['NOVA_TEST_ROOT'], 'scripts/vehicle.sh')
           if arg == '/vehicle_1811/scripts/vehicle.sh' else arg for arg in command]
os.execv('/bin/bash', ['bash', '-c', script, *command])
''')

    def stub(self, name, content):
        path = self.root / "bin" / name
        path.write_text(content)
        path.chmod(0o755)

    def run_g(self, *args, display=True, **extra_env):
        env = self.env.copy()
        env.update(extra_env)
        if not display:
            env.pop("DISPLAY", None)
        result = subprocess.run(
            [str(self.root / "g"), "port:=/dev/test", "lidar_ip:=1.2.3.4", *args],
            cwd=self.root, env=env, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=10,
        )
        events = [json.loads(line) for line in self.events.read_text().splitlines()]
        return result, events

    def test_default_uses_ros_lidar_time_and_starts_panorama(self):
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["xhost", "+SI:localuser:root"], events)
        self.assertIn(["colcon", "build", "--symlink-install", "--packages-ignore",
                       "umrr_ros2_driver", "umrr_ros2_msgs", "smart_rviz_plugin"], events)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertEqual(launch[1:5], ["1", "launch", "obc_bringup", "bringup.launch.py"])
        self.assertIn("panorama:=true", launch)
        self.assertIn("lidar_timestamp_mode:=TIME_FROM_ROS_TIME", launch)
        self.assertIn("hub:=true", launch)
        self.assertIn("rviz:=false", launch)
        self.assertIn("jetson_update:=true", launch)
        self.assertIn("jetson_branch:=feature/test", launch)
        self.assertIn("jetson_revision:=" + "a" * 40, launch)

    def test_supervised_launch_from_terminal_keeps_stdin_attached(self):
        # Create the session so ./g actually uses its background supervisor.
        (self.root / '.nova-ui-startup').mkdir()
        master, slave = pty.openpty()
        try:
            result = subprocess.run(
                [str(self.root / 'g'), 'port:=/dev/test'], cwd=self.root, env=self.env,
                stdin=slave, capture_output=True, text=True, timeout=10)
        finally:
            os.close(master)
            os.close(slave)
        self.assertEqual(result.returncode, 0, result.stderr)
        events = [json.loads(line) for line in self.events.read_text().splitlines()]
        self.assertIn(['compose_exec', False, True, 'dev', 'bash', '-ic'], events)
        self.assertTrue(any(event[0] == 'ros2' for event in events))

    def test_supervised_launch_without_terminal_disables_tty(self):
        (self.root / '.nova-ui-startup').mkdir()
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(['compose_exec', True, False, 'dev', 'bash', '-ic'], events)
        self.assertTrue(any(event[0] == 'ros2' for event in events))

    def test_development_shell_still_uses_a_terminal(self):
        env = dict(self.env, NOVA_TEST_DEV_SHELL='1')
        master, slave = pty.openpty()
        try:
            result = subprocess.run(['bash', str(self.root / 'scripts/vehicle.sh'), 'dev'],
                                    cwd=self.root, env=env, stdin=slave,
                                    capture_output=True, text=True, timeout=10)
        finally:
            os.close(master)
            os.close(slave)
        self.assertEqual(result.returncode, 0, result.stderr)
        events = [json.loads(line) for line in self.events.read_text().splitlines()]
        self.assertIn(['compose_exec', False, True, 'dev', 'bash', '-ic'], events)

    def test_early_hub_splash_passes_its_session_to_launch(self):
        # The real helper uses either host PyQt or the already-built image.
        # A stub records the request without opening a window in this test.
        source = self.root / 'ros2_ws/src/vehicle_ui_hub/src/screens.py'
        source.parent.mkdir(parents=True)
        source.write_text('import json, os\n'
                          'with open(os.environ["NOVA_TEST_EVENTS"], "a") as f:\n'
                          '    f.write(json.dumps(["host-splash"]) + "\\n")\n')
        (self.root / '.nova-ui-startup').mkdir()
        (self.root / 'image-label').write_text('existing image')
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == 'ros2')
        self.assertTrue(any(arg.startswith('hub_startup_token:=session.') for arg in launch))
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            events = [json.loads(line) for line in self.events.read_text().splitlines()]
            if any(event[0] == 'host-splash' or event[:2] == ['docker', 'run'] for event in events):
                break
            time.sleep(0.02)
        self.assertTrue(any(event[0] == 'host-splash' or event[:2] == ['docker', 'run'] for event in events))

    def test_image_is_built_only_when_the_recipe_changes(self):
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        build_event = ["docker", "compose", "build", "--build-arg"]
        self.assertEqual(events.count(build_event), 1)

        # Source files are bind-mounted and should not trigger an image build.
        (self.root / 'source.cpp').write_text('changed source')
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(events.count(build_event), 1)

        with (self.root / 'Dockerfile').open('a') as dockerfile:
            dockerfile.write('\n# dependency changed\n')
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(events.count(build_event), 2)

        with (self.root / 'docker-compose.yml').open('a') as compose:
            compose.write('\n# configuration changed\n')
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(events.count(build_event), 3)

        # An image rebuilt outside ./g may not have the launcher's fingerprint.
        (self.root / 'image-label').write_text('untracked')
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(events.count(build_event), 4)

    def test_context_copy_lets_docker_check_its_cache(self):
        with (self.root / 'Dockerfile').open('a') as dockerfile:
            dockerfile.write('\nCOPY source.cpp /tmp/source.cpp\n')
        build_event = ["docker", "compose", "build", "--build-arg"]
        result, _ = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(events.count(build_event), 2)

    def test_compose_override_lets_docker_check_its_cache(self):
        build_event = ["docker", "compose", "build", "--build-arg"]
        result, _ = self.run_g(COMPOSE_FILE='custom-compose.yml')
        self.assertEqual(result.returncode, 0, result.stderr)
        result, events = self.run_g(COMPOSE_FILE='custom-compose.yml')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(events.count(build_event), 2)

    def test_image_build_failure_does_not_start_the_container_or_stack(self):
        result, events = self.run_g(NOVA_TEST_IMAGE_BUILD_FAIL='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(event[:3] == ['docker', 'compose', 'up'] for event in events))
        self.assertFalse(any(event[0] in ('colcon', 'ros2') for event in events))

    def test_stack_captures_the_checkout_used_by_the_build(self):
        env_path = self.root / 'stack-env.json'
        result, _ = self.run_g(NOVA_TEST_STACK_ENV=str(env_path))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(env_path.read_text()), {
            'ROOT': str(self.root), 'BRANCH': 'feature/test', 'REVISION': 'a' * 40, 'CLEAN': '1',
        })

    def test_dirty_or_changed_build_cannot_be_used_for_updates(self):
        env_path = self.root / 'stack-env.json'
        result, _ = self.run_g(NOVA_TEST_STACK_ENV=str(env_path), NOVA_TEST_GIT_CHANGES=' M local')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(env_path.read_text())['CLEAN'], '0')
        marker = self.root / 'branch-changed'
        result, _ = self.run_g(NOVA_TEST_STACK_ENV=str(env_path), NOVA_TEST_BRANCH_AFTER_FILE=str(marker))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(env_path.read_text())['CLEAN'], '0')

    def test_jetson_update_can_be_disabled(self):
        result, events = self.run_g("jetson_update:=false")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("jetson_update:=false", launch)
        self.assertFalse(any(arg.startswith("jetson_revision:=") for arg in launch))

    def test_hub_uses_the_display_without_starting_a_second_rviz_window(self):
        for value in ("true", "1"):
            with self.subTest(value=value):
                self.events.unlink(missing_ok=True)
                result, events = self.run_g(f"hub:={value}")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(["xhost", "+SI:localuser:root"], events)
                launch = next(event for event in events if event[0] == "ros2")
                self.assertIn(f"hub:={value}", launch)
                self.assertIn("rviz:=false", launch)

    def test_disabling_hub_uses_the_separate_rviz_view(self):
        result, events = self.run_g("hub:=false")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("hub:=false", launch)
        self.assertIn("rviz:=true", launch)

    def test_explicit_rviz_does_not_implicitly_start_the_hub(self):
        result, events = self.run_g("rviz:=true")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("rviz:=true", launch)
        self.assertFalse(any(arg.startswith("hub:=") for arg in launch))

    def test_hub_can_be_requested_without_a_display(self):
        result, events = self.run_g("hub:=true", display=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DISPLAY is not set", result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("hub:=true", launch)
        self.assertIn("rviz:=false", launch)

    def test_uncommitted_jetson_source_blocks_remote_start(self):
        result, events = self.run_g(NOVA_TEST_GIT_CHANGES=" M scripts/vehicle.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("uncommitted changes", result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("jetson_revision:=", launch)

    def test_explicit_overrides_are_preserved(self):
        result, events = self.run_g("rviz:=false", "panorama:=true",
                                    "lidar_timestamp_mode:=TIME_FROM_PTP_1588",
                                    "registration_on_jetson:=false")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(any(event[0] == "xhost" for event in events))
        launch = next(event for event in events if event[0] == "ros2")
        self.assertEqual([arg for arg in launch if arg.startswith("rviz:=")], ["rviz:=false"])
        self.assertEqual([arg for arg in launch if arg.startswith("panorama:=")], ["panorama:=true"])
        self.assertEqual([arg for arg in launch if arg.startswith("lidar_timestamp_mode:=")],
                         ["lidar_timestamp_mode:=TIME_FROM_PTP_1588"])
        self.assertIn("registration_on_jetson:=false", launch)

    def test_no_display_starts_headless(self):
        result, events = self.run_g(display=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("No DISPLAY", result.stderr)
        self.assertFalse(any(event[0] == "xhost" for event in events))
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("panorama:=true", launch)
        self.assertIn("lidar_timestamp_mode:=TIME_FROM_ROS_TIME", launch)
        self.assertFalse(any(arg.startswith("rviz:=") for arg in launch))

    def test_disabling_cameras_does_not_enable_panorama(self):
        result, events = self.run_g("cameras:=false")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertEqual([arg for arg in launch if arg.startswith("panorama:=")], ["panorama:=false"])

    def test_explicit_panorama_override_takes_precedence(self):
        for args, expected in ((('panorama:=false',), 'panorama:=false'),
                               (('cameras:=false', 'panorama:=true'), 'panorama:=true')):
            with self.subTest(args=args):
                self.events.unlink(missing_ok=True)
                result, events = self.run_g(*args)
                self.assertEqual(result.returncode, 0, result.stderr)
                launch = next(event for event in events if event[0] == "ros2")
                self.assertEqual([arg for arg in launch if arg.startswith("panorama:=")], [expected])

    def test_empty_timestamp_override_is_forwarded(self):
        result, events = self.run_g("lidar_timestamp_mode:=")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertEqual([arg for arg in launch if arg.startswith("lidar_timestamp_mode:=")],
                         ["lidar_timestamp_mode:="])

    def test_build_failure_does_not_start_vehicle(self):
        result, events = self.run_g(NOVA_TEST_BUILD_FAIL="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(event[0] == "ros2" for event in events))

    def test_arguments_are_forwarded_without_shell_evaluation(self):
        marker = self.root / "unexpected"
        argument = f"jetson_repo:=literal $(touch {marker})"
        result, events = self.run_g(argument)
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn(argument, launch)
        self.assertFalse(marker.exists())

    def test_xhost_failure_does_not_stop_vehicle(self):
        result, events = self.run_g(NOVA_TEST_XHOST_EXIT="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Could not grant graphical display access", result.stderr)
        self.assertTrue(any(event[0] == "ros2" for event in events))


class JetsonPrepareTest(unittest.TestCase):
    """Check remote camera and registration preparation without a Jetson."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='jetson prepare ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'scripts').mkdir()
        (self.root / 'bin').mkdir()
        (self.root / 'ros2_ws').mkdir()
        (self.root / 'zed_ws/install').mkdir(parents=True)
        (self.root / 'zed_ws/install/local_setup.bash').write_text('')
        self.events = self.root / 'events'
        self.script = self.root / 'scripts/vehicle.sh'
        content = START_SCRIPT.read_text()
        original = 'LOG_DIR="$HOME/.ros/jetson_cameras"'
        self.assertIn(original, content)
        self.script.write_text(content.replace(original, f'LOG_DIR={shlex.quote(str(self.root / "logs"))}'))
        colcon = self.root / 'bin/colcon'
        colcon.write_text('''#!/usr/bin/env python3
import json, os, pathlib, sys
pathlib.Path(os.environ['NOVA_TEST_PREPARE_EVENTS']).write_text(json.dumps(sys.argv[1:]))
if os.environ.get('NOVA_TEST_PREPARE_FAIL'):
    sys.exit(2)
pathlib.Path('install').mkdir(exist_ok=True)
pathlib.Path('install/local_setup.bash').write_text('')
''')
        colcon.chmod(0o755)
        self.env = os.environ.copy()
        self.env.update({
            'PATH': f'{self.root / "bin"}:{self.env["PATH"]}',
            'ZED_WS': str(self.root / 'zed_ws'),
            'NOVA_TEST_PREPARE_EVENTS': str(self.events),
        })

    def run_prepare(self, fail=False):
        env = self.env.copy()
        if fail:
            env['NOVA_TEST_PREPARE_FAIL'] = '1'
        return subprocess.run(['bash', str(self.script), 'cameras', '--prepare', '--no-cameras'],
                              env=env, text=True, capture_output=True, timeout=10)

    def test_builds_all_jetson_packages_before_starting(self):
        result = self.run_prepare()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Jetson packages ready', result.stdout)
        self.assertEqual(json.loads(self.events.read_text()),
                         ['build', '--symlink-install', '--packages-select',
                          'jetson_bringup', 'camera_panorama', 'lidar_perception'])

    def test_build_failure_prevents_start(self):
        result = self.run_prepare(fail=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Jetson package build failed', result.stdout)
        self.assertNotIn('Jetson packages ready', result.stdout)

    def prepare_registration(self, installed_config=False, legacy_running=False):
        package = self.root / 'ros2_ws/src/lidar_perception'
        installed = self.root / 'ros2_ws/install/lidar_perception'
        (package / 'launch').mkdir(parents=True)
        (package / 'config').mkdir()
        (installed / 'lib/lidar_perception').mkdir(parents=True)
        (installed / 'share/lidar_perception/launch').mkdir(parents=True)
        (self.root / 'ros2_ws/install/local_setup.bash').write_text('')
        # Detector and launch match the installed versions, but the YAML is missing.
        (package / 'lidar_camera_registration.py').write_text('detector')
        (installed / 'lib/lidar_perception/lidar_camera_registration.py').write_text('detector')
        (package / 'interactive_calibration.py').write_text('hub backend')
        (installed / 'lib/lidar_perception/interactive_calibration.py').write_text('hub backend')
        (package / 'launch/lidar_camera_registration.launch.py').write_text('launch')
        (installed / 'share/lidar_perception/launch/lidar_camera_registration.launch.py').write_text('launch')
        (package / 'config/registration.yaml').write_text('config')
        if installed_config:
            (installed / 'share/lidar_perception/config').mkdir()
            (installed / 'share/lidar_perception/config/registration.yaml').write_text('config')

        python = self.root / 'bin/python3'
        python.write_text('#!/bin/sh\nif [ "$1" = "-c" ]; then exit 0; fi\n'
                          f'exec {shlex.quote(sys.executable)} "$@"\n')
        python.chmod(0o755)
        pgrep_body = ('case "$2" in *corner_detector*) exit 0 ;; *) exit 1 ;; esac'
                      if legacy_running else 'exit 1')
        self.registration_args = self.root / 'registration-args'
        self.env['NOVA_TEST_REGISTRATION_ARGS'] = str(self.registration_args)
        for name, body in (('pgrep', pgrep_body),
                           ('ros2', 'printf "%s\\n" "$@" > "$NOVA_TEST_REGISTRATION_ARGS"'),
                           ('stty', 'if [ "$1" = -g ]; then echo test-tty; fi\nexit 0')):
            executable = self.root / 'bin' / name
            executable.write_text(f'#!/bin/sh\n{body}\n')
            executable.chmod(0o755)

    def test_missing_config_triggers_registration_build(self):
        self.prepare_registration()
        # The legacy option remains supported and starts the same renamed launch.
        for switch in ('--registration', '--corners'):
            with self.subTest(switch=switch):
                result = subprocess.run(['bash', str(self.script), 'cameras', switch, '--no-cameras'],
                                        env=self.env, text=True, capture_output=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(json.loads(self.events.read_text()),
                                 ['build', '--symlink-install', '--packages-select',
                                  'camera_panorama', 'lidar_perception'])
                self.assertEqual(self.registration_args.read_text().splitlines(),
                                 ['launch', 'lidar_perception', 'lidar_camera_registration.launch.py'])

    def test_old_registration_process_prevents_a_duplicate(self):
        self.prepare_registration(installed_config=True, legacy_running=True)
        result = subprocess.run(['bash', str(self.script), 'cameras', '--registration', '--no-cameras'],
                                env=self.env, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('old corner_detector launch is still running', result.stdout)
        self.assertFalse(self.registration_args.exists())

    def test_hub_launch_enables_operator_approval_on_jetson(self):
        self.prepare_registration(installed_config=True)
        result = subprocess.run(['bash', str(self.script), 'cameras', '--registration', '--hub-calibration', '--no-cameras'],
                                env=self.env, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('interactive_calibration:=true', self.registration_args.read_text().splitlines())

    def test_terminal_shows_registration_results_and_errors_without_progress_noise(self):
        self.prepare_registration(installed_config=True)
        ros2 = self.root / 'bin/ros2'
        ros2.write_text('''#!/bin/sh
printf '%s\\n' 'Calibration quality front: starting FPFH/RANSAC' \\
  'Wall filter front: 2 patches' 'Calibrated ZED cameras: front' \\
  'Skipped ZED clouds: left: timestamp difference 1.00s' \\
  'Too few LiDAR points after filtering (12)' '[ERROR] registration failed'
sleep 2
''')
        result = subprocess.run(['bash', str(self.script), 'cameras', '--registration', '--no-cameras'],
                                env=self.env, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for expected in ('Calibrated ZED cameras: front', 'Skipped ZED clouds:',
                         'Too few LiDAR points', '[ERROR] registration failed'):
            self.assertIn(expected, result.stdout)
        self.assertNotIn('Calibration quality front', result.stdout)
        self.assertNotIn('Wall filter front', result.stdout)


class JetsonSyncTest(unittest.TestCase):
    """Check safe checkout synchronization with temporary Git repositories."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='jetson sync ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.origin = self.root / 'origin.git'
        self.karbon = self.root / 'karbon'
        self.jetson = self.root / 'jetson repo'
        self.env = os.environ.copy()
        self.env.update({
            'GIT_CONFIG_NOSYSTEM': '1',
            'GIT_CONFIG_GLOBAL': '/dev/null',
            'GIT_TERMINAL_PROMPT': '0',
            'GIT_AUTHOR_NAME': 'Test',
            'GIT_AUTHOR_EMAIL': 'test@example.invalid',
            'GIT_COMMITTER_NAME': 'Test',
            'GIT_COMMITTER_EMAIL': 'test@example.invalid',
        })
        bin_dir = self.root / 'bin'
        bin_dir.mkdir()
        pgrep = bin_dir / 'pgrep'
        pgrep.write_text('''#!/bin/sh
if [ -n "${NOVA_TEST_ACTIVE_PROCESS:-}" ]; then
    printf '%s\\n' "$NOVA_TEST_ACTIVE_PROCESS" | grep -Eq -- "$2"
    exit $?
fi
[ "${NOVA_TEST_ACTIVE:-0}" = 1 ]
''')
        pgrep.chmod(0o755)
        ssh = bin_dir / 'ssh'
        ssh.write_text('''#!/bin/bash
if [[ "$1" == -T ]]; then
    printf 'prepare\\n' >> "$NOVA_TEST_SSH_EVENTS"
    exec bash -c "${@: -1}"
fi
printf 'start\\n' >> "$NOVA_TEST_SSH_EVENTS"
''')
        ssh.chmod(0o755)
        setsid = bin_dir / 'setsid'
        setsid.write_text('#!/bin/sh\nexec "$@"\n')
        setsid.chmod(0o755)
        self.ssh_events = self.root / 'ssh-events'
        self.env['NOVA_TEST_SSH_EVENTS'] = str(self.ssh_events)
        self.env['PATH'] = f'{bin_dir}:{self.env["PATH"]}'
        self.git('init', '--bare', '--initial-branch=main', str(self.origin))
        self.git('clone', str(self.origin), str(self.karbon))
        (self.karbon / 'source.txt').write_text('initial\n')
        (self.karbon / '.gitignore').write_text('ignored.txt\n')
        self.git('-C', str(self.karbon), 'add', '.')
        self.git('-C', str(self.karbon), 'commit', '-m', 'initial')
        self.git('-C', str(self.karbon), 'push', '-u', 'origin', 'main')
        self.git('clone', str(self.origin), str(self.jetson))

    def git(self, *args):
        return subprocess.run(['git', *args], env=self.env, check=True,
                              text=True, capture_output=True).stdout.strip()

    def commit_update(self, name='update'):
        (self.karbon / 'source.txt').write_text(name + '\n')
        self.git('-C', str(self.karbon), 'add', 'source.txt')
        self.git('-C', str(self.karbon), 'commit', '-m', name)
        branch = self.git('-C', str(self.karbon), 'branch', '--show-current')
        self.git('-C', str(self.karbon), 'push', '-u', 'origin', branch)
        return self.git('-C', str(self.karbon), 'rev-parse', 'HEAD')

    def sync(self, revision, branch='main', active=False, process=None):
        env = self.env.copy()
        if active:
            env['NOVA_TEST_ACTIVE'] = '1'
        if process is not None:
            env['NOVA_TEST_ACTIVE_PROCESS'] = process
        return subprocess.run(['bash', str(UPDATER), str(self.jetson), branch, revision],
                              env=env, text=True, capture_output=True, timeout=10)

    def jetson_head(self):
        return self.git('-C', str(self.jetson), 'rev-parse', 'HEAD')

    def preflight(self, revision):
        env = dict(self.env, NOVA_1811_STACK_BRANCH='main', NOVA_UI_JETSON_UPDATE='true',
                   NOVA_UI_JETSON_CAMERAS='true', NOVA_UI_JETSON_REGISTRATION='true',
                   NOVA_UI_JETSON_HOST='test@jetson', NOVA_UI_JETSON_KEY='/fake/key',
                   NOVA_UI_JETSON_REPO=str(self.jetson), NOVA_TEST_ACTIVE='1')
        return subprocess.run(['bash', str(START_SCRIPT), 'check-jetson-update', revision],
                              env=env, text=True, capture_output=True, timeout=15, check=True)

    def test_preflight_checks_jetson_without_stopping_or_changing_worktree(self):
        old_head = self.jetson_head()
        revision = self.commit_update()
        result = self.preflight(revision)
        self.assertEqual(result.stdout.strip(), 'JETSON_READY')
        self.assertEqual(self.jetson_head(), old_head)
        self.assertEqual((self.jetson / 'source.txt').read_text(), 'initial\n')
        (self.jetson / 'local work.txt').write_text('keep me\n')
        result = self.preflight(revision)
        self.assertEqual(result.stdout.strip(), 'JETSON_DIRTY')
        self.assertEqual(self.jetson_head(), old_head)
        self.assertEqual((self.jetson / 'local work.txt').read_text(), 'keep me\n')

    def test_preflight_detects_diverged_jetson_branch(self):
        (self.jetson / 'source.txt').write_text('Jetson commit\n')
        self.git('-C', str(self.jetson), 'commit', '-am', 'Jetson work')
        old_head = self.jetson_head()
        revision = self.commit_update('Karbon commit')
        result = self.preflight(revision)
        self.assertEqual(result.stdout.strip(), 'JETSON_DIVERGED')
        self.assertEqual(self.jetson_head(), old_head)

    def test_fetches_and_fast_forwards_to_karbon_commit(self):
        revision = self.commit_update()
        result = self.sync(revision)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.jetson_head(), revision)
        self.assertEqual((self.jetson / 'source.txt').read_text(), 'update\n')

    def test_switches_clean_checkout_to_karbon_branch(self):
        self.git('-C', str(self.karbon), 'switch', '-c', 'feature/cameras')
        revision = self.commit_update()
        result = self.sync(revision, 'feature/cameras')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.jetson_head(), revision)
        self.assertEqual(self.git('-C', str(self.jetson), 'branch', '--show-current'),
                         'feature/cameras')

    def test_tracked_or_untracked_changes_are_preserved(self):
        revision = self.commit_update()
        old_head = self.jetson_head()
        (self.jetson / 'source.txt').write_text('local work\n')
        (self.jetson / 'new_file.txt').write_text('more work\n')
        result = self.sync(revision)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('local changes', result.stderr)
        self.assertEqual(self.jetson_head(), old_head)
        self.assertEqual((self.jetson / 'source.txt').read_text(), 'local work\n')
        self.assertEqual((self.jetson / 'new_file.txt').read_text(), 'more work\n')

    def test_diverged_branch_is_preserved(self):
        (self.jetson / 'source.txt').write_text('Jetson commit\n')
        self.git('-C', str(self.jetson), 'commit', '-am', 'Jetson work')
        old_head = self.jetson_head()
        revision = self.commit_update('Karbon commit')
        result = self.sync(revision)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('diverged or is ahead', result.stderr)
        self.assertEqual(self.jetson_head(), old_head)

    def test_running_processes_prevent_checkout_update(self):
        revision = self.commit_update()
        old_head = self.jetson_head()
        result = self.sync(revision, active=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('already running', result.stderr)
        self.assertEqual(self.jetson_head(), old_head)

    def test_old_and_renamed_registration_processes_prevent_update(self):
        revision = self.commit_update()
        old_head = self.jetson_head()
        for launch in ('corner_detector.launch.py', 'lidar_camera_registration.launch.py'):
            with self.subTest(launch=launch):
                result = self.sync(revision, process=f'ros2 launch lidar_perception {launch}')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('already running', result.stderr)
                self.assertEqual(self.jetson_head(), old_head)

    def test_ignored_files_are_not_overwritten(self):
        local_file = self.jetson / 'ignored.txt'
        local_file.write_text('local ignored content\n')
        (self.karbon / 'ignored.txt').write_text('tracked content\n')
        self.git('-C', str(self.karbon), 'add', '-f', 'ignored.txt')
        self.git('-C', str(self.karbon), 'commit', '-m', 'track formerly ignored file')
        self.git('-C', str(self.karbon), 'push')
        revision = self.git('-C', str(self.karbon), 'rev-parse', 'HEAD')
        old_head = self.jetson_head()
        result = self.sync(revision)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(local_file.read_text(), 'local ignored content\n')
        self.assertEqual(self.jetson_head(), old_head)

    def test_unavailable_commit_does_not_change_checkout(self):
        old_head = self.jetson_head()
        result = self.sync('a' * 40)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('not available', result.stderr)
        self.assertEqual(self.jetson_head(), old_head)

    def test_already_matching_checkout_needs_no_update(self):
        revision = self.jetson_head()
        result = self.sync(revision, active=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('already matches', result.stdout)

    def test_ssh_wrapper_updates_before_starting_remote_process(self):
        revision = self.commit_update()
        result = subprocess.run(
            ['bash', str(SSH_WRAPPER), 'test@jetson', '/fake/key', 'start process',
             str(self.jetson), 'main', revision], env=self.env, capture_output=True,
            text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.ssh_events.read_text().splitlines(), ['prepare', 'start'])
        self.assertEqual(self.jetson_head(), revision)

    def test_ssh_wrapper_never_starts_remote_process_if_update_fails(self):
        revision = self.commit_update()
        (self.jetson / 'source.txt').write_text('uncommitted work\n')
        result = subprocess.run(
            ['bash', str(SSH_WRAPPER), 'test@jetson', '/fake/key', 'start process',
             str(self.jetson), 'main', revision], env=self.env, capture_output=True,
            text=True, timeout=10,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('not started', result.stderr)
        self.assertEqual(self.ssh_events.read_text().splitlines(), ['prepare'])


class FloorOverlayTest(unittest.TestCase):
    """Check that the ZED wrapper patch is applied only to a private copy."""

    def test_release_patch_matches_real_wrapper_context(self):
        # Context from Stereolabs zed-ros2-wrapper v5.2.2, commit b47c727.
        # Keep its blank lines: they are significant to git apply.
        snippets = {
            'include/zed_camera_component.hpp': (
                '  objPub mPubBodyTrk;\n'
                '  depthInfoPub mPubDepthInfo;\n'
                '  planePub mPubPlane;\n'
                '  markerPub mPubMarker;\n'
                '\n'
                '  geoPosePub mPubGeoPose;\n'
            ),
            'src/zed_camera_component_main.cpp': (
                '  }\n'
                '\n'
                '  initVideoDepthPublishers();\n'
                '\n'
                '  // POS. TRACKING and GNSS FUSION parameters\n'
                '#if (ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) >= 52\n'
            ),
            'src/zed_camera_component_video_depth.cpp': (
                '#include "zed_camera_component.hpp"\n'
                '#include "sl_logging.hpp"\n'
                '#include "sl_tools.hpp"\n'
                '\n'
                '#include <sensor_msgs/distortion_models.hpp>\n'
                '#include <sensor_msgs/image_encodings.hpp>\n'
                '#include <sensor_msgs/msg/point_field.hpp>\n'
                '\n'
                'void ZedCamera::processPointCloud()\n'
                '{\n'
                '      DEBUG_STREAM_PC(\n'
                '        " * [processPointCloud] Retrieved point cloud size: " << mMatCloud.getWidth() << "x" <<\n'
                '          mMatCloud.getHeight());\n'
                '\n'
                '      // Signal Pointcloud thread that a new pointcloud is ready\n'
                '      mPcDataReadyCondVar.notify_one();\n'
                '      mPcDataReady = true;\n'
                '}\n'
            ),
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, contents in snippets.items():
                target = root / 'zed_components/src/zed_camera' / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(contents)
            result = subprocess.run(['git', 'apply', '--check', str(floor_overlay.PATCH)],
                                    cwd=root, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_patch_builds_a_copy_and_reuses_it_until_source_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            zed_ws = root / 'zed_ws'
            source = zed_ws / 'src/zed-ros2-wrapper/zed_components'
            source.mkdir(parents=True)
            (source / 'package.xml').write_text('<package><version>5.2.2</version></package>')
            (source / 'sample.cpp').write_text('before\n')
            patch_file = root / 'change.patch'
            patch_file.write_text('--- a/zed_components/sample.cpp\n'
                                  '+++ b/zed_components/sample.cpp\n'
                                  '@@ -1 +1 @@\n-before\n+after\n')
            builds = []

            def run(command, cwd):
                if command[0] == 'git':
                    subprocess.run(command, cwd=cwd, check=True, capture_output=True)
                else:
                    builds.append(command)
                    install = Path(command[command.index('--install-base') + 1])
                    (install / 'zed_components/lib').mkdir(parents=True)
                    (install / 'local_setup.bash').write_text('')
                    (install / 'zed_components/lib/libzed_camera_component.so').write_text('fake')

            with patch.object(floor_overlay, 'PATCH', patch_file), patch.object(floor_overlay, 'run', run):
                first = floor_overlay.prepare(zed_ws, root / 'cache')
                second = floor_overlay.prepare(zed_ws, root / 'cache')
                self.assertEqual(first, second)
                self.assertEqual(len(builds), 1)
                self.assertEqual((source / 'sample.cpp').read_text(), 'before\n')
                self.assertEqual((first.parents[1] / 'src/zed_components/sample.cpp').read_text(), 'after\n')
                (source / 'other.cpp').write_text('user change\n')
                third = floor_overlay.prepare(zed_ws, root / 'cache')
                self.assertNotEqual(first, third)
                self.assertEqual(len(builds), 2)
                self.assertEqual((source / 'other.cpp').read_text(), 'user change\n')

    def test_incompatible_patch_or_version_does_not_modify_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            zed_ws = root / 'zed_ws'
            source = zed_ws / 'src/zed_components'
            source.mkdir(parents=True)
            (source / 'package.xml').write_text('<package><version>5.2.2</version></package>')
            target = source / 'sample.cpp'
            target.write_text('user content\n')
            patch_file = root / 'change.patch'
            patch_file.write_text('--- a/zed_components/sample.cpp\n'
                                  '+++ b/zed_components/sample.cpp\n'
                                  '@@ -1 +1 @@\n-missing\n+after\n')
            with patch.object(floor_overlay, 'PATCH', patch_file):
                with self.assertRaises(subprocess.CalledProcessError):
                    floor_overlay.prepare(zed_ws, root / 'cache')
                self.assertEqual(target.read_text(), 'user content\n')
                (source / 'package.xml').write_text('<package><version>5.3.0</version></package>')
                with self.assertRaisesRegex(RuntimeError, 'requires zed_components 5.2.2'):
                    floor_overlay.prepare(zed_ws, root / 'cache')


if __name__ == "__main__":
    unittest.main()
