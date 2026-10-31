"""Check Jetson updates using local throwaway Git repositories, never SSH."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


UPDATER = (Path(__file__).resolve().parents[1] / 'ros2_ws/src/obc_bringup/scripts'
           / 'sync_jetson_repo.sh')
SSH_WRAPPER = UPDATER.with_name('jetson_cameras_ssh.sh')


class JetsonSyncTest(unittest.TestCase):
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
        pgrep.write_text('#!/bin/sh\n[ "${NOVA_TEST_ACTIVE:-0}" = 1 ]\n')
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

    def sync(self, revision, branch='main', active=False):
        env = self.env.copy()
        if active:
            env['NOVA_TEST_ACTIVE'] = '1'
        return subprocess.run(['bash', str(UPDATER), str(self.jetson), branch, revision],
                              env=env, text=True, capture_output=True, timeout=10)

    def jetson_head(self):
        return self.git('-C', str(self.jetson), 'rev-parse', 'HEAD')

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


if __name__ == '__main__':
    unittest.main()
