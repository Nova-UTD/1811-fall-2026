"""Exercise the Jetson package-build step without ROS nodes or hardware."""

import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


START_SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/jetson_cameras.sh'


class JetsonPrepareTest(unittest.TestCase):
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
        self.script = self.root / 'scripts/jetson_cameras.sh'
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
        return subprocess.run(['bash', str(self.script), '--prepare', '--no-cameras'],
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


if __name__ == '__main__':
    unittest.main()
