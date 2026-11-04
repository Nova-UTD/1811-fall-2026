"""Exercise the Jetson package-build step without ROS nodes or hardware."""

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
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
                result = subprocess.run(['bash', str(self.script), switch, '--no-cameras'],
                                        env=self.env, text=True, capture_output=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(json.loads(self.events.read_text()),
                                 ['build', '--symlink-install', '--packages-select',
                                  'camera_panorama', 'lidar_perception'])
                self.assertEqual(self.registration_args.read_text().splitlines(),
                                 ['launch', 'lidar_perception', 'lidar_camera_registration.launch.py'])

    def test_old_registration_process_prevents_a_duplicate(self):
        self.prepare_registration(installed_config=True, legacy_running=True)
        result = subprocess.run(['bash', str(self.script), '--registration', '--no-cameras'],
                                env=self.env, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('old corner_detector launch is still running', result.stdout)
        self.assertFalse(self.registration_args.exists())

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
        result = subprocess.run(['bash', str(self.script), '--registration', '--no-cameras'],
                                env=self.env, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for expected in ('Calibrated ZED cameras: front', 'Skipped ZED clouds:',
                         'Too few LiDAR points', '[ERROR] registration failed'):
            self.assertIn(expected, result.stdout)
        self.assertNotIn('Calibration quality front', result.stdout)
        self.assertNotIn('Wall filter front', result.stdout)


if __name__ == '__main__':
    unittest.main()
