"""Check ZED depth selection and receipt of a PointCloud2 message."""

import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


zed_launch = load_module('zed_launch_test',
                         'ros2_ws/src/jetson_bringup/launch/zed_cameras.launch.py')
bringup = load_module('zed_bringup_test',
                      'ros2_ws/src/obc_bringup/launch/bringup.launch.py')


class ZedDepthLaunchTest(unittest.TestCase):
    def test_lidar_cloud_check_runs_only_when_requested(self):
        actions = bringup.generate_launch_description().entities
        check, = [action for action in actions if type(action) is ExecuteProcess]
        context = LaunchContext()
        context.launch_configurations['lidar_reload_check'] = 'false'
        self.assertFalse(check.condition.evaluate(context))
        context.launch_configurations['lidar_reload_check'] = 'true'
        self.assertTrue(check.condition.evaluate(context))

    def test_vehicle_bringup_enables_ouster_retries(self):
        with patch.object(bringup, 'include', wraps=bringup.include) as include:
            bringup.generate_launch_description()
        ouster = next(call for call in include.call_args_list
                      if call.args[:2] == ('ouster_ros', 'sensor.launch.xml'))
        self.assertEqual(ouster.kwargs['attempt_reconnect'], 'true')
        self.assertEqual(ouster.kwargs['dormant_period_between_reconnects'], '5.0')

    def test_vehicle_bringup_does_not_start_camera_processing_locally(self):
        actions = bringup.generate_launch_description().entities
        local_packages = {action.node_package for action in actions if isinstance(action, Node)}
        self.assertTrue(local_packages.isdisjoint({'lidar_perception', 'camera_panorama',
                                                  'jetson_bringup', 'zed_wrapper'}))
        local_includes = [str(action.launch_description_source.location)
                          for action in actions if isinstance(action, IncludeLaunchDescription)]
        for package in ('lidar_perception', 'camera_panorama', 'jetson_bringup'):
            self.assertFalse(any(package in source for source in local_includes), package)
        self.assertTrue(any('localization' in source for source in local_includes))

    def test_hub_has_time_to_finish_recording_and_update_checks_during_shutdown(self):
        # Humble does not expose sigterm_timeout as a public Node attribute.
        with patch.object(bringup, 'Node', wraps=Node) as node:
            bringup.generate_launch_description()
        hub = next(call for call in node.call_args_list if call.kwargs.get('package') == 'vehicle_ui_hub')
        self.assertEqual(hub.kwargs['sigterm_timeout'], '75')

    def context(self, depth_cameras):
        context = LaunchContext()
        context.launch_configurations.update({
            'cameras': 'front,left,rear,right', 'depth_cameras': depth_cameras,
            'camera_model': 'zedx', 'ground_detection': 'false',
            **{f'{camera}_serial': '0' for camera in zed_launch.POSITIONS},
            **{f'{camera}_id': str(i) for i, camera in enumerate(zed_launch.POSITIONS)},
        })
        return context

    def configs_for(self, depth_cameras):
        context = self.context(depth_cameras)
        configs = {}
        with patch.object(zed_launch.FindPackageShare, 'find', return_value='/test/jetson_bringup'):
            for action in zed_launch.setup(context):
                if isinstance(action, IncludeLaunchDescription):
                    arguments = dict(action.launch_arguments)
                    configs[arguments['camera_name']] = Path(
                        arguments['ros_params_override_path'].perform(context)).name
        return configs

    def test_depth_is_only_enabled_for_selected_cameras(self):
        self.assertEqual(set(self.configs_for('none').values()), {'zed_1811.yaml'})
        self.assertEqual(self.configs_for('front,rear'), {
            'zed_front': 'zed_1811_depth.yaml', 'zed_left': 'zed_1811.yaml',
            'zed_rear': 'zed_1811_depth.yaml', 'zed_right': 'zed_1811.yaml',
        })
        self.assertEqual(set(self.configs_for('all').values()), {'zed_1811_depth.yaml'})
        with self.assertRaisesRegex(RuntimeError, 'Unknown depth camera'):
            self.configs_for('front,unknown')

    def test_ground_tracking_is_only_enabled_for_selected_depth_cameras(self):
        context = self.context('front,rear')
        context.launch_configurations['ground_detection'] = 'true'
        with patch.object(zed_launch.FindPackageShare, 'find', return_value='/test/jetson_bringup'):
            actions = [action for action in zed_launch.setup(context)
                       if isinstance(action, IncludeLaunchDescription)]
        arguments = {dict(action.launch_arguments)['camera_name']: dict(action.launch_arguments)
                     for action in actions}
        for name in ('zed_front', 'zed_rear'):
            self.assertIn('pos_tracking.pos_tracking_enabled:=true', arguments[name]['param_overrides'])
            self.assertIn('debug.use_pub_timestamps:=false', arguments[name]['param_overrides'])
            self.assertEqual(arguments[name]['publish_tf'], 'false')
            self.assertEqual(arguments[name]['publish_map_tf'], 'false')
        for name in ('zed_left', 'zed_right'):
            self.assertNotIn('param_overrides', arguments[name])

    def test_depth_override_keeps_image_and_tracking_settings(self):
        directory = ROOT / 'ros2_ws/src/jetson_bringup/config'
        normal = yaml.safe_load((directory / 'zed_1811.yaml').read_text())['/**']['ros__parameters']
        depth = yaml.safe_load((directory / 'zed_1811_depth.yaml').read_text())['/**']['ros__parameters']
        self.assertEqual(normal['general'], depth['general'])
        self.assertEqual(normal['pos_tracking'], depth['pos_tracking'])
        self.assertEqual(normal['depth']['depth_mode'], 'NONE')
        self.assertEqual(depth['depth']['depth_mode'], 'NEURAL_LIGHT')
        self.assertTrue(depth['depth']['publish_point_cloud'])

    def test_bringup_passes_depth_selection_to_jetson(self):
        context = LaunchContext()
        context.launch_configurations.update({
            'cameras': 'true', 'registration_on_jetson': 'true',
            'jetson': 'nova@192.168.100.2', 'jetson_repo': '~/1811-fall-2026',
            'jetson_update': 'false', 'panorama': 'true',
            'depth_cameras': 'front,rear', 'jetson_key': '/tmp/test-key',
        })
        with patch.object(bringup, 'get_package_share_directory',
                          return_value=str(ROOT / 'ros2_ws/src/obc_bringup')):
            with patch.object(bringup, 'ExecuteProcess') as execute:
                with patch.object(bringup, 'OnProcessExit'):
                    bringup.jetson_cameras(context)
        command = execute.call_args.kwargs['cmd']
        self.assertEqual(command[2], 'nova@192.168.100.2')
        self.assertIn('scripts/vehicle.sh cameras', command[-1])
        self.assertIn('scripts/jetson_cameras.sh', command[-1])
        self.assertIn('--panorama', command[-1])
        self.assertIn('--depth-cameras=front,rear', command[-1])
        self.assertIn('--registration', command[-1])

        context.launch_configurations['hub'] = 'true'
        with patch.object(bringup, 'get_package_share_directory',
                          return_value=str(ROOT / 'ros2_ws/src/obc_bringup')):
            with patch.object(bringup, 'ExecuteProcess') as execute:
                with patch.object(bringup, 'OnProcessExit'):
                    bringup.jetson_cameras(context)
        self.assertIn('--hub-calibration', execute.call_args.kwargs['cmd'][-1])

        context.launch_configurations.update({'depth_cameras': 'auto'})
        with patch.object(bringup, 'get_package_share_directory',
                          return_value=str(ROOT / 'ros2_ws/src/obc_bringup')):
            with patch.object(bringup, 'ExecuteProcess') as execute:
                with patch.object(bringup, 'OnProcessExit'):
                    bringup.jetson_cameras(context)
        self.assertIn('--depth-cameras=all', execute.call_args.kwargs['cmd'][-1])

        context.launch_configurations.update({'registration_on_jetson': 'false'})
        with patch.object(bringup, 'get_package_share_directory',
                          return_value=str(ROOT / 'ros2_ws/src/obc_bringup')):
            with patch.object(bringup, 'ExecuteProcess') as execute:
                with patch.object(bringup, 'OnProcessExit'):
                    bringup.jetson_cameras(context)
        self.assertNotIn('--depth-cameras=', execute.call_args.kwargs['cmd'][-1])
        self.assertNotIn('--registration', execute.call_args.kwargs['cmd'][-1])

    def test_remote_camera_command_supports_new_and_older_jetson_checkouts(self):
        with tempfile.TemporaryDirectory(prefix='jetson launch ') as temp:
            repo = Path(temp) / 'repo with spaces; $(touch unexpected)'
            scripts = repo / 'scripts'
            scripts.mkdir(parents=True)
            context = LaunchContext()
            context.launch_configurations.update({
                'cameras': 'front,rear', 'registration_on_jetson': 'true',
                'jetson': 'nova@192.168.100.2', 'jetson_repo': str(repo),
                'jetson_update': 'false', 'panorama': 'true',
                'depth_cameras': 'front', 'jetson_key': '/tmp/test-key',
            })
            with patch.object(bringup, 'get_package_share_directory',
                              return_value=str(ROOT / 'ros2_ws/src/obc_bringup')):
                with patch.object(bringup, 'ExecuteProcess') as execute:
                    with patch.object(bringup, 'OnProcessExit'):
                        bringup.jetson_cameras(context)
            remote = execute.call_args.kwargs['cmd'][-1]
            for script_name, expected in (('vehicle.sh', ['cameras']), ('jetson_cameras.sh', [])):
                with self.subTest(script=script_name):
                    script = scripts / script_name
                    script.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
                    result = subprocess.run(['bash', '-c', remote], cwd=temp,
                                            text=True, capture_output=True, check=True)
                    self.assertEqual(result.stdout.splitlines(), expected + [
                        '--panorama', '--registration', '--cameras=front,rear', '--depth-cameras=front'])
                    self.assertFalse((Path(temp) / 'unexpected').exists())
                    script.unlink()

    def test_panorama_defaults_to_camera_availability_and_honors_overrides(self):
        declarations = {action.name: action for action in bringup.generate_launch_description().entities
                        if isinstance(action, DeclareLaunchArgument)}
        default_context = LaunchContext()
        declarations['panorama'].execute(default_context)
        self.assertEqual(default_context.launch_configurations['panorama'], 'auto')
        for cameras, panorama, expected in (('true', 'auto', True), ('false', 'auto', False),
                                             ('true', 'false', False), ('false', 'true', True)):
            with self.subTest(cameras=cameras, panorama=panorama):
                context = LaunchContext()
                context.launch_configurations.update({
                    'cameras': cameras, 'panorama': panorama, 'registration_on_jetson': 'true',
                    'jetson': 'nova@192.168.100.2', 'jetson_repo': '~/1811-fall-2026',
                    'jetson_update': 'false', 'depth_cameras': 'none', 'jetson_key': '/tmp/test-key',
                })
                with patch.object(bringup, 'get_package_share_directory',
                                  return_value=str(ROOT / 'ros2_ws/src/obc_bringup')):
                    with patch.object(bringup, 'ExecuteProcess') as execute:
                        with patch.object(bringup, 'OnProcessExit'):
                            bringup.jetson_cameras(context)
                self.assertEqual('--panorama' in execute.call_args.kwargs['cmd'][-1], expected)

    def test_registration_switch_preserves_old_alias_and_new_name_takes_precedence(self):
        declarations = {action.name: action for action in bringup.generate_launch_description().entities
                        if isinstance(action, DeclareLaunchArgument)}
        for given, expected in (({}, 'true'),
                                ({'corners_on_jetson': 'false'}, 'false'),
                                ({'registration_on_jetson': 'false'}, 'false'),
                                ({'corners_on_jetson': 'false',
                                  'registration_on_jetson': 'true'}, 'true')):
            with self.subTest(given=given):
                context = LaunchContext()
                context.launch_configurations.update(given)
                declarations['corners_on_jetson'].execute(context)
                declarations['registration_on_jetson'].execute(context)
                self.assertEqual(context.launch_configurations['registration_on_jetson'], expected)

    def test_bringup_passes_optional_lidar_timestamp_mode_to_ouster(self):
        includes = [action for action in bringup.generate_launch_description().entities
                    if isinstance(action, IncludeLaunchDescription)]
        ouster = [dict(action.launch_arguments) for action in includes
                  if 'timestamp_mode' in dict(action.launch_arguments)]
        self.assertEqual(len(ouster), 1)
        context = LaunchContext()
        context.launch_configurations['lidar_timestamp_mode'] = 'TIME_FROM_ROS_TIME'
        self.assertEqual(ouster[0]['timestamp_mode'].perform(context), 'TIME_FROM_ROS_TIME')

    def test_jetson_script_passes_depth_selection_to_camera_launch(self):
        with tempfile.TemporaryDirectory(prefix='zed_cloud_test_') as temp:
            root = Path(temp)
            (root / 'scripts').mkdir()
            (root / 'bin').mkdir()
            (root / 'zed_ws/install').mkdir(parents=True)
            (root / 'zed_ws/install/local_setup.bash').write_text('')
            (root / 'ros2_ws/install').mkdir(parents=True)
            (root / 'ros2_ws/install/local_setup.bash').write_text('')
            script = root / 'scripts/vehicle.sh'
            original = (ROOT / 'scripts/vehicle.sh').read_text()
            script.write_text(original.replace(
                'LOG_DIR="$HOME/.ros/jetson_cameras"',
                f'LOG_DIR={shlex.quote(str(root / "logs"))}'))
            pgrep = root / 'bin/pgrep'
            pgrep.write_text('#!/bin/sh\nexit 1\n')
            pgrep.chmod(0o755)
            stty = root / 'bin/stty'
            stty.write_text('#!/bin/sh\nif [ "$1" = -g ]; then echo test-tty; fi\nexit 0\n')
            stty.chmod(0o755)
            ros2 = root / 'bin/ros2'
            ros2.write_text('''#!/usr/bin/env python3
import json, os, pathlib, sys
with pathlib.Path(os.environ['NOVA_TEST_ZED_ARGS']).open('a') as events:
    events.write(json.dumps(sys.argv[1:]) + '\\n')
''')
            ros2.chmod(0o755)
            env = os.environ.copy()
            env.update({'PATH': f'{root / "bin"}:{env["PATH"]}',
                        'ZED_WS': str(root / 'zed_ws'),
                        'NOVA_TEST_ZED_ARGS': str(root / 'args')})
            result = subprocess.run(
                ['bash', str(script), 'cameras', '--panorama', '--cameras=front,rear', '--depth-cameras=front'],
                env=env, text=True, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            commands = [json.loads(line) for line in (root / 'args').read_text().splitlines()]
            self.assertEqual(len(commands), 2)
            self.assertIn([
                'launch', 'jetson_bringup', 'zed_cameras.launch.py',
                'cameras:=front,rear', 'depth_cameras:=front',
            ], commands)
            self.assertIn(['launch', 'camera_panorama', 'panorama.launch.py'], commands)

    def test_registration_uses_floor_overlay_and_handles_build_failure(self):
        for overlay_ok, filter_enabled in ((True, True), (False, True), (False, False)):
            with self.subTest(overlay_ok=overlay_ok, filter_enabled=filter_enabled), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                for directory in ('scripts', 'bin', 'zed_ws/install', 'ros2_ws/install', 'overlay'):
                    (root / directory).mkdir(parents=True, exist_ok=True)
                (root / 'zed_ws/install/local_setup.bash').write_text('')
                (root / 'ros2_ws/install/local_setup.bash').write_text('')
                (root / 'overlay/local_setup.bash').write_text('')
                original = (ROOT / 'scripts/vehicle.sh').read_text()
                (root / 'scripts/vehicle.sh').write_text(original.replace(
                    'LOG_DIR="$HOME/.ros/jetson_cameras"', f'LOG_DIR={shlex.quote(str(root / "logs"))}'))
                helper = root / 'scripts/prepare_zed_floor_overlay.py'
                helper.write_text(f'print({str(root / "overlay/local_setup.bash")!r})\n' if overlay_ok
                                  else 'raise SystemExit(1)\n')
                for source, installed in (
                    ('lidar_camera_registration.py', 'lib/lidar_perception/lidar_camera_registration.py'),
                    ('interactive_calibration.py', 'lib/lidar_perception/interactive_calibration.py'),
                    ('lidar_perception_transforms.py', 'lib/python/lidar_perception_transforms.py'),
                    ('launch/lidar_camera_registration.launch.py',
                     'share/lidar_perception/launch/lidar_camera_registration.launch.py'),
                    ('config/registration.yaml', 'share/lidar_perception/config/registration.yaml'),
                ):
                    a = root / 'ros2_ws/src/lidar_perception' / source
                    b = root / 'ros2_ws/install/lidar_perception' / installed
                    a.parent.mkdir(parents=True, exist_ok=True)
                    b.parent.mkdir(parents=True, exist_ok=True)
                    contents = ('lidar_camera_registration:\n  ros__parameters:\n'
                                f'    ground_filter_enabled: {str(filter_enabled).lower()}\n'
                                if source == 'config/registration.yaml' else 'same')
                    a.write_text(contents)
                    b.write_text(contents)
                for name, contents in (
                    ('pgrep', '#!/bin/sh\nexit 1\n'),
                    ('stty', '#!/bin/sh\nif [ "$1" = -g ]; then echo test-tty; fi\nexit 0\n'),
                    ('ros2', '''#!/usr/bin/env python3
import json, os, pathlib, sys, time
with pathlib.Path(os.environ['NOVA_TEST_ZED_ARGS']).open('a') as events:
    events.write(json.dumps(sys.argv[1:]) + '\\n')
time.sleep(0.05)
'''),
                ):
                    executable = root / 'bin' / name
                    executable.write_text(contents)
                    executable.chmod(0o755)
                env = os.environ.copy()
                env.update({'PATH': f'{root / "bin"}:{env["PATH"]}', 'ZED_WS': str(root / 'zed_ws'),
                            'PYTHONPATH': f'{root / "ros2_ws/install/lidar_perception/lib/python"}:{env.get("PYTHONPATH", "")}',
                            'NOVA_TEST_ZED_ARGS': str(root / 'args')})
                result = subprocess.run(
                    ['bash', str(root / 'scripts/vehicle.sh'), 'cameras', '--registration', '--depth-cameras=front'],
                    env=env, text=True, capture_output=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                commands = [json.loads(line) for line in (root / 'args').read_text().splitlines()]
                camera = next(cmd for cmd in commands if cmd[:3] == [
                    'launch', 'jetson_bringup', 'zed_cameras.launch.py'])
                self.assertEqual('ground_detection:=true' in camera, overlay_ok and filter_enabled)
                registration = ['launch', 'lidar_perception', 'lidar_camera_registration.launch.py']
                self.assertEqual(registration in commands, overlay_ok or not filter_enabled)
                if not overlay_ok and filter_enabled:
                    self.assertIn('registration will not start', result.stdout)

if __name__ == '__main__':
    unittest.main()
