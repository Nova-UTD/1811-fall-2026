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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    def context(self, depth_cameras):
        context = LaunchContext()
        context.launch_configurations.update({
            'cameras': 'front,left,rear,right', 'depth_cameras': depth_cameras,
            'camera_model': 'zedx',
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
        self.assertIn('jetson_cameras.sh', command[-1])
        self.assertIn('--panorama', command[-1])
        self.assertIn('--depth-cameras=front,rear', command[-1])
        self.assertIn('--registration', command[-1])

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
            script = root / 'scripts/jetson_cameras.sh'
            original = (ROOT / 'scripts/jetson_cameras.sh').read_text()
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
                ['bash', str(script), '--panorama', '--cameras=front,rear', '--depth-cameras=front'],
                env=env, text=True, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            commands = [json.loads(line) for line in (root / 'args').read_text().splitlines()]
            self.assertEqual(len(commands), 2)
            self.assertIn([
                'launch', 'jetson_bringup', 'zed_cameras.launch.py',
                'cameras:=front,rear', 'depth_cameras:=front',
            ], commands)
            self.assertIn(['launch', 'camera_panorama', 'panorama.launch.py'], commands)

if __name__ == '__main__':
    unittest.main()
