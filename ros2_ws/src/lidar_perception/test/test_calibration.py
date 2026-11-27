# Check 1811 wiring and saved calibration. Shared solver tests run in the submodule.

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from lidar_camera_calibration.mounts import calibrated_camera_mounts, update_camera_joints
from lidar_camera_calibration.node import LidarCameraRegistration
from lidar_camera_calibration.transforms import transform_matrix
import numpy as np
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import SingleThreadedExecutor
from vehicle_calibration_fixture import PROFILE, SOURCE, vehicle_config, vehicle_description
import yaml

sys.path.insert(0, str(SOURCE))
from lidar_camera_node import VehicleLidarCameraRegistration  # noqa: E402, I100
from lidar_perception_transforms import transform_matrix as vehicle_transform_matrix  # noqa: E402


class VehicleCalibrationTest(unittest.TestCase):

    def test_profile_preserves_vehicle_wiring_and_tuning(self):
        config = vehicle_config()
        self.assertEqual(config.lidar_topic, '/ouster/points')
        self.assertEqual(config.lidar_imu_topic, '/ouster/imu')
        self.assertEqual(config.reference_frame, 'base_link')
        self.assertEqual(config.output_topic_prefix, '/lidar_perception')
        self.assertEqual(
            config.robot_state_publisher_service, '/robot_state_publisher/set_parameters',
        )
        self.assertTrue(config.apply_urdf)
        self.assertTrue(config.restore_calibration_on_startup)
        self.assertTrue(config.lidar_crop_enabled)
        self.assertEqual(config.min_points, 400)
        self.assertEqual(config.icp_min_fitness, 0.42)
        self.assertEqual(tuple(config.cameras), ('front', 'left', 'rear', 'right'))
        for name, camera in config.cameras.items():
            registered_topic = f'/zed_{name}/zed_node/point_cloud/cloud_registered'
            self.assertEqual(camera.cloud_topic, registered_topic)
            self.assertEqual(camera.floor_plane_topic, f'/zed_{name}/zed_node/floor_plane')
            self.assertEqual(camera.parent_frame, 'base_link')
            self.assertTrue(camera.crop_enabled)
            self.assertEqual(camera.box_min_m, (0.5, -2.0, -1.5))
            self.assertEqual(camera.box_max_m, (3.5, 2.0, 1.5))
        self.assertIs(VehicleLidarCameraRegistration.calibrate, LidarCameraRegistration.calibrate)
        self.assertIs(vehicle_transform_matrix, transform_matrix)

    def test_source_has_only_floating_camera_mounts_and_measured_lidar(self):
        config = vehicle_config()
        description = SOURCE.parent / 'vehicle_1811_description' / 'urdf'
        namespace = {'xacro': 'http://www.ros.org/wiki/xacro'}
        root = ET.parse(description / 'vehicle_1811.urdf.xacro').getroot()
        self.assertEqual(root.get('name'), config.robot_name)
        camera_mounts = root.findall('xacro:zed_x_camera', namespace)
        self.assertEqual({mount.get('name') for mount in camera_mounts}, set(config.cameras))
        for mount in camera_mounts:
            name = mount.get('name')
            camera = config.cameras[name]
            self.assertEqual(camera.mount_joint, f'zed_{name}_mount')
            self.assertEqual(camera.body_frame, f'zed_{name}_camera_link')
            self.assertEqual(camera.parent_frame, mount.get('parent'))
            for coordinate in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                self.assertEqual(mount.get(coordinate), '0')
        macro = ET.parse(description / 'vehicle_1811.macros.xacro').getroot()
        camera = macro.find("xacro:macro[@name='zed_x_camera']", namespace)
        self.assertEqual(camera.find("joint[@name='zed_${name}_mount']").get('type'), 'floating')
        lidar = root.find('xacro:ouster_os1', namespace)
        lidar_rpy = tuple(lidar.get(angle) for angle in ('roll', 'pitch', 'yaw'))
        self.assertEqual(lidar_rpy, ('0', '-0.0600', '0'))

    def test_launcher_rejects_a_non_jetson_before_importing_ros_or_the_solver(self):
        probe = """
import runpy
import socket
import sys
from unittest.mock import MagicMock

link = MagicMock()
link.__enter__.return_value = link
link.getsockname.return_value = ("192.168.100.1", 1234)
socket.socket = MagicMock(return_value=link)
try:
    runpy.run_path(sys.argv[1], run_name="__main__")
except SystemExit as exc:
    assert "rclpy" not in sys.modules and "lidar_camera_calibration" not in sys.modules
    raise exc
"""
        launcher = subprocess.run(
            [sys.executable, '-c', probe, str(SOURCE / 'lidar_camera_registration.py')],
            text=True, capture_output=True, timeout=5,
        )
        self.assertEqual(launcher.returncode, 2, launcher.stdout + launcher.stderr)
        self.assertIn('requires the Jetson cable address', launcher.stderr)

    def test_real_vehicle_publisher_restores_legacy_mount_and_hub_status(self):
        try:
            publisher_prefix = Path(get_package_prefix('robot_state_publisher'))
        except PackageNotFoundError:
            self.skipTest('requires robot_state_publisher')
        executable = publisher_prefix / 'lib/robot_state_publisher/robot_state_publisher'
        config = vehicle_config()
        description = vehicle_description(config)
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
            'ROS_DOMAIN_ID': '112', 'ROS_LOCALHOST_ONLY': '1',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'ROS_STATIC_PEERS': '',
            'ROS_LOG_DIR': directory,
        }):
            os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
            output_urdf = Path(directory) / 'vehicle_1811_calibrated.urdf'
            mount = np.eye(4)
            mount[:3, 3] = [1.1, -0.3, 0.8]
            legacy_urdf = update_camera_joints(description, {'front': mount}, config)
            output_urdf.write_text(legacy_urdf)
            publisher_config = Path(directory) / 'publisher.yaml'
            publisher_config.write_text(yaml.safe_dump({'robot_state_publisher': {
                'ros__parameters': {'robot_description': description},
            }}))
            process = subprocess.Popen(
                [str(executable), '--ros-args', '--params-file', str(publisher_config)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            node = executor = None
            try:
                rclpy.init(args=[
                    '--ros-args', '--params-file', str(PROFILE), '--', '--ros-args',
                    '-p', 'interactive_calibration:=true', '-p', f'output_urdf:={output_urdf}',
                ])
                node = VehicleLidarCameraRegistration()
                executor = SingleThreadedExecutor()
                executor.add_node(node)
                deadline = time.monotonic() + 12.0
                while node.hub.accepted != {'front'} and time.monotonic() < deadline:
                    self.assertIsNone(process.poll(), 'robot_state_publisher exited')
                    executor.spin_once(timeout_sec=0.1)
                self.assertEqual(node.hub.accepted, {'front'})
                self.assertFalse(node.hub.active)
                self.assertFalse(node.camera_clouds)
                self.assertIsNone(node.latest_lidar)
                client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
                self.assertTrue(client.wait_for_service(timeout_sec=3.0))
                request = client.call_async(GetParameters.Request(names=['robot_description']))
                executor.spin_until_future_complete(request, timeout_sec=3.0)
                self.assertTrue(request.done())
                live_description = request.result().values[0].string_value
                restored = calibrated_camera_mounts(live_description, config)
                self.assertEqual(set(restored), {'front'})
                np.testing.assert_allclose(restored['front'], mount)
                self.assertEqual(output_urdf.read_text(), legacy_urdf)
            finally:
                if executor is not None:
                    executor.shutdown()
                if node is not None:
                    node.destroy_node()
                rclpy.try_shutdown()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
