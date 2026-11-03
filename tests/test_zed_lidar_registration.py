"""Exercise global registration, ICP, and the four-camera ROS pipeline."""

import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
from threading import Thread
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import open3d as o3d
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import yaml


PATH = Path(__file__).resolve().parents[1] / 'ros2_ws/src/lidar_perception/lidar_camera_registration.py'
spec = importlib.util.spec_from_file_location('registration_node_test', PATH)
registration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(registration)
CONFIG = registration.DEFAULT_CONFIG
CONFIG_FILE = PATH.parent / 'config/registration.yaml'


def cloud(points):
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float).reshape(-1, 3))
    return result


def ros_cloud(points, frame, seconds=123):
    header = Header(frame_id=frame)
    header.stamp.sec = seconds
    return point_cloud2.create_cloud_xyz32(header, np.asarray(points).tolist())


def registration_scene():
    """A partially overlapping scene with planes and asymmetric curved objects."""
    rng = np.random.default_rng(81)

    def rectangle(corner, u, v, count):
        xy = rng.random((count, 2))
        return corner + xy[:, :1] * np.array(u) + xy[:, 1:] * np.array(v)

    def ellipsoid(center, radii, count):
        xyz = rng.normal(size=(count, 3))
        xyz /= np.linalg.norm(xyz, axis=1)[:, None]
        return xyz * radii + center

    scene = np.concatenate([
        rectangle([3, -3, -1], [4, 0, 0], [0, 6, 0], 2500),
        rectangle([7, -3, -1], [0, 6, 0], [0, 0, 3], 1600),
        ellipsoid([4, -0.8, -0.1], [0.7, 0.5, 0.9], 1300),
        ellipsoid([5.6, 1, 0.5], [0.4, 0.8, 0.6], 1200),
        rectangle([4.9, -2, -1], [0, 1, 0], [0, 0, 1.7], 900),
    ])
    # No camera pose is provided to registration; use a large 3D offset as the answer.
    pose = np.eye(4)
    pose[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz([0.6, -0.4, 1.35])
    pose[:3, 3] = [1.2, -0.7, 0.5]
    visible = scene[scene[:, 0] < 6.2]  # The camera sees only part of the LiDAR scene.
    camera = (visible - pose[:3, 3]) @ pose[:3, :3]
    camera_count, lidar_count = int(0.6 * len(camera)), int(0.7 * len(scene))
    camera = camera[rng.choice(len(camera), camera_count, replace=False)]
    lidar = scene[rng.choice(len(scene), lidar_count, replace=False)]
    # Independent sampling and small noise keep the two clouds from being identical.
    camera += rng.normal(0, 0.003, camera.shape)
    lidar += rng.normal(0, 0.003, lidar.shape)
    return camera, lidar, pose


class JetsonExecutionTest(unittest.TestCase):
    def test_registration_rejects_other_hosts_before_loading_open3d(self):
        # Simulate the Karbon's route in a fresh interpreter, before any processing imports.
        script = '''
import runpy
import socket
import sys

class FakeSocket:
    def __init__(self, *args, **kwargs): pass
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def connect(self, address): pass
    def getsockname(self): return ('192.168.100.1', 12345)

socket.socket = FakeSocket
try:
    runpy.run_path(sys.argv[1], run_name='__main__')
except SystemExit as exc:
    assert exc.code == 2
else:
    raise AssertionError('registration did not reject the Karbon')
assert 'open3d' not in sys.modules
assert 'numpy' not in sys.modules
'''
        result = subprocess.run([sys.executable, '-I', '-c', script, str(PATH)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('refusing to process sensor data', result.stderr)

    def test_node_requires_the_configured_jetson_route(self):
        with patch.object(registration, '_is_jetson_link', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'configured Jetson'):
                registration.LidarCameraRegistration()
        with patch.object(registration.socket, 'socket') as socket_factory:
            sock = socket_factory.return_value.__enter__.return_value
            sock.getsockname.return_value = ('192.168.100.2', 12345)
            self.assertTrue(registration._is_jetson_link())
            sock.getsockname.return_value = ('192.168.100.1', 12345)
            self.assertFalse(registration._is_jetson_link())
        with patch.object(registration.socket, 'socket', side_effect=OSError('no route')):
            self.assertFalse(registration._is_jetson_link())


class RegistrationMathTest(unittest.TestCase):
    def test_fpfh_ransac_and_icp_recover_large_pose_without_tf(self):
        camera, lidar, expected_pose = registration_scene()
        camera_cloud = cloud(camera).voxel_down_sample(CONFIG.icp_voxel_size_m)
        lidar_cloud = cloud(lidar).voxel_down_sample(CONFIG.icp_voxel_size_m)
        o3d.utility.random.seed(1)
        guess, reason = registration.estimate_initial_pose(
            camera_cloud, registration.prepare_fpfh(lidar_cloud, CONFIG), CONFIG)
        self.assertIsNone(reason)
        self.assertIsNotNone(guess)
        registered, pose, reason = registration.register_camera(camera_cloud, lidar_cloud, guess, CONFIG)
        self.assertIsNone(reason)
        self.assertIsNotNone(registered)
        np.testing.assert_allclose(pose, expected_pose, atol=0.03)
        # The output points really are transformed, not just labeled with the LiDAR frame.
        error = np.asarray(registered.compute_point_cloud_distance(lidar_cloud))
        self.assertLess(np.percentile(error, 90), 0.20)

    def test_rejects_too_few_points_and_weak_ransac_matches(self):
        camera, lidar, _ = registration_scene()
        lidar_features = registration.prepare_fpfh(cloud(lidar), CONFIG)
        pose, reason = registration.estimate_initial_pose(cloud(camera[:10]), lidar_features, CONFIG)
        self.assertIsNone(pose)
        self.assertIn('too few points', reason)

        weak = SimpleNamespace(transformation=np.eye(4), fitness=0.01,
                               correspondence_set=[0] * 10)
        with patch.object(o3d.pipelines.registration,
                          'registration_ransac_based_on_feature_matching', return_value=weak):
            pose, reason = registration.estimate_initial_pose(cloud(camera), lidar_features, CONFIG)
        self.assertIsNone(pose)
        self.assertIn('RANSAC overlap', reason)

    def test_icp_rejects_weak_matches_and_large_corrections(self):
        points = np.random.default_rng(9).uniform(-2, 2, (500, 3))
        registered, pose, reason = registration.register_camera(
            cloud(points + 100), cloud(points), np.eye(4), CONFIG)
        self.assertIsNone(registered)
        self.assertIsNone(pose)
        self.assertIn('overlap', reason)

        bad_pose = np.eye(4)
        bad_pose[0, 3] = 0.8
        bad_result = SimpleNamespace(transformation=bad_pose, fitness=0.8,
                                     inlier_rmse=0.02, correspondence_set=[0] * 100)
        with patch.object(o3d.pipelines.registration, 'registration_icp', return_value=bad_result):
            registered, pose, reason = registration.register_camera(cloud(points), cloud(points), np.eye(4), CONFIG)
        self.assertIsNone(registered)
        self.assertIsNone(pose)
        self.assertIn('correction too large', reason)

    def test_camera_decode_ignores_color_nans_and_invalid_xyz(self):
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in (('x', 0), ('y', 4), ('z', 8), ('rgb', 12))]
        msg = point_cloud2.create_cloud(Header(frame_id='camera'), fields, [
            [1.0, 2.0, 3.0, float('nan')], [4.0, 5.0, 6.0, 0.0],
            [float('nan'), 1.0, 2.0, 0.0], [float('inf'), 1.0, 2.0, 0.0],
        ])
        actual = np.asarray(registration.point_cloud_from_message(msg).points)
        self.assertEqual(len(actual), 2)
        self.assertTrue(all(any(np.allclose(p, expected) for p in actual)
                            for expected in ([1, 2, 3], [4, 5, 6])))


class RegistrationConfigTest(unittest.TestCase):
    def load_node(self, config_file):
        rclpy.init(args=['--ros-args', '--params-file', str(config_file)])
        self.addCleanup(rclpy.try_shutdown)
        with patch.object(registration, '_is_jetson_link', return_value=True):
            node = registration.LidarCameraRegistration()
        self.addCleanup(node.destroy_node)
        return node

    def edited_config(self, changes):
        config = yaml.safe_load(CONFIG_FILE.read_text())
        config['lidar_camera_registration']['ros__parameters'].update(changes)
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        path = Path(temp.name) / 'registration.yaml'
        path.write_text(yaml.safe_dump(config))
        return path

    def test_default_yaml_loads_existing_values_and_parameters_are_read_only(self):
        node = self.load_node(CONFIG_FILE)
        parameters = yaml.safe_load(CONFIG_FILE.read_text())['lidar_camera_registration']['ros__parameters']
        for name, expected in parameters.items():
            if isinstance(expected, list):
                expected = tuple(expected)
            self.assertEqual(getattr(node.config, name), expected, name)
        self.assertTrue(node.describe_parameter('icp_voxel_size_m').read_only)
        self.assertEqual(node.describe_parameter('icp_voxel_size_m').type, Parameter.Type.DOUBLE.value)
        self.assertEqual(node.describe_parameter('min_points').type, Parameter.Type.INTEGER.value)
        self.assertEqual(node.describe_parameter('front_rgb').type, Parameter.Type.INTEGER_ARRAY.value)
        result = node.set_parameters([Parameter('icp_voxel_size_m', value=0.3)])[0]
        self.assertFalse(result.successful)  # Settings are read once; restart to change them.

    def test_yaml_overrides_change_registration_and_output_color(self):
        path = self.edited_config({
            'fpfh_voxel_size_m': 0.4,
            'ransac_max_distance_multiplier': 1.5,
            'icp_max_translation_m': 1.0,
            'front_rgb': [1, 2, 3],
        })
        node = self.load_node(path)
        config = node.config
        self.assertAlmostEqual(config.fpfh_normal_radius_m, 0.8)
        self.assertAlmostEqual(config.fpfh_feature_radius_m, 2.0)
        self.assertAlmostEqual(config.ransac_max_distance_m, 0.6)

        # The configured RGB reaches the ROS message used by the registered RViz display.
        message = registration.pack_registered_cloud([('front', cloud([[1, 2, 3]]))], Header(), config)
        self.assertEqual(point_cloud2.read_points(message, field_names=['rgb'])['rgb'][0], 0x010203)

        # A correction rejected by the default 0.5 m limit passes with the configured 1.0 m limit.
        points = np.random.default_rng(9).uniform(-2, 2, (100, 3))
        pose = np.eye(4)
        pose[0, 3] = 0.8
        result = SimpleNamespace(transformation=pose, fitness=0.8, inlier_rmse=0.02,
                                 correspondence_set=[0] * 100)
        with patch.object(o3d.pipelines.registration, 'registration_icp', return_value=result):
            registered, _, reason = registration.register_camera(cloud(points), cloud(points), np.eye(4), config)
        self.assertIsNone(reason)
        self.assertIsNotNone(registered)

    def test_invalid_config_fails_at_startup(self):
        path = self.edited_config({'fpfh_voxel_size_m': 0.0})
        rclpy.init(args=['--ros-args', '--params-file', str(path)])
        self.addCleanup(rclpy.try_shutdown)
        node = rclpy.create_node('lidar_camera_registration')
        self.addCleanup(node.destroy_node)
        with self.assertRaisesRegex(ValueError, 'fpfh_voxel_size_m'):
            registration.load_registration_config(node)

    def test_invalid_color_fails_at_startup(self):
        path = self.edited_config({'front_rgb': [0, 256, 0]})
        rclpy.init(args=['--ros-args', '--params-file', str(path)])
        self.addCleanup(rclpy.try_shutdown)
        node = rclpy.create_node('lidar_camera_registration')
        self.addCleanup(node.destroy_node)
        with self.assertRaisesRegex(ValueError, 'front_rgb'):
            registration.load_registration_config(node)


class RegistrationNodeTest(unittest.TestCase):
    def test_lidar_wall_and_ground_points_reach_registered_output(self):
        # Distinct voxels represent ground, wall, and other scene points.
        points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0],
                           [2, 0, 1], [2, 1, 1], [2, 0, 2]])
        node = Mock()
        node.config = CONFIG
        node.snapshot_camera_clouds.return_value = {}
        node.publish_registered_cloud.side_effect = (
            lambda lidar, header, cameras:
            registration.LidarCameraRegistration.publish_registered_cloud(node, lidar, header, cameras))
        registration.LidarCameraRegistration.pcd_callback(node, ros_cloud(points, 'os_lidar'))
        result = node.registered_pub.publish.call_args.args[0]
        xyz = point_cloud2.read_points(result, field_names=['x', 'y', 'z'])
        actual = np.column_stack((xyz['x'], xyz['y'], xyz['z']))
        self.assertEqual({tuple(point) for point in actual}, {tuple(point) for point in points})

    def test_empty_lidar_still_publishes_an_empty_frame(self):
        node = Mock()
        node.config = CONFIG
        node.snapshot_camera_clouds.return_value = {}
        node.publish_registered_cloud.side_effect = (
            lambda lidar, header, cameras:
            registration.LidarCameraRegistration.publish_registered_cloud(node, lidar, header, cameras))
        registration.LidarCameraRegistration.pcd_callback(node, ros_cloud([], 'os_lidar'))
        result = node.registered_pub.publish.call_args.args[0]
        self.assertEqual(result.width, 0)
        self.assertEqual(result.header.frame_id, 'os_lidar')

    def test_cached_pose_is_reused_and_reset_on_frame_change_or_failures(self):
        points = np.random.default_rng(3).uniform(-2, 2, (100, 3))
        node = Mock()
        node.config = CONFIG
        node.camera_poses = {}
        node.camera_failures = {}
        header = Header(frame_id='os_lidar')
        header.stamp.sec = 123
        cameras = {'front': ros_cloud(points, 'zed_front_optical_frame')}
        estimate = (np.eye(4), None)
        with patch.object(registration, 'estimate_initial_pose', return_value=estimate) as global_match, \
             patch.object(registration, 'register_camera', return_value=(cloud(points), np.eye(4), None)) as icp:
            for _ in range(2):
                registration.LidarCameraRegistration.publish_registered_cloud(node, cloud(points), header, cameras)
            self.assertEqual(global_match.call_count, 1)  # The second scan reuses the anchor.
            self.assertEqual(icp.call_count, 2)
            self.assertIn('front', node.camera_poses)

            cameras['front'] = ros_cloud(points, 'new_front_frame')
            registration.LidarCameraRegistration.publish_registered_cloud(node, cloud(points), header, cameras)
            self.assertEqual(global_match.call_count, 2)
            self.assertEqual(node.camera_poses['front'][1], 'new_front_frame')

            icp.return_value = (None, None, 'insufficient ICP overlap')
            for _ in range(CONFIG.max_icp_failures):
                registration.LidarCameraRegistration.publish_registered_cloud(node, cloud(points), header, cameras)
            self.assertNotIn('front', node.camera_poses)
            self.assertIn('retrying FPFH/RANSAC',
                          node.get_logger.return_value.warning.call_args.args[0])
            icp.return_value = (cloud(points), np.eye(4), None)
            registration.LidarCameraRegistration.publish_registered_cloud(node, cloud(points), header, cameras)
            self.assertEqual(global_match.call_count, 3)
            self.assertIn('front', node.camera_poses)

    def test_four_cameras_are_registered_and_stale_camera_is_skipped(self):
        rclpy.init(args=[])
        node = sender = executor = thread = None
        try:
            with patch.object(registration, '_is_jetson_link', return_value=True):
                node = registration.LidarCameraRegistration()
            sender = rclpy.create_node('registration_test_sender')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            camera_publishers = {
                name: sender.create_publisher(
                    PointCloud2, f'/zed_{name}/zed_node/point_cloud/cloud_registered', qos)
                for name in registration.CAMERAS
            }
            lidar_publisher = sender.create_publisher(PointCloud2, '/ouster/points', qos)
            received = []
            output_sub = sender.create_subscription(
                PointCloud2, registration.REGISTERED_CLOUD_TOPIC, received.append, qos)
            self.assertIsNotNone(output_sub)
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            executor.add_node(sender)
            thread = Thread(target=executor.spin, daemon=True)
            thread.start()

            camera_points, lidar_points, front_pose = registration_scene()
            visible_world = camera_points @ front_pose[:3, :3].T + front_pose[:3, 3]
            poses = {}
            camera_samples = {}
            for index, name in enumerate(registration.CAMERAS):
                pose = np.eye(4)
                pose[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz(
                    [0.4, -0.3, 1.35 + index * np.pi / 2])
                pose[:3, 3] = [1.2 - 0.2 * index, -0.7 + 0.3 * index, 0.5]
                poses[name] = pose
                camera_samples[name] = (visible_world - pose[:3, 3]) @ pose[:3, :3]
            camera_messages = {name: ros_cloud(camera_samples[name], f'zed_{name}_optical_frame')
                               for name in registration.CAMERAS}
            lidar_msg = ros_cloud(lidar_points, 'os_lidar')
            self.assertFalse(hasattr(node, 'tf_buffer'))  # No camera TF is needed for registration.
            # Exercise real ROS delivery, FPFH, RANSAC, and ICP for all four cameras.
            with patch.object(registration, 'estimate_initial_pose', wraps=registration.estimate_initial_pose) as global_match:
                deadline = time.monotonic() + 20
                while len(node.snapshot_camera_clouds()) < 4 and time.monotonic() < deadline:
                    for name, publisher in camera_publishers.items():
                        publisher.publish(camera_messages[name])
                    time.sleep(0.05)
                self.assertEqual(set(node.snapshot_camera_clouds()), set(registration.CAMERAS))
                while not received and time.monotonic() < deadline:
                    lidar_publisher.publish(lidar_msg)
                    time.sleep(0.05)
                self.assertTrue(received, 'registered cloud was not published')
                result = received[-1]
                self.assertEqual(result.header.frame_id, 'os_lidar')
                self.assertEqual(result.header.stamp.sec, 123)
                colors = set(point_cloud2.read_points(result, field_names=['rgb'])['rgb'])
                expected = {(r << 16) | (g << 8) | b
                            for r, g, b in (getattr(CONFIG, f'{name}_rgb')
                                            for name in ('lidar', *registration.CAMERAS))}
                self.assertEqual(colors, expected)
                self.assertEqual(global_match.call_count, 4)
                self.assertEqual(set(node.camera_poses), set(registration.CAMERAS))
                # All colored camera points must be close to the LiDAR geometry.
                xyzrgb = point_cloud2.read_points(result, field_names=['x', 'y', 'z', 'rgb'])
                reference = cloud(lidar_points)
                for name in registration.CAMERAS:
                    np.testing.assert_allclose(node.camera_poses[name][2], poses[name], atol=0.04)
                    r, g, b = getattr(CONFIG, f'{name}_rgb')
                    selected = xyzrgb[xyzrgb['rgb'] == ((r << 16) | (g << 8) | b)]
                    xyz = np.column_stack((selected['x'], selected['y'], selected['z']))
                    error = np.asarray(cloud(xyz).compute_point_cloud_distance(reference))
                    self.assertLess(np.percentile(error, 90), 0.20, name)

                stale = ros_cloud(camera_samples['rear'], 'zed_rear_optical_frame', seconds=122)
                camera_publishers['rear'].publish(stale)
                while (node.snapshot_camera_clouds()['rear'].header.stamp.sec != 122
                       and time.monotonic() < deadline):
                    time.sleep(0.01)
                self.assertEqual(node.snapshot_camera_clouds()['rear'].header.stamp.sec, 122)
                received.clear()
                deadline = time.monotonic() + 20
                red, green, blue = CONFIG.rear_rgb
                rear_color = (red << 16) | (green << 8) | blue
                colors = set()
                while time.monotonic() < deadline:
                    lidar_publisher.publish(lidar_msg)
                    time.sleep(0.05)
                    if received:
                        colors = set(point_cloud2.read_points(received[-1], field_names=['rgb'])['rgb'])
                        if rear_color not in colors:
                            break
                self.assertNotIn(rear_color, colors)
                self.assertEqual(len(colors), 4)  # LiDAR plus the other three cameras.
                self.assertEqual(global_match.call_count, 4)  # Cached poses avoid repeated RANSAC.
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=5)
            if thread is not None:
                thread.join(timeout=5)
            if node is not None:
                node.destroy_node()
            if sender is not None:
                sender.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
