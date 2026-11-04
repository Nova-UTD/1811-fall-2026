"""Regression checks for cropped LiDAR/ZED calibration and URDF updates."""
import importlib.util
import math
import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock, patch

# Open3D's small synthetic scenes run faster and reproducibly with one worker.
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np
import yaml
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


SOURCE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("calibration", SOURCE / "lidar_camera_registration.py")
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)

URDF = """<robot name="vehicle_1811">
  <link name="base_link"/><link name="os_sensor"/>
  <joint name="lidar_mount" type="fixed"><parent link="base_link"/><child link="os_sensor"/>
    <origin xyz="0.015 -0.105 1.26" rpy="0 0.0698 0.215"/></joint>
  <link name="zed_front_camera_link"/>
  <joint name="zed_front_mount" type="floating"><parent link="base_link"/>
    <child link="zed_front_camera_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
  <link name="zed_left_camera_link"/>
  <joint name="zed_left_mount" type="floating"><parent link="base_link"/>
    <child link="zed_left_camera_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
</robot>"""


def rotation_from_rpy(roll, pitch, yaw):
    """Independent URDF Rz(yaw) Ry(pitch) Rx(roll) reconstruction."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def message(frame="os_lidar", stamp=100):
    result = PointCloud2()
    result.header.frame_id = frame
    result.header.stamp.sec = stamp
    return result


def identity_tf_buffer():
    transform = Transform()
    transform.rotation.w = 1.0
    buffer = Mock()
    buffer.lookup_transform.return_value = SimpleNamespace(transform=transform)
    return buffer


class CalibrationTest(unittest.TestCase):
    def test_boxes_crop_in_reference_frames_and_keep_sensor_coordinates(self):
        config = calibration.DEFAULT_CONFIG._replace(wall_filter_enabled=False)
        lidar_transform = Transform()
        lidar_transform.translation.x, lidar_transform.translation.z = 0.2, 1.2
        lidar_transform.rotation.z = math.sin(math.pi / 4)
        lidar_transform.rotation.w = math.cos(math.pi / 4)
        camera_transform = Transform()  # Optical +Z forward, +X right, +Y down.
        camera_transform.translation.x, camera_transform.translation.y = 0.02, 0.06
        camera_transform.rotation.x, camera_transform.rotation.y = -0.5, 0.5
        camera_transform.rotation.z, camera_transform.rotation.w = -0.5, 0.5
        for name in ("lidar", *calibration.CAMERAS):
            with self.subTest(sensor=name):
                is_lidar = name == "lidar"
                transform = lidar_transform if is_lidar else camera_transform
                reference = "base_link" if is_lidar else f"zed_{name}_camera_link"
                frame = "os_lidar" if is_lidar else f"zed_{name}_left_camera_frame_optical"
                ref_points = (np.array([[0, 0, 1], [4.9, 4.9, 3.4], [5.1, 0, 1],
                                        [0, -5.1, 1], [0, 0, 3.6], [0, 0, -0.6]])
                              if is_lidar else
                              np.array([[0.6, 0, 0], [3.4, 1.9, 1.4], [0.4, 0, 0],
                                        [3.6, 0, 0], [1, 2.1, 0], [1, 0, -1.6], [-1, 0, 0]]))
                # Independent inverse of the known 90-degree LiDAR rotation or
                # the optical axes: camera body XYZ = [optical Z, -X, -Y].
                if is_lidar:
                    points = np.column_stack((ref_points[:, 1], 0.2 - ref_points[:, 0],
                                              ref_points[:, 2] - 1.2))
                else:
                    points = np.column_stack((0.06 - ref_points[:, 1], -ref_points[:, 2],
                                              ref_points[:, 0] - 0.02))
                header = Header(frame_id=frame)
                header.stamp.sec = 123
                msg = point_cloud2.create_cloud_xyz32(header, points.tolist())
                original = bytes(msg.data)
                buffer = Mock()
                buffer.lookup_transform.return_value = SimpleNamespace(transform=transform)
                node = SimpleNamespace(config=config, tf_buffer=buffer, get_logger=Mock(),
                                       input_pubs={name: Mock()})
                matching, display = calibration.LidarCameraRegistration.prepare_calibration_input(node, name, msg)
                output = node.input_pubs[name].publish.call_args.args[0]
                self.assertEqual(output.header, header)
                self.assertEqual(bytes(msg.data), original)
                self.assertEqual(len(matching.points), 2)
                np.testing.assert_allclose(np.asarray(display.points), points[:2], atol=1e-6)
                buffer.lookup_transform.assert_called_once()
                self.assertEqual(buffer.lookup_transform.call_args.args[:2], (reference, frame))

    def test_box_boundaries_empty_cloud_and_missing_tf(self):
        config = calibration.DEFAULT_CONFIG
        cloud = calibration.o3d.geometry.PointCloud()
        cloud.points = calibration.o3d.utility.Vector3dVector(
            np.array([[0.5, -2, -1.5], [3.5, 2, 1.5], [0.49, 0, 0], [3.51, 0, 0]]))
        cropped = calibration.crop_calibration_cloud(cloud, Mock(), "camera", "camera",
                                                      config.camera_box_min_m, config.camera_box_max_m)
        np.testing.assert_array_equal(np.asarray(cropped.points), np.asarray(cloud.points)[:2])
        empty = calibration.crop_calibration_cloud(calibration.o3d.geometry.PointCloud(), Mock(),
                                                    "camera", "camera", config.camera_box_min_m,
                                                    config.camera_box_max_m)
        self.assertEqual(len(empty.points), 0)
        buffer = Mock()
        buffer.lookup_transform.side_effect = calibration.TransformException("missing TF")
        with self.assertRaises(calibration.TransformException):
            calibration.crop_calibration_cloud(cloud, buffer, "cloud", "camera",
                                               config.camera_box_min_m, config.camera_box_max_m)

    def test_box_settings_are_loaded_and_invalid_boxes_are_rejected(self):
        parameters = yaml.safe_load((SOURCE / "config/registration.yaml").read_text())[
            "lidar_camera_registration"]["ros__parameters"]

        def load(changes):
            node = Mock()

            def declare(name, default, descriptor):
                self.assertTrue(descriptor.read_only)
                return SimpleNamespace(value=changes.get(name, parameters.get(name, default)))

            node.declare_parameter.side_effect = declare
            return calibration.load_registration_config(node)

        config = load({})
        for name in ("lidar_box_min_m", "lidar_box_max_m", "camera_box_min_m", "camera_box_max_m"):
            self.assertEqual(getattr(config, name), tuple(parameters[name]))
        for name, value in (("lidar_box_min_m", [0.0, 1.0]),
                            ("lidar_box_max_m", [float("nan"), 5.0, 3.0]),
                            ("camera_box_min_m", [-0.1, -2.0, -1.5]),
                            ("camera_box_max_m", [0.5, 2.0, 1.5])):
            with self.subTest(parameter=name), self.assertRaisesRegex(ValueError, name):
                load({name: value})

    def test_missing_camera_tf_skips_only_that_camera(self):
        points = np.column_stack((np.linspace(0.6, 3.4, 40), np.zeros(40), np.zeros(40))).tolist()

        def sample(frame):
            header = Header(frame_id=frame)
            header.stamp.sec = 100
            return point_cloud2.create_cloud_xyz32(header, points), 100.0

        transform = Transform()
        transform.rotation.w = 1.0
        buffer = Mock()

        def lookup(target, source, time):
            if target == "zed_front_camera_link":
                raise calibration.TransformException("missing front TF")
            return SimpleNamespace(transform=transform)

        buffer.lookup_transform.side_effect = lookup
        node = SimpleNamespace(
            config=calibration.DEFAULT_CONFIG._replace(wall_filter_enabled=False, min_points=3),
            tf_buffer=buffer, get_logger=Mock(), data_lock=Lock(), state_lock=Lock(),
            next_calibration=0.0, latest_lidar=sample("os_lidar"),
            camera_clouds={name: sample(f"zed_{name}_left_camera_frame") for name in ("front", "left")},
            robot_description=URDF, camera_poses={}, camera_mounts={}, retry_global=set(),
            update_in_flight=False, pending_update=None, update_started=0.0,
            input_pubs={name: Mock() for name in ("lidar", *calibration.CAMERAS)},
            publish_registered_inputs=Mock(), calibrate=Mock(), description_client=Mock(),
        )
        node.description_client.service_is_ready.return_value = True
        node.prepare_calibration_input = lambda *args: (
            calibration.LidarCameraRegistration.prepare_calibration_input(node, *args))
        with patch.object(calibration.time, "monotonic", return_value=100.0):
            calibration.LidarCameraRegistration.calibration_tick(node)
        self.assertEqual(node.input_pubs["front"].publish.call_args.args[0].width, 0)
        self.assertGreater(node.input_pubs["left"].publish.call_args.args[0].width, 0)
        self.assertEqual(set(node.calibrate.call_args.args[2]), {"left"})

    def test_cropped_clouds_recover_a_pose_with_fpfh_ransac_and_icp(self):
        rng = np.random.default_rng(81)

        def ellipsoid(center, radius, count):
            points = rng.normal(size=(count, 3))
            points /= np.linalg.norm(points, axis=1)[:, None]
            return points * radius + center

        floor = np.column_stack((rng.uniform(0.6, 3.4, 1800), rng.uniform(-1.8, 1.8, 1800),
                                 np.full(1800, -1.0)))
        wall = np.column_stack((np.full(1200, 3.3), rng.uniform(-1.8, 1.8, 1200),
                                rng.uniform(-1.0, 1.2, 1200)))
        scene = np.vstack((floor, wall, ellipsoid([1.5, -0.7, -0.1], [0.35, 0.4, 0.75], 1500),
                           ellipsoid([2.6, 0.7, 0.2], [0.5, 0.35, 0.5], 1300)))
        expected = np.eye(4)
        expected[:3, :3] = rotation_from_rpy(0.02, -0.12, 0.4)
        expected[:3, 3] = [0.2, -0.1, 1.1]
        camera = scene[rng.choice(len(scene), int(0.7 * len(scene)), replace=False)]
        lidar = scene[rng.choice(len(scene), int(0.8 * len(scene)), replace=False)] @ expected[:3, :3].T + expected[:3, 3]
        camera += rng.normal(0, 0.002, camera.shape)
        lidar += rng.normal(0, 0.002, lidar.shape)
        camera = np.vstack((camera, [[-2, 0, 0], [5, 0, 0]]))
        lidar = np.vstack((lidar, [[15, 0, 0], [0, 15, 0]]))
        parameters = yaml.safe_load((SOURCE / "config/registration.yaml").read_text())[
            "lidar_camera_registration"]["ros__parameters"]
        config = calibration.DEFAULT_CONFIG._replace(**{
            name: tuple(value) if isinstance(value, list) else value for name, value in parameters.items()
        })
        node = SimpleNamespace(config=config, tf_buffer=Mock(), get_logger=Mock(),
                               input_pubs={"lidar": Mock(), "front": Mock()})

        def prepare(name, frame, points):
            msg = point_cloud2.create_cloud_xyz32(Header(frame_id=frame), points.tolist())
            return calibration.LidarCameraRegistration.prepare_calibration_input(node, name, msg)[0]

        lidar_cloud = prepare("lidar", "base_link", lidar)
        camera_cloud = prepare("front", "zed_front_camera_link", camera)
        calibration.o3d.utility.random.seed(1)
        guess, reason = calibration.estimate_initial_pose(
            camera_cloud, calibration.prepare_fpfh(lidar_cloud, config), config)
        self.assertIsNone(reason)
        registered, pose, reason = calibration.register_camera(camera_cloud, lidar_cloud, guess, config)
        self.assertIsNone(reason)
        self.assertIsNotNone(registered)
        np.testing.assert_allclose(pose, expected, atol=0.04)

    def test_node_rejects_non_jetson_before_loading_registration_libraries(self):
        script = '''
import runpy, socket, sys
class FakeSocket:
    def __init__(self, *args): pass
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
    raise AssertionError('registration did not reject this host')
assert 'open3d' not in sys.modules and 'numpy' not in sys.modules
'''
        result = subprocess.run([sys.executable, "-I", "-c", script,
                                 str(SOURCE / "lidar_camera_registration.py")],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("refusing to process sensor data", result.stderr)
        with patch.object(calibration, "_is_jetson_link", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "only runs on the Jetson"):
                calibration.LidarCameraRegistration()

    def test_cloud_decode_ignores_invalid_xyz_but_not_packed_color(self):
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in (("x", 0), ("y", 4), ("z", 8), ("rgb", 12))]
        msg = point_cloud2.create_cloud(Header(frame_id="camera"), fields,
                                        [[1.0, 2.0, 3.0, float("nan")],
                                         [float("nan"), 2.0, 3.0, 0.0],
                                         [1.0, float("inf"), 3.0, 0.0]])
        np.testing.assert_array_equal(np.asarray(calibration.point_cloud_from_message(msg).points),
                                      [[1.0, 2.0, 3.0]])

    def test_wall_patches_are_removed_in_sensor_coordinates_but_ground_is_kept(self):
        rng = np.random.default_rng(12)
        x, y = np.meshgrid(np.linspace(-2, 2, 100), np.linspace(-2, 2, 100))
        ground = np.column_stack((x.ravel(), y.ravel(), rng.normal(0, 0.003, x.size)))
        y, z = np.meshgrid(np.linspace(-2, 2, 100), np.linspace(0, 2.5, 65))
        wall = np.column_stack((3 + rng.normal(0, 0.003, y.size), y.ravel(), z.ravel()))
        objects = rng.uniform([-0.4, -0.4, 0.4], [0.4, 0.4, 1.2], (1000, 3))
        points = np.vstack((ground, wall, objects))
        config = calibration.DEFAULT_CONFIG._replace(fpfh_voxel_size_m=0.10)

        # Rotate the entire scene and the up vector as a camera or LiDAR frame
        # would. Detection must work without assuming the cloud's Z axis is up.
        for angles in ((0, 0, 0), (0.4, -0.3, 0.8)):
            with self.subTest(angles=angles):
                rotation = calibration.o3d.geometry.get_rotation_matrix_from_xyz(angles)
                cloud = calibration.o3d.geometry.PointCloud()
                cloud.points = calibration.o3d.utility.Vector3dVector(points @ rotation.T)
                cloud = cloud.voxel_down_sample(0.05)
                before = np.asarray(cloud.points).copy()
                filtered, walls, removed = calibration.remove_wall_patches(
                    cloud, rotation @ [0, 0, 1], config)
                remaining = np.asarray(filtered.points) @ rotation
                original = before @ rotation
                self.assertGreaterEqual(walls, 1)
                self.assertGreater(removed, 0)
                self.assertLess(np.count_nonzero(np.abs(remaining[:, 0] - 3) < 0.02),
                                np.count_nonzero(np.abs(original[:, 0] - 3) < 0.02) * 0.05)
                self.assertGreater(np.count_nonzero(np.abs(remaining[:, 2]) < 0.02),
                                   np.count_nonzero(np.abs(original[:, 2]) < 0.02) * 0.95)
                self.assertGreater(np.count_nonzero((np.abs(remaining[:, 0]) < 0.45)
                                                    & (remaining[:, 2] > 0.4)), 800)
                np.testing.assert_array_equal(np.asarray(cloud.points), before)

    def test_up_vector_uses_the_internal_tf_in_the_correct_direction(self):
        transform = Transform()
        transform.rotation.x = math.sin(math.pi / 4)
        transform.rotation.w = math.cos(math.pi / 4)
        buffer = Mock()
        buffer.lookup_transform.return_value = SimpleNamespace(transform=transform)
        up = calibration.up_in_cloud_frame(buffer, "zed_front_optical", "zed_front_camera_link")
        np.testing.assert_allclose(up, [0, -1, 0], atol=1e-12)
        self.assertEqual(buffer.lookup_transform.call_args.args[:2],
                         ("zed_front_optical", "zed_front_camera_link"))

    def test_initial_pose_requires_distinct_scans_and_pairwise_agreement(self):
        pending = {}
        config = calibration.DEFAULT_CONFIG
        frames = ("os_lidar", "zed_front_left_camera_frame")

        def candidate(x, lidar_sec, camera_sec):
            pose = np.eye(4)
            pose[0, 3] = x
            lidar_stamp = SimpleNamespace(sec=lidar_sec, nanosec=0)
            camera_stamp = SimpleNamespace(sec=camera_sec, nanosec=0)
            return calibration.confirm_initial_pose(
                pending, "front", frames, pose, lidar_stamp, camera_stamp, config)

        self.assertFalse(candidate(0.00, 1, 1)[0])
        self.assertFalse(candidate(0.00, 1, 2)[0])  # Reusing the LiDAR scan does not count.
        self.assertFalse(candidate(0.00, 2, 1)[0])  # Reusing the camera scan does not count either.
        self.assertEqual(len(pending["front"]["poses"]), 1)
        self.assertFalse(candidate(0.04, 2, 2)[0])
        accepted, reason = candidate(-0.04, 3, 3)  # 8 cm from the second candidate: reset.
        self.assertFalse(accepted)
        self.assertIn("restarting confirmations (1/3)", reason)
        self.assertEqual(len(pending["front"]["poses"]), 1)
        self.assertFalse(candidate(-0.03, 4, 4)[0])
        self.assertTrue(candidate(-0.02, 5, 5)[0])
        # The three confirmations survive until robot_state_publisher accepts
        # the URDF, so a failed service update can be retried using the same scan.
        self.assertEqual(len(pending["front"]["poses"]), 3)
        self.assertTrue(candidate(-0.02, 5, 5)[0])
        self.assertFalse(candidate(1.00, 5, 5)[0])
        self.assertEqual(len(pending["front"]["poses"]), 3)

    def test_camera_confirmation_counts_are_independent(self):
        pending = {}
        config = calibration.DEFAULT_CONFIG

        def candidate(name, x, second):
            pose = np.eye(4)
            pose[0, 3] = x
            stamp = SimpleNamespace(sec=second, nanosec=0)
            return calibration.confirm_initial_pose(
                pending, name, ("os_lidar", name), pose, stamp, stamp, config)

        candidate("front", 0.0, 1)
        candidate("left", 0.0, 1)
        candidate("front", 0.0, 2)
        self.assertIn("restarting", candidate("left", 1.0, 2)[1])
        self.assertEqual(len(pending["left"]["poses"]), 1)
        self.assertEqual(len(pending["front"]["poses"]), 2)
        self.assertTrue(candidate("front", 0.0, 3)[0])
        self.assertEqual(len(pending["left"]["poses"]), 1)

    def test_new_quality_settings_reject_invalid_values(self):
        for name, value in (("icp_max_rmse_m", 0.0), ("max_camera_lidar_distance_m", 0.0),
                            ("initial_pose_confirmations", 0), ("pose_agreement_rotation_deg", 181.0),
                            ("initial_pose_confirmation_enabled", "false"),
                            ("wall_max_tilt_deg", 90.0), ("wall_distance_tolerance_m", 0.0),
                            ("visualization_voxel_size_m", -0.1)):
            with self.subTest(name=name):
                node = Mock()
                node.declare_parameter.side_effect = lambda key, default, descriptor: (
                    SimpleNamespace(value=value if key == name else default))
                with self.assertRaisesRegex(ValueError, name):
                    calibration.load_registration_config(node)

    def test_confirmation_setting_is_loaded_as_a_boolean(self):
        node = Mock()
        node.declare_parameter.side_effect = lambda name, default, descriptor: SimpleNamespace(
            value=False if name == "initial_pose_confirmation_enabled" else default)
        config = calibration.load_registration_config(node)
        self.assertFalse(config.initial_pose_confirmation_enabled)

    def test_icp_rejects_high_match_error(self):
        cloud = calibration.o3d.geometry.PointCloud()
        cloud.points = calibration.o3d.utility.Vector3dVector(np.ones((40, 3)))
        result = SimpleNamespace(transformation=np.eye(4), fitness=0.8,
                                 correspondence_set=[0] * 40, inlier_rmse=0.12)
        metrics = {}
        with patch.object(calibration.o3d.pipelines.registration, "registration_icp", return_value=result):
            registered, pose, reason = calibration.register_camera(
                cloud, cloud, np.eye(4),
                calibration.DEFAULT_CONFIG._replace(initial_pose_confirmation_enabled=False), metrics)
        self.assertIsNone(registered)
        self.assertIsNone(pose)
        self.assertIn("match error too high", reason)
        self.assertEqual(metrics, {"fitness": 0.8, "rmse": 0.12, "matches": 40})

    def test_ransac_rejection_reports_the_active_threshold(self):
        cloud = calibration.o3d.geometry.PointCloud()
        cloud.points = calibration.o3d.utility.Vector3dVector(np.ones((40, 3)))
        result = SimpleNamespace(transformation=np.eye(4), fitness=0.10, correspondence_set=[0] * 40)
        config = calibration.DEFAULT_CONFIG._replace(ransac_min_fitness=0.80)
        metrics = {}
        with patch.object(calibration, "prepare_fpfh", return_value=(cloud, Mock())), \
                patch.object(calibration.o3d.pipelines.registration,
                             "registration_ransac_based_on_feature_matching", return_value=result):
            pose, reason = calibration.estimate_initial_pose(cloud, (cloud, Mock()), config, metrics)
        self.assertIsNone(pose)
        self.assertIn("fitness 0.10, minimum 0.80", reason)
        self.assertEqual(metrics, {"fitness": 0.10, "matches": 40})

    def test_missing_lidar_features_skip_camera_feature_computation(self):
        with patch.object(calibration, "prepare_fpfh") as prepare:
            pose, reason = calibration.estimate_initial_pose(Mock(), (None, None), calibration.DEFAULT_CONFIG)
        self.assertIsNone(pose)
        self.assertEqual(reason, "too few points for FPFH")
        prepare.assert_not_called()

    def test_ransac_and_icp_keep_their_overlap_thresholds(self):
        cloud = calibration.o3d.geometry.PointCloud()
        cloud.points = calibration.o3d.utility.Vector3dVector(np.ones((40, 3)))
        config = calibration.DEFAULT_CONFIG._replace(ransac_min_fitness=0.2, icp_min_fitness=0.1)
        for fitness, matches, ransac_ok, icp_ok in ((0.2, 30, True, True), (0.1, 30, False, True),
                                                     (0.09, 40, False, False), (0.8, 29, False, False),
                                                     (float("nan"), 40, False, False)):
            with self.subTest(fitness=fitness, matches=matches):
                result = SimpleNamespace(transformation=np.eye(4), fitness=fitness,
                                         correspondence_set=[0] * matches, inlier_rmse=0.02)
                with patch.object(calibration, "prepare_fpfh", return_value=(cloud, Mock())), \
                        patch.object(calibration.o3d.pipelines.registration,
                                     "registration_ransac_based_on_feature_matching", return_value=result), \
                        patch.object(calibration.o3d.pipelines.registration, "registration_icp", return_value=result):
                    pose, ransac_reason = calibration.estimate_initial_pose(cloud, (cloud, Mock()), config)
                    registered, _, icp_reason = calibration.register_camera(cloud, cloud, np.eye(4), config)
                self.assertEqual(pose is not None, ransac_ok)
                self.assertEqual(registered is not None, icp_ok)
                if not ransac_ok:
                    self.assertIn("insufficient RANSAC overlap", ransac_reason)
                if not icp_ok:
                    self.assertIn("insufficient ICP overlap", icp_reason)

    def test_icp_rejects_a_camera_outside_the_vehicle_even_with_small_correction(self):
        cloud = calibration.o3d.geometry.PointCloud()
        cloud.points = calibration.o3d.utility.Vector3dVector(np.ones((40, 3)))
        wrong_pose = np.eye(4)
        wrong_pose[0, 3] = 4.0
        result = SimpleNamespace(transformation=wrong_pose, fitness=0.8,
                                 correspondence_set=[0] * 40, inlier_rmse=0.02)
        with patch.object(calibration.o3d.pipelines.registration, "registration_icp", return_value=result):
            registered, pose, reason = calibration.register_camera(
                cloud, cloud, wrong_pose,
                calibration.DEFAULT_CONFIG._replace(initial_pose_confirmation_enabled=False))
        self.assertIsNone(registered)
        self.assertIsNone(pose)
        self.assertIn("camera-to-LiDAR distance too large", reason)

    def test_filtered_lidar_and_camera_are_used_for_both_fpfh_and_icp(self):
        config = calibration.DEFAULT_CONFIG._replace(initial_pose_confirmations=1)

        def cloud(offset):
            result = calibration.o3d.geometry.PointCloud()
            result.points = calibration.o3d.utility.Vector3dVector(
                np.arange(120, dtype=float).reshape(40, 3) + offset)
            return result

        lidar, camera = cloud(0), cloud(10)
        lidar_filtered, camera_filtered = cloud(20), cloud(30)
        lidar_msg = message("os_lidar")
        camera_msg = message("zed_front_left_camera_frame")
        node = SimpleNamespace(
            config=config, tf_buffer=Mock(), state_lock=Lock(), data_lock=Lock(), pending_candidates={},
            camera_anchors={}, camera_failures={}, retry_global=set(), get_logger=Mock(),
            description_client=Mock(), update_in_flight=False, pending_update=None, update_started=0.0,
            description_update_done=Mock(), next_calibration=0.0, latest_lidar=(lidar_msg, 100.0),
            camera_clouds={"front": (camera_msg, 100.0)}, camera_poses={}, camera_mounts={},
            robot_description=URDF, registered_pub=Mock(),
            input_pubs={name: Mock() for name in ("lidar", *calibration.CAMERAS)},
        )
        node.description_client.service_is_ready.return_value = True
        node.prepare_calibration_input = lambda *args: (
            calibration.LidarCameraRegistration.prepare_calibration_input(node, *args))
        node.publish_registered_inputs = lambda *args: (
            calibration.LidarCameraRegistration.publish_registered_inputs(node, *args))
        node.calibrate = lambda *args: calibration.LidarCameraRegistration.calibrate(node, *args)
        features = (lidar_filtered, Mock())
        with patch.object(calibration, "up_in_cloud_frame", return_value=np.array([0, 0, 1])) as up, \
                patch.object(calibration, "remove_wall_patches", side_effect=[
                    (lidar_filtered, 1, 10), (camera_filtered, 1, 10)]) as remove, \
                patch.object(calibration, "crop_calibration_cloud", side_effect=lambda cloud, *_: cloud), \
                patch.object(calibration, "point_cloud_from_message", side_effect=[lidar, camera]), \
                patch.object(calibration, "prepare_fpfh", return_value=features) as fpfh, \
                patch.object(calibration, "estimate_initial_pose", return_value=(np.eye(4), None)) as ransac, \
                patch.object(calibration, "register_camera", return_value=(camera_filtered, np.eye(4), None)) as icp, \
                patch.object(calibration, "camera_mount_matrix", return_value=np.eye(4)), \
                patch.object(calibration.time, "monotonic", return_value=100.0):
            calibration.LidarCameraRegistration.calibration_tick(node)

        self.assertEqual(remove.call_count, 2)
        self.assertEqual(up.call_args_list[0].args[1:], ("os_lidar", "base_link"))
        self.assertEqual(up.call_args_list[1].args[1:],
                         ("zed_front_left_camera_frame", "zed_front_camera_link"))
        fpfh.assert_called_once_with(lidar_filtered, config)
        self.assertIs(ransac.call_args.args[0], camera_filtered)
        self.assertIs(ransac.call_args.args[1], features)
        self.assertIs(icp.call_args.args[0], camera_filtered)
        self.assertIs(icp.call_args.args[1], lidar_filtered)
        node.description_client.call_async.assert_called_once()
        # Inspectable inputs are the same clouds, with original frames, before
        # the first pose is accepted or the robot description is updated.
        for name, expected, frame in (("lidar", lidar_filtered, "os_lidar"),
                                      ("front", camera_filtered, camera_msg.header.frame_id)):
            output = node.input_pubs[name].publish.call_args.args[0]
            self.assertEqual(output.header.frame_id, frame)
            xyz = point_cloud2.read_points(output, field_names=("x", "y", "z"))
            np.testing.assert_allclose(np.column_stack([xyz[axis] for axis in "xyz"]),
                                       np.asarray(expected.points))
        self.assertEqual(node.registered_pub.publish.call_args.args[0].width, 0)

    def test_missing_lidar_tf_skips_calibration_instead_of_using_walls(self):
        lidar = message()
        cloud = calibration.o3d.geometry.PointCloud()
        cloud.points = calibration.o3d.utility.Vector3dVector(np.arange(120).reshape(40, 3))
        buffer = Mock()
        buffer.lookup_transform.side_effect = calibration.TransformException("missing TF")
        node = SimpleNamespace(config=calibration.DEFAULT_CONFIG, tf_buffer=buffer, get_logger=Mock(),
                               data_lock=Lock(), state_lock=Lock(), next_calibration=0.0,
                               latest_lidar=(lidar, 100.0), camera_clouds={}, robot_description=URDF,
                               camera_poses={}, camera_mounts={}, retry_global=set(), update_in_flight=False,
                               pending_update=None, update_started=0.0, calibrate=Mock(),
                               input_pubs={name: Mock() for name in ("lidar", *calibration.CAMERAS)},
                               publish_registered_inputs=Mock())
        node.prepare_calibration_input = lambda *args: (
            calibration.LidarCameraRegistration.prepare_calibration_input(node, *args))
        with patch.object(calibration, "remove_wall_patches") as remove, \
                patch.object(calibration, "point_cloud_from_message", return_value=cloud), \
                patch.object(calibration.time, "monotonic", return_value=100.0):
            calibration.LidarCameraRegistration.calibration_tick(node)
        node.calibrate.assert_not_called()
        remove.assert_not_called()
        self.assertEqual(node.input_pubs["lidar"].publish.call_args.args[0].width, 0)
        self.assertIn("calibration input failed", node.get_logger.return_value.warning.call_args_list[0].args[0])

    def test_filtered_lidar_is_visible_without_cameras_or_urdf(self):
        rng = np.random.default_rng(12)
        y, z = np.meshgrid(np.linspace(-2, 2, 100), np.linspace(0, 2.5, 65))
        wall = np.column_stack((3 + rng.normal(0, 0.003, y.size), y.ravel(), z.ravel()))
        x, y = np.meshgrid(np.linspace(-2, 2, 100), np.linspace(-2, 2, 100))
        ground = np.column_stack((x.ravel(), y.ravel(), rng.normal(0, 0.003, x.size)))
        header = Header(frame_id="os_lidar")
        header.stamp.sec = 100
        lidar = point_cloud2.create_cloud_xyz32(header, np.vstack((wall, ground)).tolist())
        original = bytes(lidar.data)
        node = SimpleNamespace(
            config=calibration.DEFAULT_CONFIG._replace(icp_voxel_size_m=0.05, fpfh_voxel_size_m=0.10),
            tf_buffer=identity_tf_buffer(), get_logger=Mock(), data_lock=Lock(), state_lock=Lock(),
            next_calibration=0.0, latest_lidar=(lidar, 100.0), camera_clouds={},
            robot_description=None, camera_poses={}, camera_mounts={}, retry_global=set(),
            update_in_flight=False, pending_update=None, update_started=0.0,
            registered_pub=Mock(), calibrate=Mock(),
            input_pubs={name: Mock() for name in ("lidar", *calibration.CAMERAS)},
        )
        node.prepare_calibration_input = lambda *args: (
            calibration.LidarCameraRegistration.prepare_calibration_input(node, *args))
        node.publish_registered_inputs = lambda *args: (
            calibration.LidarCameraRegistration.publish_registered_inputs(node, *args))
        with patch.object(calibration, "up_in_cloud_frame", return_value=np.array([0, 0, 1])), \
                patch.object(calibration.time, "monotonic", return_value=100.0):
            calibration.LidarCameraRegistration.calibration_tick(node)

        output = node.input_pubs["lidar"].publish.call_args.args[0]
        xyz = point_cloud2.read_points(output, field_names=("x", "y", "z"))
        self.assertEqual(output.header, header)
        self.assertLess(np.count_nonzero(np.abs(xyz["x"] - 3) < 0.02), 100)
        self.assertGreater(np.count_nonzero(np.abs(xyz["z"]) < 0.02), 5000)
        self.assertEqual(bytes(lidar.data), original)
        self.assertEqual(node.registered_pub.publish.call_args.args[0].width, 0)
        node.calibrate.assert_not_called()

    def test_camera_input_is_visible_without_a_lidar_pair(self):
        header = Header(frame_id="zed_front_left_camera_frame")
        header.stamp.sec = 100
        camera = point_cloud2.create_cloud_xyz32(header, np.arange(120).reshape(40, 3).tolist())
        filtered = calibration.o3d.geometry.PointCloud()
        filtered.points = calibration.o3d.utility.Vector3dVector(np.arange(60).reshape(20, 3))
        node = SimpleNamespace(
            config=calibration.DEFAULT_CONFIG, tf_buffer=identity_tf_buffer(),
            get_logger=Mock(), data_lock=Lock(), state_lock=Lock(), next_calibration=0.0,
            latest_lidar=None, camera_clouds={"front": (camera, 100.0)}, robot_description=None,
            camera_poses={}, camera_mounts={}, retry_global=set(), update_in_flight=False,
            pending_update=None, update_started=0.0, calibrate=Mock(),
            input_pubs={name: Mock() for name in ("lidar", *calibration.CAMERAS)},
        )
        node.prepare_calibration_input = lambda *args: (
            calibration.LidarCameraRegistration.prepare_calibration_input(node, *args))
        with patch.object(calibration.time, "monotonic", return_value=100.0), \
                patch.object(calibration, "up_in_cloud_frame", return_value=np.array([0, 0, 1])) as up, \
                patch.object(calibration, "remove_wall_patches", return_value=(filtered, 1, 20)):
            calibration.LidarCameraRegistration.calibration_tick(node)
        output = node.input_pubs["front"].publish.call_args.args[0]
        self.assertEqual(output.header, header)
        self.assertEqual(output.width, 20)
        self.assertEqual(up.call_args.args[1:], (header.frame_id, "zed_front_camera_link"))
        node.calibrate.assert_not_called()

    def test_filter_disabled_displays_cropped_clouds_but_matches_downsampled_copies(self):
        lidar_header = Header(frame_id="os_lidar")
        camera_header = Header(frame_id="zed_front_left_camera_frame")
        lidar_header.stamp.sec = camera_header.stamp.sec = 100
        points = [[1.0, 0.0, 0.0], [1.001, 0.0, 0.0], [6.0, 0.0, 0.0],
                  [float("nan"), 0.0, 0.0]]
        lidar = point_cloud2.create_cloud_xyz32(lidar_header, points)
        camera = point_cloud2.create_cloud_xyz32(camera_header, points)
        config = calibration.DEFAULT_CONFIG._replace(wall_filter_enabled=False, min_points=1)
        pose = np.eye(4)
        node = SimpleNamespace(
            config=config, tf_buffer=identity_tf_buffer(), get_logger=Mock(), data_lock=Lock(), state_lock=Lock(),
            next_calibration=0.0, latest_lidar=(lidar, 100.0), camera_clouds={"front": (camera, 100.0)},
            robot_description=URDF, camera_poses={"front": ("os_lidar", camera_header.frame_id, pose)},
            camera_mounts={}, retry_global=set(), update_in_flight=False, pending_update=None,
            update_started=0.0, registered_pub=Mock(), calibrate=Mock(), description_client=Mock(),
            input_pubs={name: Mock() for name in ("lidar", *calibration.CAMERAS)},
        )
        node.description_client.service_is_ready.return_value = True
        node.prepare_calibration_input = lambda *args: (
            calibration.LidarCameraRegistration.prepare_calibration_input(node, *args))
        node.publish_registered_inputs = lambda *args: (
            calibration.LidarCameraRegistration.publish_registered_inputs(node, *args))
        with patch.object(calibration.time, "monotonic", return_value=100.0), \
                patch.object(calibration, "remove_wall_patches") as remove:
            calibration.LidarCameraRegistration.calibration_tick(node)

        remove.assert_not_called()
        self.assertEqual(node.input_pubs["lidar"].publish.call_args.args[0].width, 2)
        self.assertEqual(node.input_pubs["front"].publish.call_args.args[0].width, 2)
        self.assertEqual(node.registered_pub.publish.call_args.args[0].width, 4)
        lidar_for_matching, _, cameras_for_matching = node.calibrate.call_args.args[:3]
        self.assertEqual(len(lidar_for_matching.points), 1)
        self.assertEqual(len(cameras_for_matching["front"][1].points), 1)

    def test_only_confirmed_global_poses_update_urdf_and_later_drift_is_rejected(self):
        header = Header(frame_id="os_lidar")
        camera = message("zed_front_left_camera_frame")
        config = calibration.DEFAULT_CONFIG._replace(wall_filter_enabled=False)
        source = Mock()
        source.voxel_down_sample.return_value = source
        future = Mock()
        future.result.return_value.results = [SimpleNamespace(successful=True, reason="")]
        with tempfile.TemporaryDirectory() as directory:
            node = SimpleNamespace(
                config=config, state_lock=Lock(), pending_candidates={}, camera_anchors={},
                camera_failures={}, retry_global=set(), camera_poses={}, camera_mounts={},
                tf_buffer=Mock(), description_client=Mock(), get_logger=Mock(),
                update_in_flight=False, pending_update=None, update_started=0.0,
                robot_description=URDF, output_urdf=Path(directory) / "calibrated.urdf",
            )
            node.description_client.call_async.return_value = future
            node.description_update_done = lambda *args: (
                calibration.LidarCameraRegistration.description_update_done(node, *args))
            pose = np.eye(4)

            def good_icp(camera_cloud, lidar_cloud, initial, settings, metrics):
                metrics.update(fitness=0.8, rmse=0.02, matches=40)
                return camera_cloud, pose.copy(), None

            reasons = {name: "no cloud" for name in calibration.CAMERAS}
            reasons["front"] = None
            with patch.object(calibration, "point_cloud_from_message", return_value=source), \
                    patch.object(calibration, "prepare_fpfh", return_value=(source, Mock())), \
                    patch.object(calibration, "estimate_initial_pose", return_value=(np.eye(4), None)) as ransac, \
                    patch.object(calibration, "register_camera", side_effect=good_icp), \
                    patch.object(calibration, "camera_mount_matrix", return_value=np.eye(4)):
                for second in (100, 101):
                    header.stamp.sec = camera.header.stamp.sec = second
                    calibration.LidarCameraRegistration.calibrate(
                        node, source, header, {"front": (camera, source)}, reasons,
                        node.camera_poses, node.camera_mounts, node.retry_global, URDF)
                    node.description_client.call_async.assert_not_called()
                header.stamp.sec = camera.header.stamp.sec = 102
                calibration.LidarCameraRegistration.calibrate(
                    node, source, header, {"front": (camera, source)}, reasons,
                    node.camera_poses, node.camera_mounts, node.retry_global, URDF)
                node.description_client.call_async.assert_called_once()
                callback = future.add_done_callback.call_args.args[0]
                callback(future)
                self.assertIn("front", node.camera_anchors)
                node.description_client.call_async.reset_mock()
                self.assertEqual(ransac.call_count, 3)

                # Each update starts close to the cached pose, but the anchor
                # prevents a 6 cm total drift from being published.
                pose[0, 3] = 0.06
                for second in (103, 104, 105):
                    header.stamp.sec = camera.header.stamp.sec = second
                    calibration.LidarCameraRegistration.calibrate(
                        node, source, header, {"front": (camera, source)}, reasons,
                        node.camera_poses, node.camera_mounts, node.retry_global, URDF)
                node.description_client.call_async.assert_not_called()
                self.assertIn("front", node.retry_global)
                np.testing.assert_allclose(node.camera_poses["front"][2], np.eye(4))
                self.assertEqual(ransac.call_count, 3)  # Cached ICP did not rerun RANSAC.
                pose[0, 3] = 0.0
                header.stamp.sec = camera.header.stamp.sec = 106
                calibration.LidarCameraRegistration.calibrate(
                    node, source, header, {"front": (camera, source)}, reasons,
                    node.camera_poses, node.camera_mounts, node.retry_global, URDF)
                self.assertEqual(ransac.call_count, 4)  # Three ICP failures triggered RANSAC.
                self.assertEqual(len(node.pending_candidates["front"]["poses"]), 1)

    def test_failed_search_keeps_a_cameras_confirmations_and_does_not_block_another(self):
        header = Header(frame_id="os_lidar")
        front = message("zed_front_left_camera_frame")
        left = message("zed_left_left_camera_frame")
        front_cloud, left_cloud, lidar = Mock(), Mock(), Mock()
        config = calibration.DEFAULT_CONFIG._replace(wall_filter_enabled=False)
        node = SimpleNamespace(
            config=config, state_lock=Lock(), pending_candidates={}, camera_anchors={},
            camera_failures={}, retry_global=set(), camera_poses={}, camera_mounts={}, tf_buffer=Mock(),
            description_client=Mock(), get_logger=Mock(), update_in_flight=False,
            pending_update=None, update_started=0.0, description_update_done=Mock(),
        )
        reasons = {name: "no cloud" for name in calibration.CAMERAS}
        reasons.update(front=None, left=None)
        cameras = {"front": (front, front_cloud), "left": (left, left_cloud)}

        def ransac(cloud, features, settings, metrics):
            if cloud is front_cloud and header.stamp.sec == 101:
                return None, "insufficient RANSAC overlap"
            return np.eye(4), None

        def icp(cloud, target, initial, settings, metrics):
            if cloud is left_cloud:
                return None, None, "camera-to-LiDAR distance too large (4.30 m)"
            if header.stamp.sec == 102:
                return None, None, "insufficient ICP overlap"
            return cloud, np.eye(4), None

        with patch.object(calibration, "prepare_fpfh", return_value=(lidar, Mock())), \
                patch.object(calibration, "estimate_initial_pose", side_effect=ransac), \
                patch.object(calibration, "register_camera", side_effect=icp), \
                patch.object(calibration, "camera_mount_matrix", return_value=np.eye(4)), \
                patch.object(calibration, "update_camera_joints", wraps=calibration.update_camera_joints) as update:
            for second, expected in ((100, 1), (101, 1), (102, 1), (103, 2), (104, 3)):
                header.stamp.sec = front.header.stamp.sec = left.header.stamp.sec = second
                calibration.LidarCameraRegistration.calibrate(
                    node, lidar, header, cameras, reasons, node.camera_poses,
                    node.camera_mounts, node.retry_global, URDF)
                self.assertEqual(len(node.pending_candidates["front"]["poses"]), expected)
            node.description_client.call_async.assert_called_once()
            self.assertEqual(set(update.call_args.args[1]), {"front"})
            self.assertNotIn("left", node.pending_candidates)
        warnings = " ".join(call.args[0] for call in node.get_logger.return_value.warning.call_args_list)
        self.assertIn("keeping 1/3 confirmations", warnings)
        self.assertIn("camera-to-LiDAR distance too large (4.30 m)", warnings)

    def test_single_estimate_mode_still_requires_a_valid_ransac_and_icp_result(self):
        header = Header(frame_id="os_lidar")
        camera = message("zed_front_left_camera_frame")
        source, lidar = Mock(), Mock()
        reasons = {name: "no cloud" for name in calibration.CAMERAS}
        reasons["front"] = None
        config = calibration.DEFAULT_CONFIG._replace(initial_pose_confirmation_enabled=False)

        for outcome in ("accepted", "ransac_failed", "icp_failed"):
            with self.subTest(outcome=outcome):
                node = SimpleNamespace(
                    config=config, state_lock=Lock(), pending_candidates={}, camera_anchors={},
                    camera_failures={}, retry_global=set(), camera_poses={}, camera_mounts={}, tf_buffer=Mock(),
                    description_client=Mock(), get_logger=Mock(), update_in_flight=False,
                    pending_update=None, update_started=0.0, description_update_done=Mock(),
                )
                ransac_result = ((None, "insufficient RANSAC overlap") if outcome == "ransac_failed"
                                 else (np.eye(4), None))
                icp_result = ((None, None, "ICP match error too high") if outcome == "icp_failed"
                              else (source, np.eye(4), None))
                with patch.object(calibration, "prepare_fpfh", return_value=(lidar, Mock())), \
                        patch.object(calibration, "estimate_initial_pose", return_value=ransac_result), \
                        patch.object(calibration, "register_camera", return_value=icp_result), \
                        patch.object(calibration, "confirm_initial_pose") as confirm, \
                        patch.object(calibration, "camera_mount_matrix", return_value=np.eye(4)):
                    calibration.LidarCameraRegistration.calibrate(
                        node, lidar, header, {"front": (camera, source)}, reasons,
                        {}, {}, set(), URDF)
                confirm.assert_not_called()
                self.assertEqual(node.description_client.call_async.call_count, int(outcome == "accepted"))
                self.assertNotIn("front", node.pending_candidates)
                warnings = node.get_logger.return_value.warning
                if outcome == "accepted":
                    warnings.assert_not_called()  # Cameras that were never started are not failures.
                else:
                    warnings.assert_called_once()
                    self.assertNotIn("rear: no cloud", warnings.call_args.args[0])
                    self.assertEqual(warnings.call_args.kwargs["throttle_duration_sec"], 30.0)

    def test_cloud_frame_change_invalidates_only_that_cameras_pending_estimates(self):
        pending = {
            "front": {"frames": ("os_lidar", "old_front"), "poses": [np.eye(4)], "observations": []},
            "left": {"frames": ("os_lidar", "left"), "poses": [np.eye(4)], "observations": []},
        }
        node = SimpleNamespace(
            config=calibration.DEFAULT_CONFIG, state_lock=Lock(), pending_candidates=pending,
            camera_failures={}, retry_global=set(), get_logger=Mock(),
        )
        reasons = {name: "no cloud" for name in calibration.CAMERAS}
        reasons["front"] = None
        with patch.object(calibration, "prepare_fpfh", return_value=(Mock(), Mock())), \
                patch.object(calibration, "estimate_initial_pose", return_value=(None, "no match")):
            calibration.LidarCameraRegistration.calibrate(
                node, Mock(), Header(frame_id="os_lidar"), {"front": (message("new_front"), Mock())},
                reasons, {}, {}, set(), URDF)
        self.assertNotIn("front", pending)
        self.assertIn("left", pending)

    def test_registered_display_requires_a_usable_camera_and_preserves_lidar(self):
        lidar_header = Header(frame_id="os_lidar")
        lidar_header.stamp.sec = 100
        lidar = point_cloud2.create_cloud_xyz32(lidar_header, [[1, 2, 3], [1.001, 2, 3], [4, 5, 6]])
        original_lidar = bytes(lidar.data)
        camera_header = Header(frame_id="zed_front_left_camera_frame")
        camera_header.stamp.sec = 100
        camera = point_cloud2.create_cloud_xyz32(camera_header, [[0.25, -0.5, 1], [0.251, -0.5, 1]])
        node = SimpleNamespace(config=calibration.DEFAULT_CONFIG, registered_pub=Mock())
        lidar_input = calibration.point_cloud_from_message(lidar)
        camera_input = calibration.point_cloud_from_message(camera)
        cameras = {"front": (camera, camera_input)}
        reasons = {"front": None}

        # A received camera is not a registered camera until its pose is accepted.
        calibration.LidarCameraRegistration.publish_registered_inputs(
            node, lidar_header, lidar_input, cameras, reasons, {})
        output = node.registered_pub.publish.call_args.args[0]
        self.assertEqual(output.width, 0)
        self.assertEqual(output.header.frame_id, "os_lidar")

        pose = np.eye(4)
        pose[0, 3] = 2.0
        poses = {"front": ("os_lidar", camera_header.frame_id, pose)}
        calibration.LidarCameraRegistration.publish_registered_inputs(
            node, lidar_header, lidar_input, cameras, reasons, poses)
        output = node.registered_pub.publish.call_args.args[0]
        points = point_cloud2.read_points(output, field_names=("x", "y", "z", "rgb"))
        lidar_color = (230 << 16) | (230 << 8) | 230
        camera_color = (40 << 16) | (160 << 8) | 255
        lidar_points = points[points["rgb"] == lidar_color]
        camera_points = points[points["rgb"] == camera_color]
        # The already-filtered input is used directly; the camera is transformed
        # in a copy, preserving its original points for calibration.
        self.assertEqual(len(lidar_points), 3)
        np.testing.assert_allclose(np.sort(lidar_points["x"]), [1, 1.001, 4], atol=1e-6)
        self.assertEqual(len(camera_points), 2)
        np.testing.assert_allclose(np.sort(camera_points["x"]), [2.25, 2.251], atol=1e-6)
        np.testing.assert_allclose(camera_points["y"], [-0.5, -0.5])
        np.testing.assert_allclose(camera_points["z"], [1, 1])
        self.assertEqual(bytes(lidar.data), original_lidar)
        np.testing.assert_allclose(np.asarray(camera_input.points)[:, 0], [0.25, 0.251])

        # A stale camera must clear the previously displayed result.
        calibration.LidarCameraRegistration.publish_registered_inputs(
            node, lidar_header, lidar_input, cameras, {"front": "camera cloud is stale"}, poses)
        self.assertEqual(node.registered_pub.publish.call_args.args[0].width, 0)

    def test_compose_lidar_and_camera_internal_frames_in_correct_direction(self):
        base_from_lidar = Transform()
        base_from_lidar.translation.x = 1.0
        base_from_lidar.translation.y = 2.0
        base_from_lidar.translation.z = 3.0
        base_from_lidar.rotation.z = math.sin(math.pi / 4)
        base_from_lidar.rotation.w = math.cos(math.pi / 4)
        camera_from_body = Transform()
        camera_from_body.translation.x = -1.0
        camera_from_body.rotation.x = math.sin(math.pi / 4)
        camera_from_body.rotation.w = math.cos(math.pi / 4)
        buffer = Mock()
        buffer.lookup_transform.side_effect = [SimpleNamespace(transform=base_from_lidar),
                                                SimpleNamespace(transform=camera_from_body)]
        camera_to_lidar = np.eye(4)
        camera_to_lidar[:3, 3] = [4, 5, 6]
        mount = calibration.camera_mount_matrix(buffer, "os_lidar", "zed_front_optical",
                                                   "zed_front_camera_link", camera_to_lidar)
        # Body point -> rotate about X and offset -> camera-to-LiDAR -> rotate about Z and offset.
        np.testing.assert_allclose(mount @ [0, 1, 0, 1], [-4, 5, 10, 1], atol=1e-12)
        self.assertEqual(buffer.lookup_transform.call_args_list[0].args[:2], ("base_link", "os_lidar"))
        self.assertEqual(buffer.lookup_transform.call_args_list[1].args[:2],
                         ("zed_front_optical", "zed_front_camera_link"))

    def test_urdf_joint_update_preserves_lidar_and_unmatched_cameras(self):
        pose = np.eye(4)
        pose[:3, 3] = [1.2, -0.2, 1.1]
        pose[:3, :3] = rotation_from_rpy(0.3, -0.4, 0.7)
        root = ET.fromstring(calibration.update_camera_joints(URDF, {"front": pose}))
        front = root.find("./joint[@name='zed_front_mount']")
        self.assertEqual(front.get("type"), "fixed")
        origin = front.find("origin")
        np.testing.assert_allclose([float(v) for v in origin.get("xyz").split()], pose[:3, 3])
        np.testing.assert_allclose(rotation_from_rpy(*[float(v) for v in origin.get("rpy").split()]),
                                   pose[:3, :3], atol=1e-10)
        self.assertEqual(root.find("./joint[@name='zed_left_mount']").get("type"), "floating")
        lidar = root.find("./joint[@name='lidar_mount']/origin")
        self.assertEqual(lidar.get("xyz"), "0.015 -0.105 1.26")
        self.assertEqual(lidar.get("rpy"), "0 0.0698 0.215")

    def test_urdf_rotation_handles_both_gimbal_lock_directions(self):
        for pitch in (math.pi / 2, -math.pi / 2):
            with self.subTest(pitch=pitch):
                pose = np.eye(4)
                pose[:3, :3] = rotation_from_rpy(0.3, pitch, 0.7)
                root = ET.fromstring(calibration.update_camera_joints(URDF, {"front": pose}))
                rpy = root.find("./joint[@name='zed_front_mount']/origin").get("rpy")
                np.testing.assert_allclose(rotation_from_rpy(*map(float, rpy.split())),
                                           pose[:3, :3], atol=1e-10)

    def test_invalid_mount_or_wrong_urdf_is_rejected(self):
        invalid = np.eye(4)
        invalid[0, 0] = 2
        for description, pose in ((URDF, invalid), (URDF.replace("vehicle_1811", "other"), np.eye(4))):
            with self.assertRaises(ValueError):
                calibration.update_camera_joints(description, {"front": pose})

    def test_source_has_only_floating_camera_mounts_and_measured_lidar(self):
        description_path = SOURCE.parent / "vehicle_1811_description" / "urdf"
        macro = ET.parse(description_path / "vehicle_1811.macros.xacro").getroot()
        ns = {"xacro": "http://www.ros.org/wiki/xacro"}
        camera = macro.find("xacro:macro[@name='zed_x_camera']", ns)
        self.assertEqual(camera.find("joint[@name='zed_${name}_mount']").get("type"), "floating")
        root = ET.parse(description_path / "vehicle_1811.urdf.xacro").getroot()
        mounts = root.findall("xacro:zed_x_camera", ns)
        self.assertEqual(len(mounts), 4)
        for mount in mounts:
            for field in ("x", "y", "z", "roll", "pitch", "yaw"):
                self.assertEqual(mount.get(field), "0")
        lidar = root.find("xacro:ouster_os1", ns)
        self.assertEqual((lidar.get("roll"), lidar.get("pitch"), lidar.get("yaw")), ("0", "0.0698", "0.2150"))

    def test_pair_rejects_stale_and_unsynchronized_sensor_messages(self):
        lidar, camera = message(), message("zed_front_optical")
        config = calibration.DEFAULT_CONFIG
        self.assertIsNone(calibration.pair_error(lidar, (camera, 10.0), 10.5, config))
        self.assertEqual(calibration.pair_error(lidar, (camera, 10.0), 13.0, config), "camera cloud is stale")
        camera.header.stamp.sec = 102
        self.assertIn("timestamp difference", calibration.pair_error(lidar, (camera, 10.0), 10.5, config))

    def test_urdf_acceptance_commits_poses_and_failure_does_not(self):
        for successful in (True, False):
            with self.subTest(successful=successful), tempfile.TemporaryDirectory() as directory:
                future = Mock()
                future.result.return_value.results = [SimpleNamespace(successful=successful, reason="rejected")]
                node = SimpleNamespace(
                    pending_update=future, state_lock=Lock(), robot_description=URDF,
                    camera_poses={}, camera_anchors={}, camera_mounts={}, camera_failures={"front": 1},
                    retry_global={"front"}, update_in_flight=True, pending_candidates={"front": "confirmed"},
                    output_urdf=Path(directory) / "calibrated.urdf", get_logger=Mock(),
                )
                updated = {"front": ("os_lidar", "zed_front_optical", np.eye(4))}
                calibration.LidarCameraRegistration.description_update_done(
                    node, future, "updated vehicle urdf", updated, {"front": np.eye(4)}, {"front"})
                self.assertFalse(node.update_in_flight)
                self.assertEqual("front" in node.camera_poses, successful)
                self.assertEqual("front" in node.camera_anchors, successful)
                self.assertEqual("front" in node.pending_candidates, not successful)
                self.assertEqual(node.output_urdf.exists(), successful)
                if successful:
                    self.assertEqual(node.output_urdf.read_text(), "updated vehicle urdf")
                    self.assertNotIn("front", node.retry_global)

    def test_success_is_logged_for_new_global_pose_but_not_periodic_icp(self):
        with tempfile.TemporaryDirectory() as directory:
            node = SimpleNamespace(
                pending_update=None, state_lock=Lock(), robot_description=URDF,
                camera_poses={}, camera_anchors={}, camera_mounts={}, camera_failures={},
                retry_global=set(), update_in_flight=True, pending_candidates={},
                output_urdf=Path(directory) / "calibrated.urdf", get_logger=Mock(),
            )
            updated = {"front": ("os_lidar", "zed_front_optical", np.eye(4))}
            for confirmed in ({"front"}, set(), {"front"}):
                future = Mock()
                future.result.return_value.results = [SimpleNamespace(successful=True)]
                node.pending_update = future
                calibration.LidarCameraRegistration.description_update_done(
                    node, future, "updated vehicle urdf", updated, {"front": np.eye(4)}, confirmed)
            logger = node.get_logger.return_value
            self.assertEqual(logger.info.call_count, 2)  # Initial calibration and global retry.
            self.assertEqual(logger.debug.call_count, 3)  # Pose details remain available at DEBUG.

    def test_first_attempt_is_immediate_and_next_attempt_waits_ten_seconds(self):
        lidar, camera = message(), message("zed_front_optical")
        node = SimpleNamespace(
            next_calibration=0.0, config=calibration.DEFAULT_CONFIG,
            data_lock=Lock(), state_lock=Lock(), latest_lidar=(lidar, 100.0),
            camera_clouds={"front": (camera, 100.0)}, robot_description=URDF,
            camera_poses={}, camera_mounts={}, retry_global=set(),
            update_in_flight=False, pending_update=None, update_started=0.0,
            description_client=Mock(), get_logger=Mock(), calibrate=Mock(),
            prepare_calibration_input=Mock(), publish_registered_inputs=Mock(),
        )
        node.description_client.service_is_ready.return_value = True
        cloud = Mock()
        cloud.points = np.ones((40, 3))
        node.prepare_calibration_input.return_value = (cloud, cloud)
        with patch.object(calibration.time, "monotonic", return_value=100.0):
            calibration.LidarCameraRegistration.calibration_tick(node)
        self.assertEqual(node.calibrate.call_count, 1)
        self.assertEqual(node.next_calibration, 110.0)
        with patch.object(calibration.time, "monotonic", return_value=105.0):
            calibration.LidarCameraRegistration.calibration_tick(node)
        self.assertEqual(node.calibrate.call_count, 1)
        node.latest_lidar = (lidar, 110.0)
        node.camera_clouds = {"front": (camera, 110.0)}
        with patch.object(calibration.time, "monotonic", return_value=110.0):
            calibration.LidarCameraRegistration.calibration_tick(node)
        self.assertEqual(node.calibrate.call_count, 2)


if __name__ == "__main__":
    unittest.main()
