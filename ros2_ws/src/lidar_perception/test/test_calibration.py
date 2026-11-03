"""Checks for sensor-frame composition, URDF updates, and calibration scheduling."""
import importlib.util
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2
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


class CalibrationTest(unittest.TestCase):
    def test_registered_display_requires_a_usable_camera_and_preserves_lidar(self):
        lidar_header = Header(frame_id="os_lidar")
        lidar_header.stamp.sec = 100
        lidar = point_cloud2.create_cloud_xyz32(lidar_header, [[1, 2, 3], [4, 5, 6]])
        original_lidar = bytes(lidar.data)
        camera_header = Header(frame_id="zed_front_left_camera_frame")
        camera_header.stamp.sec = 100
        camera = point_cloud2.create_cloud_xyz32(camera_header, [[0.25, -0.5, 1]])
        node = SimpleNamespace(
            data_lock=Lock(), state_lock=Lock(), latest_lidar=None,
            camera_clouds={"front": (camera, 100.0)}, camera_poses={},
            config=calibration.DEFAULT_CONFIG, registered_pub=Mock(), get_logger=Mock(),
        )

        # A received camera is not a registered camera until its pose is accepted.
        with patch.object(calibration.time, "monotonic", return_value=100.5):
            calibration.LidarCameraRegistration.pcd_callback(node, lidar)
        output = node.registered_pub.publish.call_args.args[0]
        self.assertEqual(output.width, 0)
        self.assertEqual(output.header.frame_id, "os_lidar")

        pose = np.eye(4)
        pose[0, 3] = 2.0
        node.camera_poses["front"] = ("os_lidar", camera_header.frame_id, pose)
        with patch.object(calibration.time, "monotonic", return_value=100.5):
            calibration.LidarCameraRegistration.pcd_callback(node, lidar)
        output = node.registered_pub.publish.call_args.args[0]
        points = point_cloud2.read_points(output, field_names=("x", "y", "z", "rgb"))
        lidar_color = (230 << 16) | (230 << 8) | 230
        camera_color = (40 << 16) | (160 << 8) | 255
        lidar_points = points[points["rgb"] == lidar_color]
        camera_points = points[points["rgb"] == camera_color]
        self.assertEqual({(p["x"], p["y"], p["z"]) for p in lidar_points},
                         {(1, 2, 3), (4, 5, 6)})
        self.assertEqual(len(camera_points), 1)
        np.testing.assert_allclose([camera_points[0]["x"], camera_points[0]["y"],
                                    camera_points[0]["z"]], [2.25, -0.5, 1])
        self.assertEqual(bytes(lidar.data), original_lidar)

        # A stale camera must clear the previously displayed result.
        with patch.object(calibration.time, "monotonic", return_value=103.0):
            calibration.LidarCameraRegistration.pcd_callback(node, lidar)
        self.assertEqual(node.registered_pub.publish.call_args.args[0].width, 0)
        node.get_logger.return_value.error.assert_not_called()

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
                    camera_poses={}, camera_mounts={}, camera_failures={"front": 1},
                    retry_global={"front"}, update_in_flight=True,
                    output_urdf=Path(directory) / "calibrated.urdf", get_logger=Mock(),
                )
                updated = {"front": ("os_lidar", "zed_front_optical", np.eye(4))}
                calibration.LidarCameraRegistration.description_update_done(
                    node, future, "updated vehicle urdf", updated, {"front": np.eye(4)})
                self.assertFalse(node.update_in_flight)
                self.assertEqual("front" in node.camera_poses, successful)
                self.assertEqual(node.output_urdf.exists(), successful)
                if successful:
                    self.assertEqual(node.output_urdf.read_text(), "updated vehicle urdf")
                    self.assertNotIn("front", node.retry_global)

    def test_first_attempt_is_immediate_and_next_attempt_waits_ten_seconds(self):
        lidar, camera = message(), message("zed_front_optical")
        node = SimpleNamespace(
            next_calibration=0.0, config=calibration.DEFAULT_CONFIG,
            data_lock=Lock(), state_lock=Lock(), latest_lidar=(lidar, 100.0),
            camera_clouds={"front": (camera, 100.0)}, robot_description=URDF,
            camera_poses={}, camera_mounts={}, retry_global=set(),
            update_in_flight=False, pending_update=None, update_started=0.0,
            description_client=Mock(), get_logger=Mock(), calibrate=Mock(),
        )
        node.description_client.service_is_ready.return_value = True
        cloud = Mock()
        cloud.voxel_down_sample.return_value = cloud
        with patch.object(calibration, "point_cloud_from_message", return_value=cloud), \
                patch.object(calibration.time, "monotonic", return_value=100.0):
            calibration.LidarCameraRegistration.calibration_tick(node)
        self.assertEqual(node.calibrate.call_count, 1)
        self.assertEqual(node.next_calibration, 110.0)
        with patch.object(calibration.time, "monotonic", return_value=105.0):
            calibration.LidarCameraRegistration.calibration_tick(node)
        self.assertEqual(node.calibrate.call_count, 1)
        node.latest_lidar = (lidar, 110.0)
        node.camera_clouds = {"front": (camera, 110.0)}
        with patch.object(calibration, "point_cloud_from_message", return_value=cloud), \
                patch.object(calibration.time, "monotonic", return_value=110.0):
            calibration.LidarCameraRegistration.calibration_tick(node)
        self.assertEqual(node.calibrate.call_count, 2)


if __name__ == "__main__":
    unittest.main()
