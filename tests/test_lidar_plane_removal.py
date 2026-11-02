"""Exercise planar-patch removal with Open3D and ROS point clouds."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock

import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


PACKAGE = Path(__file__).resolve().parents[1] / 'ros2_ws/src/lidar_perception'


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


detector = load_module('lidar_plane_detector_test', 'lidar_camera_registration.py')
viewer = load_module('lidar_plane_viewer_test', 'corner_viewer.py')


def point_cloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float).reshape(-1, 3))
    return pcd


def ros_cloud(points, frame='os_lidar'):
    return point_cloud2.create_cloud_xyz32(
        Header(frame_id=frame), np.asarray(points, dtype=np.float32).reshape(-1, 3).tolist()
    )


@unittest.skip('Planar-patch removal was removed from the detector')
class LidarPlaneRemovalTest(unittest.TestCase):
    def test_real_patch_detection_removes_noisy_ground_and_wall(self):
        rng = np.random.default_rng(12)
        a, b = np.meshgrid(np.linspace(-1.5, 1.5, 80), np.linspace(-1.5, 1.5, 80))
        ground = np.column_stack((a.ravel(), b.ravel(), rng.normal(0, 0.003, a.size)))
        a, b = np.meshgrid(np.linspace(-1.5, 1.5, 80), np.linspace(0, 2, 60))
        wall = np.column_stack((2 + rng.normal(0, 0.003, a.size), a.ravel(), b.ravel()))
        obstacle = rng.uniform([-0.18, -0.18, 0.75], [0.18, 0.18, 1.1], (400, 3))

        points = np.vstack((ground, wall, obstacle))
        for degrees in (0, 35):
            with self.subTest(sensor_tilt_degrees=degrees):
                angle = np.deg2rad(degrees)
                rotation = np.array([[np.cos(angle), 0, np.sin(angle)],
                                     [0, 1, 0],
                                     [-np.sin(angle), 0, np.cos(angle)]])
                pcd, patches = detector.find_patches(ros_cloud(points @ rotation.T))
                remaining = np.asarray(detector.remove_planar_patches(pcd, patches).points)
                remaining = remaining @ rotation

                self.assertGreaterEqual(len(patches), 2)
                self.assertLess(np.count_nonzero(np.abs(remaining[:, 2]) < 0.02),
                                len(ground) * 0.05)
                self.assertLess(np.count_nonzero(np.abs(remaining[:, 0] - 2) < 0.02),
                                len(wall) * 0.05)
                obstacle_remaining = ((np.abs(remaining[:, 0]) < 0.2)
                                      & (np.abs(remaining[:, 1]) < 0.2)
                                      & (remaining[:, 2] > 0.7))
                self.assertGreater(np.count_nonzero(obstacle_remaining), len(obstacle) * 0.9)

    def test_rotated_patch_removal_is_limited_to_its_box_and_tolerance(self):
        angle = np.deg2rad(35)
        rotation = np.array([[np.cos(angle), 0, np.sin(angle)],
                             [0, 1, 0],
                             [-np.sin(angle), 0, np.cos(angle)]])
        center = np.array([2.0, -1.0, 3.0])
        local_points = np.array([[0, 0, 0.02], [0.5, 0.5, -0.04], [1.01, 0, 0],
                                 [0.5, 0.5, 0.06], [1.02, 0, 0], [0, 0, -0.2]])
        points = local_points @ rotation.T + center
        patch = o3d.geometry.OrientedBoundingBox(center, rotation, [2, 2, 0.01])

        remaining = detector.remove_planar_patches(point_cloud(points), [patch, patch])
        np.testing.assert_allclose(np.asarray(remaining.points), points[3:])
        # Filtering does not modify the detector's original patch geometry.
        np.testing.assert_allclose(patch.extent, [2, 2, 0.01])

    def test_measured_patch_thickness_is_not_reduced(self):
        patch = o3d.geometry.OrientedBoundingBox([0, 0, 0], np.eye(3), [2, 2, 0.16])
        pcd = point_cloud([[0, 0, 0.075], [0, 0, 0.09]])
        remaining = detector.remove_planar_patches(pcd, [patch])
        np.testing.assert_allclose(np.asarray(remaining.points), [[0, 0, 0.09]])

    def test_small_cloud_without_patches_keeps_finite_points(self):
        msg = ros_cloud([[1, 2, 3], [4, 5, 6], [float('nan'), 1, 2],
                         [1, float('inf'), 2]])
        pcd, patches = detector.find_patches(msg)
        self.assertEqual(patches, [])
        remaining = detector.remove_planar_patches(pcd, patches)
        np.testing.assert_allclose(np.asarray(remaining.points), [[1, 2, 3], [4, 5, 6]])

    def test_cloud_round_trip_preserves_frame_time_and_empty_frames(self):
        header = Header(frame_id='os_lidar')
        header.stamp.sec = 123
        header.stamp.nanosec = 456
        for points in ([[1, 2, 3], [-1, -2, -3]], []):
            packed = detector.pack_filtered_cloud(point_cloud(points), header)
            self.assertEqual(packed.header, header)
            self.assertEqual(packed.width, len(points))
            self.assertEqual(packed.point_step, 12)
            self.assertEqual(packed.row_step, 12 * len(points))
            np.testing.assert_allclose(viewer.unpack_filtered_cloud(packed),
                                       np.asarray(points).reshape(-1, 3))

    def test_callback_publishes_empty_frame(self):
        node = Mock()
        detector.LidarCameraRegistration.pcd_callback(node, ros_cloud([]))
        node.filtered_pub.publish.assert_called_once()
        self.assertEqual(node.filtered_pub.publish.call_args.args[0].width, 0)
        node.get_logger.return_value.error.assert_not_called()

    def test_viewer_clears_an_empty_frame_and_accepts_a_later_cloud(self):
        visualizer = viewer.CloudVisualizer.__new__(viewer.CloudVisualizer)
        visualizer.o3d = o3d
        visualizer.vis = Mock()
        visualizer.pcd = point_cloud([])
        visualizer.added = False

        visualizer.show_cloud(np.array([[1, 2, 3]]))
        visualizer.show_cloud(np.empty((0, 3)))
        self.assertFalse(visualizer.added)
        visualizer.vis.remove_geometry.assert_called_once()
        visualizer.show_cloud(np.array([[4, 5, 6]]))
        self.assertTrue(visualizer.added)
        self.assertEqual(visualizer.vis.add_geometry.call_count, 2)

    def test_viewer_rejects_invalid_cloud_layout(self):
        msg = detector.pack_filtered_cloud(point_cloud([[1, 2, 3]]), Header())
        msg.point_step = 16
        with self.assertRaises(ValueError):
            viewer.unpack_filtered_cloud(msg)


if __name__ == '__main__':
    unittest.main()
