"""Exercise the combined lidar edge checks with small Open3D clouds."""

import importlib.util
from pathlib import Path
import unittest

import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


DETECTOR = (Path(__file__).resolve().parents[1] / 'ros2_ws/src/lidar_perception'
            / 'corner_detector.py')
spec = importlib.util.spec_from_file_location('lidar_edge_detector_test', DETECTOR)
detector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(detector)


def cloud(points, normals):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    pcd.normals = o3d.utility.Vector3dVector(np.asarray(normals, dtype=float))
    return pcd


class Patch:
    def __init__(self, indices, normal):
        self.indices = indices
        self.R = np.eye(3)
        self.R[:, 2] = normal

    def get_point_indices_within_bounding_box(self, points):
        return self.indices


class LidarEdgeConfirmationTest(unittest.TestCase):
    def test_two_synthetic_planes_produce_confirmed_points_near_their_edge(self):
        rng = np.random.default_rng(12)
        x = np.linspace(-1.2, 1.2, 80)
        y = np.linspace(0, 1.2, 40)
        a, b = np.meshgrid(x, y, indexing='ij')
        xyz = np.vstack([
            np.column_stack((a.ravel(), b.ravel(), np.zeros(a.size))),
            np.column_stack((a.ravel(), np.zeros(a.size), b.ravel())),
        ])
        xyz += rng.normal(0, .0005, xyz.shape)
        msg = point_cloud2.create_cloud_xyz32(
            Header(frame_id='os_sensor'), xyz.astype('float32').tolist()
        )

        pcd, patches = detector.find_patches(msg)
        groups = detector.detect_lidar_edge_points(pcd, patches)
        count = detector.color_edge_points(pcd, groups)
        self.assertGreater(count, 0)
        self.assertLess(count, len(pcd.points))
        edge_points = np.asarray(pcd.points)[np.asarray(pcd.colors)[:, 2] > .9]
        # Both surfaces meet along the x axis, where y and z are close to zero.
        self.assertLess(np.max(np.linalg.norm(edge_points[:, 1:3], axis=1)), .15)

    def test_only_confirmed_points_in_shared_patch_voxels_are_blue(self):
        pcd = cloud(
            [[.02, .02, .02], [.03, .02, .02], [.03, .03, .02], [.03, .04, .02],
             [.21, .21, .21], [.26, .02, .02], [.27, .02, .02],
             [1.02, .02, .02], [1.03, .02, .02], [1.03, .03, .02], [1.03, .04, .02]],
            [[0, 0, 1], [1, 0, 0], [1, 0, 0], [1, 0, 0],
             [0, 0, 1], [0, 0, 1], [0, 0, 1],
             [0, 0, 1], [1, 0, 0], [1, 0, 0], [1, 0, 0]],
        )
        patches = [Patch([0, 4, 5], [0, 0, 1]), Patch([1, 6], [1, 0, 0])]
        groups = detector.detect_lidar_edge_points(pcd, patches)

        # Point 7 has a normal-based edge neighborhood but no matching patch voxel.
        point_candidates = detector.detect_point_normal_edges(
            pcd, np.ones(len(pcd.points), dtype=bool)
        )
        self.assertTrue(point_candidates[7])

        self.assertEqual(detector.color_edge_points(pcd, groups), 1)
        colors = np.asarray(pcd.colors)
        np.testing.assert_allclose(colors[0], [0.0, 0.65, 1.0])
        np.testing.assert_array_equal(colors[1:], np.zeros((len(colors) - 1, 3)))
        # Point 4 shares a patch voxel with point 0 but lacks a normal edge.
        self.assertFalse(point_candidates[4])

    def test_opposite_normal_directions_do_not_create_point_edges(self):
        pcd = cloud([[0, 0, 0], [.01, 0, 0], [.02, 0, 0], [.03, 0, 0]],
                    [[0, 0, 2], [0, 0, -1], [0, 0, -3], [0, 0, -0.1]])
        mask = detector.detect_point_normal_edges(pcd, np.ones(4, dtype=bool))
        self.assertFalse(mask.any())

    def test_neighbors_across_voxel_boundary_still_support_candidate(self):
        pcd = cloud([[.249, 0, 0], [.26, 0, 0], [.27, 0, 0], [.28, 0, 0]],
                    [[0, 0, 1], [1, 0, 0], [1, 0, 0], [1, 0, 0]])
        mask = detector.detect_point_normal_edges(pcd, np.array([True, False, False, False]))
        np.testing.assert_array_equal(mask, [True, False, False, False])

    def test_opposite_patch_normals_do_not_create_patch_edges(self):
        pcd = cloud([[.01, 0, 0], [.26, 0, 0]], [[0, 0, 1], [0, 0, -1]])
        patches = [Patch([0, 1], [0, 0, 1]), Patch([0, 1], [0, 0, -1])]
        self.assertEqual(detector.detect_lidar_edge_points(pcd, patches), [])

    def test_requires_three_different_neighbors_and_respects_radius(self):
        pcd = cloud([[0, 0, 0], [.01, 0, 0], [.02, 0, 0], [.11, 0, 0]],
                    [[0, 0, 1], [1, 0, 0], [1, 0, 0], [1, 0, 0]])
        mask = detector.detect_point_normal_edges(pcd, np.array([True, False, False, False]))
        self.assertFalse(mask.any())

    def test_invalid_normals_are_not_counted(self):
        pcd = cloud([[0, 0, 0], [.01, 0, 0], [.02, 0, 0], [.03, 0, 0]],
                    [[0, 0, 1], [0, 0, 0], [np.nan, 0, 0], [1, 0, 0]])
        mask = detector.detect_point_normal_edges(pcd, np.ones(4, dtype=bool))
        self.assertFalse(mask.any())

    def test_no_patch_edge_candidates_leaves_all_points_black(self):
        pcd = cloud([[0, 0, 0], [.01, 0, 0]], [[0, 0, 1], [1, 0, 0]])
        self.assertEqual(detector.color_edge_points(pcd, []), 0)
        np.testing.assert_array_equal(np.asarray(pcd.colors), np.zeros((2, 3)))


if __name__ == '__main__':
    unittest.main()
