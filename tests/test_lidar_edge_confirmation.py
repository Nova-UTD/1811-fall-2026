"""Exercise patch-based lidar edge detection with small Open3D clouds."""

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


def cloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    return pcd


class Patch:
    def __init__(self, indices, normal):
        self.indices = indices
        self.R = np.eye(3)
        self.R[:, 2] = normal

    def get_point_indices_within_bounding_box(self, points):
        return self.indices


class LidarPatchEdgeTest(unittest.TestCase):
    def test_two_synthetic_planes_produce_patch_edges(self):
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
        # Both surfaces meet along x; 25 cm voxels allow a broad candidate band.
        self.assertLess(np.max(np.linalg.norm(edge_points[:, 1:3], axis=1)), .36)

    def test_shared_patch_voxels_get_one_blue_center_each(self):
        pcd = cloud([
            [.02, .02, .02], [.03, .02, .02], [.03, .03, .02], [.03, .04, .02],
            [.21, .21, .21], [.26, .02, .02], [.27, .02, .02],
            [1.02, .02, .02], [1.03, .02, .02], [1.03, .03, .02], [1.03, .04, .02],
        ])
        patches = [Patch([0, 4, 5], [0, 0, 1]), Patch([1, 6], [1, 0, 0])]
        groups = detector.detect_lidar_edge_points(pcd, patches)

        original_points = np.asarray(pcd.points).copy()
        self.assertEqual(detector.color_edge_points(pcd, groups), 2)
        np.testing.assert_allclose(np.asarray(pcd.points)[:11], original_points)
        np.testing.assert_allclose(np.asarray(pcd.points)[11:],
                                   [[.125, .125, .125], [.375, .125, .125]])
        colors = np.asarray(pcd.colors)
        np.testing.assert_array_equal(colors[:11], np.zeros((11, 3)))
        np.testing.assert_allclose(colors[11:], np.tile([0.0, 0.65, 1.0], (2, 1)))

        msg = detector.pack_colored_cloud(pcd, Header(frame_id='os_sensor'))
        self.assertEqual(msg.header.frame_id, 'os_sensor')
        self.assertEqual(msg.width, 13)
        packed_xyz = np.frombuffer(msg.data, dtype='<f4').reshape(-1, 4)[:, :3]
        np.testing.assert_allclose(packed_xyz[11:],
                                   [[.125, .125, .125], [.375, .125, .125]])
        packed_rgb = np.frombuffer(msg.data, dtype='<u4').reshape(-1, 4)[:, 3]
        np.testing.assert_array_equal(packed_rgb[[0, 11, 12]], [0x000000, 0x00a5ff, 0x00a5ff])

    def test_overlapping_groups_and_negative_voxels_have_unique_centers(self):
        pcd = cloud([[0, 0, 0]])
        groups = [{(-1, 0, 2), (0, 0, 0)}, {(0, 0, 0), (1, -1, 0)}]
        self.assertEqual(detector.color_edge_points(pcd, groups), 3)
        np.testing.assert_allclose(np.asarray(pcd.points)[1:], [
            [-.125, .125, .625], [.125, .125, .125], [.375, -.125, .125],
        ])

    def test_separated_patches_still_match_within_large_voxels(self):
        pcd = cloud([[.02, .05, .02], [.27, .05, .02],
                     [.02, .15, .02], [.27, .15, .02]])
        patches = [Patch([0, 1], [0, 0, 1]), Patch([2, 3], [1, 0, 0])]
        groups = detector.detect_lidar_edge_points(pcd, patches)
        self.assertEqual(groups, [{(0, 0, 0), (1, 0, 0)}])
        self.assertEqual(detector.color_edge_points(pcd, groups), 2)
        np.testing.assert_allclose(np.asarray(pcd.points)[4:],
                                   [[.125, .125, .125], [.375, .125, .125]])

    def test_opposite_patch_normals_do_not_create_patch_edges(self):
        pcd = cloud([[.01, 0, 0], [.26, 0, 0]])
        patches = [Patch([0, 1], [0, 0, 1]), Patch([0, 1], [0, 0, -1])]
        self.assertEqual(detector.detect_lidar_edge_points(pcd, patches), [])

    def test_patch_pair_requires_two_shared_voxels(self):
        pcd = cloud([[.01, 0, 0], [.26, 0, 0]])
        patch_a = Patch([0, 1], [0, 0, 1])
        one_shared = Patch([0], [1, 0, 0])
        two_shared = Patch([0, 1], [1, 0, 0])
        self.assertEqual(detector.detect_lidar_edge_points(pcd, [patch_a, one_shared]), [])
        groups = detector.detect_lidar_edge_points(pcd, [patch_a, two_shared])
        self.assertEqual(groups, [{(0, 0, 0), (1, 0, 0)}])

    def test_no_candidate_voxels_leaves_all_points_black(self):
        pcd = cloud([[0, 0, 0], [.01, 0, 0]])
        self.assertEqual(detector.color_edge_points(pcd, []), 0)
        np.testing.assert_array_equal(np.asarray(pcd.colors), np.zeros((2, 3)))


if __name__ == '__main__':
    unittest.main()
