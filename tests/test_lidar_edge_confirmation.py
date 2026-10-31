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

    def test_shared_patch_voxels_get_centers_and_interpolated_points(self):
        pcd = cloud([
            [.02, .02, .02], [.03, .02, .02], [.03, .03, .02], [.03, .04, .02],
            [.21, .21, .21], [.26, .02, .02], [.27, .02, .02],
            [1.02, .02, .02], [1.03, .02, .02], [1.03, .03, .02], [1.03, .04, .02],
        ])
        patches = [Patch([0, 4, 5], [0, 0, 1]), Patch([1, 6], [1, 0, 0])]
        groups = detector.detect_lidar_edge_points(pcd, patches)

        original_points = np.asarray(pcd.points).copy()
        self.assertEqual(detector.color_edge_points(pcd, groups), 11)
        np.testing.assert_allclose(np.asarray(pcd.points)[:11], original_points)
        np.testing.assert_allclose(np.asarray(pcd.points)[11:13],
                                   [[.125, .125, .125], [.375, .125, .125]])
        blue_points = np.asarray(pcd.points)[11:]
        sorted_x = np.sort(blue_points[:, 0])
        np.testing.assert_allclose(np.diff(sorted_x), detector.EDGE_POINT_SPACING)
        np.testing.assert_allclose(blue_points[:, 1:], np.full((11, 2), .125))
        colors = np.asarray(pcd.colors)
        np.testing.assert_array_equal(colors[:11], np.zeros((11, 3)))
        np.testing.assert_allclose(colors[11:], np.tile([0.0, 0.65, 1.0], (11, 1)))

        msg = detector.pack_colored_cloud(pcd, Header(frame_id='os_sensor'))
        self.assertEqual(msg.header.frame_id, 'os_sensor')
        self.assertEqual(msg.width, 22)
        packed_xyz = np.frombuffer(msg.data, dtype='<f4').reshape(-1, 4)[:, :3]
        np.testing.assert_allclose(packed_xyz[11:13],
                                   [[.125, .125, .125], [.375, .125, .125]])
        packed_rgb = np.frombuffer(msg.data, dtype='<u4').reshape(-1, 4)[:, 3]
        np.testing.assert_array_equal(packed_rgb[[0, 11, 21]], [0x000000, 0x00a5ff, 0x00a5ff])

    def test_overlapping_groups_and_negative_voxels_have_unique_centers(self):
        pcd = cloud([[0, 0, 0]])
        groups = [{(-1, 0, 2), (0, 0, 0)}, {(0, 0, 0), (1, -1, 0)}]
        steps_a = int(np.ceil(np.sqrt(5) * detector.EDGE_VOXEL_SIZE / detector.EDGE_POINT_SPACING))
        steps_b = int(np.ceil(np.sqrt(2) * detector.EDGE_VOXEL_SIZE / detector.EDGE_POINT_SPACING))
        self.assertEqual(detector.color_edge_points(pcd, groups), 3 + steps_a + steps_b - 2)
        np.testing.assert_allclose(np.asarray(pcd.points)[1:4], [
            [-.125, .125, .625], [.125, .125, .125], [.375, -.125, .125],
        ])

    def test_separated_patches_still_match_within_large_voxels(self):
        pcd = cloud([[.02, .05, .02], [.27, .05, .02],
                     [.02, .15, .02], [.27, .15, .02]])
        patches = [Patch([0, 1], [0, 0, 1]), Patch([2, 3], [1, 0, 0])]
        groups = detector.detect_lidar_edge_points(pcd, patches)
        self.assertEqual(groups, [{(0, 0, 0), (1, 0, 0)}])
        self.assertEqual(detector.color_edge_points(pcd, groups), 11)
        np.testing.assert_allclose(np.asarray(pcd.points)[4:6],
                                   [[.125, .125, .125], [.375, .125, .125]])

    def test_diagonal_neighbors_get_points_no_farther_apart_than_spacing(self):
        points = detector.interpolate_edge_voxels([{(0, 0, 0), (1, 1, 1)}])
        points = points[np.argsort(points[:, 0])]
        self.assertEqual(len(points), 19)
        np.testing.assert_allclose(points[0], [.125, .125, .125])
        np.testing.assert_allclose(points[-1], [.375, .375, .375])
        self.assertLessEqual(np.max(np.linalg.norm(np.diff(points, axis=0), axis=1)),
                             detector.EDGE_POINT_SPACING + 1e-12)

    def test_neighboring_centers_from_separate_patch_pairs_are_not_connected(self):
        groups = [{(0, 0, 0), (1, 0, 0)}, {(2, 0, 0), (3, 0, 0)}]
        points = detector.interpolate_edge_voxels(groups)
        self.assertEqual(len(points), 22)
        self.assertFalse(np.any((points[:, 0] > .375) & (points[:, 0] < .625)))

    def test_nonadjacent_voxels_in_one_group_connect_to_the_nearest(self):
        points = detector.interpolate_edge_voxels([{(0, 0, 0), (2, 0, 0)}])
        points = points[np.argsort(points[:, 0])]
        self.assertEqual(len(points), 21)
        np.testing.assert_allclose(points[0], [.125, .125, .125])
        np.testing.assert_allclose(points[-1], [.625, .125, .125])
        np.testing.assert_allclose(np.diff(points[:, 0]), detector.EDGE_POINT_SPACING)

    def test_dense_group_uses_only_nearest_connections(self):
        group = {(0, 0, 0), (0, 1, 0), (1, 0, 0), (1, 1, 0)}
        connections = detector.nearest_edge_connections([group])
        self.assertEqual(connections, {
            ((0, 0, 0), (0, 1, 0)),
            ((0, 0, 0), (1, 0, 0)),
            ((0, 1, 0), (1, 1, 0)),
        })
        self.assertEqual(len(detector.interpolate_edge_voxels([group])), 31)

    def test_sparse_group_does_not_connect_every_pair(self):
        group = {(0, 0, 0), (2, 0, 0), (5, 0, 0)}
        self.assertEqual(detector.nearest_edge_connections([group]), {
            ((0, 0, 0), (2, 0, 0)),
            ((2, 0, 0), (5, 0, 0)),
        })

    def test_sparse_equal_distance_ties_are_stable(self):
        group = {(-4, 0, 0), (-3, 0, 0), (0, 0, 0), (3, 0, 0), (4, 0, 0)}
        self.assertEqual(detector.nearest_edge_connections([group]), {
            ((-4, 0, 0), (-3, 0, 0)),
            ((-3, 0, 0), (0, 0, 0)),
            ((3, 0, 0), (4, 0, 0)),
        })

    def test_single_voxel_has_no_connection(self):
        self.assertEqual(detector.nearest_edge_connections([{(0, 0, 0)}]), set())
        np.testing.assert_allclose(detector.interpolate_edge_voxels([{(0, 0, 0)}]),
                                   [[.125, .125, .125]])

    def test_nearest_connections_match_brute_force_for_dense_and_sparse_groups(self):
        rng = np.random.default_rng(41)
        for radius, count in [(2, 30), (20, 60)]:
            group = set()
            while len(group) < count:
                group.add(tuple(int(v) for v in rng.integers(-radius, radius + 1, 3)))
            expected = set()
            for voxel in group:
                neighbor = min(
                    (sum((a - b) ** 2 for a, b in zip(voxel, other)), other)
                    for other in group if other != voxel
                )[1]
                expected.add(tuple(sorted((voxel, neighbor))))
            self.assertEqual(detector.nearest_edge_connections([group]), expected)

    def test_repeated_patch_pair_does_not_duplicate_connections(self):
        group = {(0, 0, 0), (1, 0, 0)}
        points = detector.interpolate_edge_voxels([group, group])
        self.assertEqual(len(points), 11)
        self.assertEqual(len(np.unique(points, axis=0)), 11)

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
