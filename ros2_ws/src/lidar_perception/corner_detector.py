#!/usr/bin/env python3
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d

EDGE_VOXEL_SIZE = 0.05


def find_patches(msg):
    # Extract the point cloud data into Nx3 numpy array
    cloud = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )
    xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    #   xyz = [
    #   [x1, y1, z1],
    #   [x2, y2, z2],
    #   [x3, y3, z3],
    #   ...
    #   [xn, yn, zn],
    #   ]

    # If the point cloud has less than 30 points, return None and an empty list
    if len(xyz) < 30:
        return None, []

    # Initialize Open3D point cloud with xyz coordinates
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))

    # Downsample the point cloud to reduce the number of points
    pcd = pcd.voxel_down_sample(voxel_size=0.03)
    if len(pcd.points) < 30:
        return pcd, []

    # Planar patch detection requires a normal for each point.
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.25,  # 25cm starting value
            max_nn=30,
        )
    )

    # Detect planar patches
    patches = pcd.detect_planar_patches()

    # Return the downsampled point cloud and the detected patches
    return pcd, patches

def detect_edge_points(pcd,patches):
    voxel_size = EDGE_VOXEL_SIZE

    # Assign every point in the cloud to a voxel.
    points = np.asarray(pcd.points)
    voxel_ids = np.floor(points / voxel_size).astype(np.int64)

    # patch_voxels = [
    #   {voxel_id1, voxel_id2, voxel_id3, ...}, -> patch 1
    #   {voxel_id4, voxel_id5, voxel_id6, ...}, -> patch 2
    #   {voxel_id7, voxel_id8, voxel_id9, ...}, -> patch 3
    #   ...
    # ]
    patches_voxels = []
    for patch in patches:
        indices = patch.get_point_indices_within_bounding_box(pcd.points)
        patch_voxels = set(map(tuple,voxel_ids[indices]))
        patches_voxels.append(patch_voxels)

    edge_points = []
    for i in range(len(patches)):
        for j in range(i+1, len(patches)):
            patch_a = patches[i]
            patch_b = patches[j]

            normal_patch_a = np.asarray(patch_a.R)[:, 2]
            normal_patch_b = np.asarray(patch_b.R)[:, 2]

            # If the patches are parallel, skip
            if abs(np.dot(normal_patch_a, normal_patch_b)) >= 0.9:
                continue

            shared_voxels = patches_voxels[i] & patches_voxels[j]

            if len(shared_voxels) < 2:
                continue

            edge_points.append(shared_voxels)

    return edge_points

class EdgeVisualizer:
    # Display the point cloud with detected edge points in an Open3D window.

    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        if not self.vis.create_window(
            window_name="Lidar edge points", width=1280, height=720
        ):
            raise RuntimeError("Could not open the Open3D window; check DISPLAY and X11 access")

        options = self.vis.get_render_option()
        options.background_color = np.array([0.05, 0.05, 0.05])
        options.point_size = 2.0
        options.light_on = False
        self.view_fitted = False

    def show_edges(self, pcd, edge_voxel_groups):
        # Color the existing points in detected voxels neon blue.
        pcd.paint_uniform_color([0.6, 0.6, 0.6])
        edge_voxels = set().union(*edge_voxel_groups)
        edge_count = 0
        if edge_voxels:
            points = np.asarray(pcd.points)
            voxel_ids = np.floor(points / EDGE_VOXEL_SIZE).astype(np.int64)
            edge_mask = np.fromiter(
                (tuple(voxel) in edge_voxels for voxel in voxel_ids),
                dtype=bool,
                count=len(voxel_ids),
            )
            np.asarray(pcd.colors)[edge_mask] = [0.0, 0.65, 1.0]
            edge_count = np.count_nonzero(edge_mask)

        self.vis.clear_geometries()
        self.vis.add_geometry(pcd, reset_bounding_box=not self.view_fitted)
        self.view_fitted = True
        self.vis.update_renderer()
        return edge_count

    def poll_events(self):
        return self.vis.poll_events()

    def close(self):
        self.vis.destroy_window()


class CornerDetector(Node):
    def __init__(self):
        super().__init__("corner_detector")
        self.visualizer = EdgeVisualizer()
        self.interval = 2.0  # Seconds between updates: at most 0.5 Hz.
        self.next_update = 0.0
        self.pcd_sub = self.create_subscription(
            PointCloud2, "/ouster/points", self.pcd_callback, qos_profile_sensor_data
        )
        self.get_logger().info("Corner detector started, listening to /ouster/points")

    def pcd_callback(self, msg: PointCloud2):
        if time.monotonic() < self.next_update:
            return

        try:
            pcd, patches = find_patches(msg)
            if pcd is None or pcd.is_empty():
                return

            edge_voxel_groups = detect_edge_points(pcd, patches)
            edge_count = self.visualizer.show_edges(pcd, edge_voxel_groups)
            self.get_logger().info(f"Displayed {len(pcd.points)} points and {edge_count} edge points")
        except Exception as exc:
            self.get_logger().error(f"Edge detection or visualization failed: {exc}")
        finally:
            self.next_update = time.monotonic() + self.interval


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CornerDetector()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not node.visualizer.poll_events():
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.visualizer.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
