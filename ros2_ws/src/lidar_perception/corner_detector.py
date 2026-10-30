#!/usr/bin/env python3
import colorsys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d


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
    pcd = pcd.voxel_down_sample(voxel_size=0.05)
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


class PatchVisualizer:
    # Display a point cloud and its planar patches in an Open3D window.

    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        if not self.vis.create_window(
            window_name="Lidar planar patches", width=1280, height=720
        ):
            raise RuntimeError("Could not open the Open3D window; check DISPLAY and X11 access")

        options = self.vis.get_render_option()
        options.background_color = np.array([0.05, 0.05, 0.05])
        options.point_size = 2.0
        options.light_on = False
        self.view_fitted = False

    def show_patches(self, pcd, patches):
        # Replace the displayed cloud and draw each patch in a different color.
        pcd.paint_uniform_color([0.6, 0.6, 0.6])
        self.vis.clear_geometries()
        self.vis.add_geometry(pcd, reset_bounding_box=not self.view_fitted)
        self.view_fitted = True

        for i, patch in enumerate(patches):
            color = colorsys.hsv_to_rgb(i / len(patches), 0.85, 0.95)
            mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(
                patch, scale=np.array([1.0, 1.0, 0.0001])
            )
            mesh.paint_uniform_color(color)
            self.vis.add_geometry(mesh, reset_bounding_box=False)

        self.vis.update_renderer()

    def poll_events(self):
        return self.vis.poll_events()

    def close(self):
        self.vis.destroy_window()


class CornerDetector(Node):
    def __init__(self):
        super().__init__("corner_detector")
        self.visualizer = PatchVisualizer()
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

            self.visualizer.show_patches(pcd, patches)
            self.get_logger().info(f"Displayed {len(pcd.points)} points and {len(patches)} patches")
        except Exception as exc:
            self.get_logger().error(f"Patch visualization failed: {exc}")
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
