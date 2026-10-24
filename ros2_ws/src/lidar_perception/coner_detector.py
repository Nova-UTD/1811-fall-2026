#!/usr/bin/env python3
import copy
import os
import platform
import threading
import time

if "microsoft" in platform.release().lower():
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    os.environ["GALLIUM_DRIVER"] = "llvmpipe"
    os.environ["MESA_LOADER_DRIVER_OVERRIDE"] = "llvmpipe"

import open3d as o3d
import numpy as np
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs_py import point_cloud2

# Largest planes only. A fit with fewer points than this is leftover noise.
MAX_PLANES = 20
MIN_PLANE_POINTS = 100
PLANE_INLIER_DISTANCE = 0.05
# A point is on a seam when it is this close to two planes that actually meet.
EDGE_BAND = 0.08
MIN_PLANE_ANGLE_DEG = 30.0

def _local_display():
    display = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    if display:
        return display
    for name in ("X0", "X1"):
        if os.path.exists(f"/tmp/.X11-unix/{name}"):
            os.environ["DISPLAY"] = ":" + name[1:]
            return os.environ["DISPLAY"]
    return ""

class CornerDetector(Node):
    def __init__(self):
        super().__init__("corner_detector")

        # Visualizer initialization
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest_xyz = None
        self._pending = None
        self._view_fitted = False
        self.vis = o3d.visualization.Visualizer()
        display = _local_display()
        self._vis_open = bool(display) and self.vis.create_window(window_name="corner detector", width=1280, height=720)
        if not self._vis_open:
            self.get_logger().error(
                "No window. This process cannot see the monitor. On the Karbon desktop, "
                "not inside Docker, run: xhost +local:root. Then restart this node with DISPLAY=:0."
            )
            self._worker = None
            return
        self.get_logger().info(f"Opening the viewer on {display}")
        opt = self.vis.get_render_option()
        opt.light_on = False
        opt.background_color = np.array([1.0, 1.0, 1.0])
        opt.point_size = 2.0
        self.create_timer(0.03, self._draw)

        self._worker = threading.Thread(target=self._process_loop, daemon=True)
        self._worker.start()
        self.pcd_sub = self.create_subscription(PointCloud2, "/ouster/points", self.pcd_callback, qos_profile_sensor_data)
        self.get_logger().info("Waiting for /ouster/points")

    # Callback function for the point cloud topic
    def pcd_callback(self, msg):
        if not self._vis_open:
            return
        try:
            cloud = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
        except Exception as exc:
            self.get_logger().error(f"could not read /ouster/points: {exc}", throttle_duration_sec=2.0)
            return
        if xyz.shape[0] < 30:
            return
        with self._lock:
            self._latest_xyz = xyz

    def _process_loop(self):
        while not self._stop.is_set():
            with self._lock:
                xyz = self._latest_xyz
                self._latest_xyz = None
            if xyz is None:
                time.sleep(0.01)
                continue
            start = time.perf_counter()
            try:
                geoms = self._detect(xyz)
            except Exception as exc:
                self.get_logger().error(f"detection failed: {exc}")
                continue
            if geoms is None:
                continue
            corners = len(geoms[1].points) if len(geoms) > 1 else 0
            self.get_logger().info(
                f"scan {xyz.shape[0]} pts -> {len(geoms[0].points)} after voxel, {corners} edges, "
                f"{(time.perf_counter() - start) * 1000:.0f} ms",
                throttle_duration_sec=2.0,
            )
            with self._lock:
                self._pending = geoms

    def close(self):
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
        if self._vis_open:
            self._vis_open = False
            self.vis.destroy_window()

    def _detect(self, xyz):
        # Load point cloud and downsample
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
        downpcd = pcd.voxel_down_sample(voxel_size=0.02)
        if len(downpcd.points) < 30:
            return None

        # Detect planes using RANSAC
        planes = []
        supports = []
        remaining = copy.deepcopy(downpcd)
        for _ in range(MAX_PLANES):
            if len(remaining.points) < MIN_PLANE_POINTS:
                break
            plane_model, inliers = remaining.segment_plane(
                distance_threshold=PLANE_INLIER_DISTANCE, ransac_n=3, num_iterations=1000)
            if len(inliers) < MIN_PLANE_POINTS:
                break
            inlier_points = np.asarray(remaining.points)[inliers]
            planes.append(plane_model)
            supports.append((inlier_points.min(axis=0) - EDGE_BAND, inlier_points.max(axis=0) + EDGE_BAND))
            remaining = remaining.select_by_index(inliers, invert=True)

        downpcd.paint_uniform_color([0.75, 0.75, 0.75])
        downpcd.normals = o3d.utility.Vector3dVector()
        geoms = [downpcd]
        if len(planes) < 2:
            return geoms

        # Find edges between planes using plane equations and distance threshold
        candidate_points = np.asarray(downpcd.points)
        plane_equations = np.asarray(planes)
        equations_abc = plane_equations[:, :3]
        equations_d = plane_equations[:, 3]
        distances = np.abs(candidate_points @ equations_abc.T + equations_d) / np.linalg.norm(equations_abc, axis=1)
        near_plane = distances < EDGE_BAND
        for plane_index, (low, high) in enumerate(supports):
            on_patch = np.all((candidate_points >= low) & (candidate_points <= high), axis=1)
            near_plane[:, plane_index] &= on_patch

        edge = np.zeros(len(candidate_points), dtype=bool)
        min_alignment = np.cos(np.deg2rad(MIN_PLANE_ANGLE_DEG))
        for i in range(len(planes)):
            for j in range(i + 1, len(planes)):
                if abs(np.dot(equations_abc[i], equations_abc[j])) >= min_alignment:
                    continue
                edge |= near_plane[:, i] & near_plane[:, j]
        potential_points_indices = np.flatnonzero(edge)
        if potential_points_indices.size == 0:
            return geoms

        # Intersection of two or more planes
        edge_cloud = downpcd.select_by_index(potential_points_indices)
        edge_cloud.paint_uniform_color([0.0, 0.0, 0.0])
        edge_cloud.normals = o3d.utility.Vector3dVector()
        geoms.append(edge_cloud)
        return geoms

    # Draw the latest scan in the window
    def _draw(self):
        if not self._vis_open:
            return
        if not self.vis.poll_events():
            self._vis_open = False
            self._stop.set()
            self.vis.destroy_window()
            self.get_logger().info("Viewer closed. Corner detector is idle.")
            return
        with self._lock:
            pending = self._pending
            self._pending = None
        if pending is not None:
            self.vis.clear_geometries()
            for geom in pending:
                self.vis.add_geometry(geom, reset_bounding_box=not self._view_fitted)
            self._view_fitted = True
        self.vis.update_renderer()

# Run the node
def main(args=None):
    rclpy.init(args=args)

    node = CornerDetector()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
