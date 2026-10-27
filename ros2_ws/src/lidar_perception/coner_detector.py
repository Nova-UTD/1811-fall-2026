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
        self._refresh_interval = 2.0  # Seconds between frame updates to keep CPU/GPU load low
        self._last_scan_time = 0.0
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
        now = time.monotonic()
        if now - self._last_scan_time < self._refresh_interval:
            return
        try:
            cloud = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
        except Exception as exc:
            self.get_logger().error(f"could not read /ouster/points: {exc}", throttle_duration_sec=2.0)
            return
        if xyz.shape[0] < 30:
            return
        self._last_scan_time = now
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
        # Convert raw XYZ array to an Open3D point cloud and downsample (2 cm voxels)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
        downpcd = pcd.voxel_down_sample(voxel_size=0.02)
        if len(downpcd.points) < 30:
            return None

        # Repeatedly fit planes with RANSAC, stripping out found planes each pass
        min_plane_points = 80
        segment_planes = []
        segment_downpcd = downpcd
        for i in range(50):
            if len(segment_downpcd.points) < min_plane_points:
                break
            plane_model, inliers = segment_downpcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=300)
            if len(inliers) < min_plane_points:
                break  # Largest remaining plane is just small noise, stop early

            # Only keep this plane if it isn't almost parallel to one we already found
            for segmented_planes in segment_planes:
                dot = np.dot(plane_model[:3], segmented_planes[:3])
                if dot > 0.95 and np.abs(plane_model[3] - segmented_planes[3]) < 0.15:
                    break  # Same normal direction and within 10 cm -> duplicate plane
                elif dot < -0.95 and np.abs(plane_model[3] + segmented_planes[3]) < 0.15:
                    break  # Opposite normal direction and within 10 cm -> duplicate plane
            else:
                segment_planes.append(plane_model) 

            # Remove the points belonging to this plane before searching for the next one
            segment_downpcd = segment_downpcd.select_by_index(inliers, invert=True)

        # Color the base point cloud light gray for unsegmented background points
        downpcd.paint_uniform_color([0.75, 0.75, 0.75])
        downpcd.normals = o3d.utility.Vector3dVector()
        geoms = [downpcd]
        if len(segment_planes) == 0:
            return geoms

        # Measure the perpendicular distance from every point to every detected plane (ax + by + cz + d = 0)
        candidate_points = np.asarray(downpcd.points)
        plane_equations = np.asarray(segment_planes)
        equations_abc = plane_equations[:, :3]  # Plane normal vectors [a, b, c]
        equations_d = plane_equations[:, 3]     # Plane offsets [d]
        distances = np.abs(candidate_points @ equations_abc.T + equations_d) / np.linalg.norm(equations_abc, axis=1)

        # Paint each detected plane with a distinct color
        plane_colors = [
            [0.90, 0.20, 0.20],  # Red
            [0.20, 0.75, 0.20],  # Green
            [0.20, 0.45, 0.90],  # Blue
            [0.95, 0.60, 0.10],  # Orange
            [0.65, 0.25, 0.85],  # Purple
            [0.10, 0.80, 0.80],  # Cyan
            [0.90, 0.80, 0.15],  # Yellow
            [0.90, 0.30, 0.65],  # Pink
        ]
        colors = np.asarray(downpcd.colors)
        for idx in range(len(segment_planes)):
            colors[distances[:, idx] < 0.02] = plane_colors[idx % len(plane_colors)]

        if len(segment_planes) < 2:
            return geoms

        # Points within 5 cm of at least 2 non-parallel planes lie on an edge/corner intersection
        potential_points_indices = np.flatnonzero(np.count_nonzero(distances < 0.05, axis=1) >= 2)
        if potential_points_indices.size == 0:
            return geoms

        # Highlight the detected edge/corner points in black
        colors[potential_points_indices] = [0.0, 0.0, 0.0]
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
