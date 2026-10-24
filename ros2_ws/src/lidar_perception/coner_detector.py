#!/usr/bin/env python3
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
                f"scan {xyz.shape[0]} pts -> {len(geoms[0].points)} after voxel, {corners} corners, "
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

        # Calculate 1 normal (vector) per point from the closest 30 points inside 0.1 m.
        downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        points_np = np.asarray(downpcd.points)
        points = o3d.core.Tensor(points_np, dtype=o3d.core.float32)
        normals = np.asarray(downpcd.normals, dtype=np.float32)

        # Detect different planes in the point cloud
        boxes = downpcd.detect_planar_patches(
            normal_variance_threshold_deg=60,
            coplanarity_deg=75,
            outlier_ratio=0.75,
            min_plane_edge_length=0.0,
            min_num_points=100,
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30),
        )

        # Color different segmented planes
        colors = np.full((len(points_np), 3), 0.75)
        claimed = np.zeros(len(points_np), dtype=bool)
        for box in boxes:
            indices = np.asarray(box.get_point_indices_within_bounding_box(downpcd.points), dtype=np.int64)
            indices = indices[~claimed[indices]]
            if indices.size < 100:
                continue
            claimed[indices] = True
            colors[indices] = (0.15, 0.45, 0.95)
        downpcd.colors = o3d.utility.Vector3dVector(colors)

        # Find 20 nearest neighbors for each point to calculate normals and find corners
        nns = o3d.core.nns.NearestNeighborSearch(points)
        nns.knn_index()
        idx, _ = nns.knn_search(points, 20)
        
        # Detect corners: a point whose neighborhood disagrees in more than two directions
        cosine = np.abs(np.sum(normals[idx.numpy()] * normals[:, None, :], axis=2))
        sharp = np.count_nonzero(cosine < np.cos(np.deg2rad(35.0)), axis=1)
        potential_corners = np.flatnonzero(sharp > 1)
        if potential_corners.size == 0:
            downpcd.normals = o3d.utility.Vector3dVector()
            return [downpcd]
        edge_cloud = downpcd.select_by_index(potential_corners)

        # Group corners into clusters
        clusters = edge_cloud.cluster_dbscan(eps=0.1, min_points=10)

        # Color different corner clusters
        labels = np.asarray(clusters)
        n_clusters = int(labels.max(initial=-1)) + 1
        corner_colors = np.zeros((labels.shape[0], 3), dtype=np.float64)
        if n_clusters:
            hue = (np.arange(n_clusters) * 0.618033988749895) % 1.0
            sector = np.floor(hue * 6.0).astype(np.int32) % 6
            fraction = hue * 6.0 - np.floor(hue * 6.0)
            sat, val = 0.85, 0.95
            p = val * (1.0 - sat)
            q = val * (1.0 - sat * fraction)
            tch = val * (1.0 - sat * (1.0 - fraction))
            palette = np.stack((
                np.select([sector == 0, sector == 1, sector == 2, sector == 3, sector == 4], [val, q, p, p, tch], default=val),
                np.select([sector == 0, sector == 1, sector == 2, sector == 3, sector == 4], [tch, val, val, q, p], default=p),
                np.select([sector == 0, sector == 1, sector == 2, sector == 3, sector == 4], [p, p, tch, val, val], default=q),
            ), axis=1)
            valid = labels >= 0
            corner_colors[valid] = palette[labels[valid]]
        edge_cloud.colors = o3d.utility.Vector3dVector(corner_colors)
        geoms = [downpcd, edge_cloud]
        for geom in geoms:
            geom.normals = o3d.utility.Vector3dVector()
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
