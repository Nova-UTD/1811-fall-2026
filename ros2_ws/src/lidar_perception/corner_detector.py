#!/usr/bin/env python3
import os
import platform
import threading
import time

if "microsoft" in platform.release().lower():
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    os.environ["GALLIUM_DRIVER"] = "llvmpipe"
    os.environ["MESA_LOADER_DRIVER_OVERRIDE"] = "llvmpipe"

try:
    import open3d as o3d
    HAS_O3D = True
except ImportError:
    o3d = None
    HAS_O3D = False

try:
    import torch
    HAS_CUDA = torch.cuda.is_available()
except ImportError:
    torch = None
    HAS_CUDA = False

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

_VOXEL_OFFSETS_27 = np.array(
    [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)],
    dtype=np.int64,
)
_NEIGHBOR_TUPLE_27 = [tuple(row) for row in _VOXEL_OFFSETS_27]


def _local_display():
    display = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    if display:
        return display
    for name in ("X0", "X1"):
        if os.path.exists(f"/tmp/.X11-unix/{name}"):
            os.environ["DISPLAY"] = ":" + name[1:]
            return os.environ["DISPLAY"]
    return ""


def _pack_voxels(coords: np.ndarray) -> np.ndarray:
    """Pack integer 3D voxel coordinates into 1D int64 keys for fast lookup."""
    return (coords[:, 0] * 1000003 + coords[:, 1]) * 1000003 + coords[:, 2]


def _largest_connected_wall(pts: np.ndarray, eps: float = 0.22) -> np.ndarray:
    """Return a boolean mask of the single largest physically connected cluster of points within distance eps.

    Unlike axis-aligned percentiles, this keeps 100% of a contiguous diagonal wall
    while dropping 100% of disconnected points where the infinite plane sliced other objects across the room.
    """
    if len(pts) == 0:
        return np.zeros(0, dtype=bool)
    cell_size = eps / np.sqrt(3.0)
    coords = np.floor(pts / cell_size).astype(np.int64)
    voxel_map = {}
    for idx, c in enumerate(map(tuple, coords)):
        voxel_map.setdefault(c, []).append(idx)

    visited = set()
    best_component = []
    for start_voxel in voxel_map:
        if start_voxel in visited:
            continue
        queue = [start_voxel]
        visited.add(start_voxel)
        comp = []
        while queue:
            vx, vy, vz = queue.pop()
            comp.extend(voxel_map[(vx, vy, vz)])
            for dx, dy, dz in _NEIGHBOR_TUPLE_27:
                nb = (vx + dx, vy + dy, vz + dz)
                if nb in voxel_map and nb not in visited:
                    visited.add(nb)
                    queue.append(nb)
        if len(comp) > len(best_component):
            best_component = comp

    mask = np.zeros(len(pts), dtype=bool)
    mask[best_component] = True
    return mask


def _near_finite_wall(candidate_pts: np.ndarray, wall_pts: np.ndarray, radius: float = 0.08) -> np.ndarray:
    """Return boolean mask of candidate_pts that are physically within ~radius of an actual point on wall_pts."""
    w_coords = np.floor(wall_pts / radius).astype(np.int64)
    expanded = (w_coords[:, None, :] + _VOXEL_OFFSETS_27[None, :, :]).reshape(-1, 3)
    expanded_keys = np.unique(_pack_voxels(expanded))
    c_keys = _pack_voxels(np.floor(candidate_pts / radius).astype(np.int64))
    return np.isin(c_keys, expanded_keys)


def _pack_xyzrgb_msg(xyz: np.ndarray, colors: np.ndarray, header) -> PointCloud2:
    """Pack Nx3 XYZ (float) and Nx3 RGB (0..1 float) into a PointCloud2 message."""
    pts = np.asarray(xyz, dtype=np.float32)
    rgb_u8 = np.clip(np.asarray(colors) * 255.0, 0, 255).astype(np.uint32)
    rgb_f32 = ((rgb_u8[:, 0] << 16) | (rgb_u8[:, 1] << 8) | rgb_u8[:, 2]).view(np.float32)

    cloud_arr = np.empty(
        len(pts),
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")],
    )
    cloud_arr["x"] = pts[:, 0]
    cloud_arr["y"] = pts[:, 1]
    cloud_arr["z"] = pts[:, 2]
    cloud_arr["rgb"] = rgb_f32

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(pts)
    msg.is_dense = True
    msg.data = cloud_arr.tobytes()
    return msg


def _unpack_xyzrgb_msg(msg: PointCloud2):
    """Unpack a PointCloud2 message produced by _pack_xyzrgb_msg into (xyz, colors)."""
    arr = np.frombuffer(
        msg.data,
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")],
    )
    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)
    rgb_u32 = arr["rgb"]
    colors = (
        np.column_stack(((rgb_u32 >> 16) & 0xFF, (rgb_u32 >> 8) & 0xFF, rgb_u32 & 0xFF)).astype(np.float64)
        / 255.0
    )
    return xyz, colors


def _ransac_plane_cuda_or_numpy(pts: np.ndarray, distance_threshold: float, num_iterations: int):
    """Fast batch RANSAC plane fitting (uses Jetson CUDA GPU if PyTorch CUDA is available, else NumPy)."""
    n_pts = pts.shape[0]
    rng = np.random.default_rng()
    idx = rng.integers(0, n_pts, size=(num_iterations, 3))

    if HAS_CUDA:
        dev = torch.device("cuda:0")
        t_pts = torch.as_tensor(pts, dtype=torch.float32, device=dev)
        t_idx = torch.as_tensor(idx, dtype=torch.long, device=dev)
        p1 = t_pts[t_idx[:, 0]]
        p2 = t_pts[t_idx[:, 1]]
        p3 = t_pts[t_idx[:, 2]]
        normals = torch.cross(p2 - p1, p3 - p1, dim=1)
        norms = torch.linalg.norm(normals, dim=1, keepdim=True)
        valid = norms.squeeze(1) > 1e-6
        normals = normals[valid] / norms[valid]
        p1 = p1[valid]
        if normals.shape[0] == 0:
            return None, []
        d = -torch.sum(normals * p1, dim=1)
        dists = torch.abs(t_pts @ normals.T + d)
        inlier_counts = torch.sum(dists < distance_threshold, dim=0)
        best = int(torch.argmax(inlier_counts).item())
        best_normal = normals[best].cpu().numpy()
        best_d = float(d[best].cpu().item())
        inliers = torch.nonzero(dists[:, best] < distance_threshold, as_tuple=False).squeeze(1).cpu().numpy().tolist()
        plane_model = np.array([best_normal[0], best_normal[1], best_normal[2], best_d], dtype=np.float64)
        return plane_model, inliers

    p1 = pts[idx[:, 0]]
    p2 = pts[idx[:, 1]]
    p3 = pts[idx[:, 2]]
    normals = np.cross(p2 - p1, p3 - p1)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    valid = norms[:, 0] > 1e-6
    normals = normals[valid] / norms[valid]
    p1 = p1[valid]
    if normals.shape[0] == 0:
        return None, []
    d = -np.einsum("ij,ij->i", normals, p1)
    eval_pts = pts if n_pts <= 2000 else pts[rng.choice(n_pts, size=2000, replace=False)]
    dists_sub = np.abs(eval_pts @ normals.T + d)
    best = int(np.argmax(np.count_nonzero(dists_sub < distance_threshold, axis=0)))
    best_n = normals[best]
    best_d = d[best]
    full_dists = np.abs(pts @ best_n + best_d)
    inliers = np.flatnonzero(full_dists < distance_threshold).tolist()
    plane_model = np.array([best_n[0], best_n[1], best_n[2], best_d], dtype=np.float64)
    return plane_model, inliers


class CornerDetector(Node):
    def __init__(self):
        super().__init__("corner_detector")
        self.declare_parameter("headless", False)
        self.declare_parameter("refresh_interval", 2.0)

        self._headless = bool(self.get_parameter("headless").value)
        self._refresh_interval = float(self.get_parameter("refresh_interval").value)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest_xyz = None
        self._latest_header = None
        self._pending = None
        self._view_fitted = False
        self._last_scan_time = 0.0
        self._last_remote_time = 0.0
        self._vis_open = False

        if self._headless:
            # Headless mode (runs on the Jetson): compute planes & corners and publish PointCloud2 topics
            backend = "CUDA GPU" if HAS_CUDA else ("Open3D" if HAS_O3D else "NumPy")
            self.get_logger().info(
                f"Corner detector started in headless compute mode ({backend}) -> publishing /lidar_perception/segmented_points"
            )
            self.seg_pub = self.create_publisher(PointCloud2, "/lidar_perception/segmented_points", qos_profile_sensor_data)
            self.corner_pub = self.create_publisher(PointCloud2, "/lidar_perception/corners", qos_profile_sensor_data)
            self._worker = threading.Thread(target=self._process_loop, daemon=True)
            self._worker.start()
            self.pcd_sub = self.create_subscription(
                PointCloud2, "/ouster/points", self.pcd_callback, qos_profile_sensor_data
            )
            return

        # Viewer mode (runs on the Karbon): open the Open3D window
        if not HAS_O3D:
            self.get_logger().error("Open3D is not installed; cannot open viewer window.")
            self._worker = None
            return

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
        self.create_timer(0.05, self._draw)

        self._worker = threading.Thread(target=self._process_loop, daemon=True)
        self._worker.start()
        # Subscribe to pre-computed cloud from the Jetson, and fallback to /ouster/points if Jetson isn't running
        self.remote_sub = self.create_subscription(
            PointCloud2, "/lidar_perception/segmented_points", self.remote_cloud_callback, qos_profile_sensor_data
        )
        self.pcd_sub = self.create_subscription(
            PointCloud2, "/ouster/points", self.pcd_callback, qos_profile_sensor_data
        )
        self.get_logger().info("Waiting for /lidar_perception/segmented_points (from Jetson) or /ouster/points")

    def remote_cloud_callback(self, msg: PointCloud2):
        """Receive already-segmented colored point cloud from the Jetson (zero RANSAC on the Karbon)."""
        if not self._vis_open or msg.width == 0:
            return
        try:
            xyz, colors = _unpack_xyzrgb_msg(msg)
        except Exception as exc:
            self.get_logger().error(f"could not unpack /lidar_perception/segmented_points: {exc}", throttle_duration_sec=2.0)
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        self._last_remote_time = time.monotonic()
        with self._lock:
            self._latest_xyz = None
            self._pending = [pcd]

    # Callback function for the raw point cloud topic
    def pcd_callback(self, msg: PointCloud2):
        if not self._headless and not self._vis_open:
            return
        now = time.monotonic()
        # If the Jetson is actively sending segmented clouds, skip local RANSAC on the Karbon
        if not self._headless and (now - self._last_remote_time < 4.0):
            return
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
            self._latest_header = msg.header

    def _process_loop(self):
        while not self._stop.is_set():
            with self._lock:
                xyz = self._latest_xyz
                header = self._latest_header
                self._latest_xyz = None
            if xyz is None:
                time.sleep(0.02)
                continue
            start = time.perf_counter()
            try:
                result = self._detect_arrays(xyz)
            except Exception as exc:
                self.get_logger().error(f"detection failed: {exc}")
                continue
            if result is None:
                continue
            down_xyz, colors, corner_indices = result
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            self.get_logger().info(
                f"scan {xyz.shape[0]} pts -> {len(down_xyz)} after voxel, {len(corner_indices)} edges, {elapsed_ms:.0f} ms",
                throttle_duration_sec=2.0,
            )

            if self._headless:
                seg_msg = _pack_xyzrgb_msg(down_xyz, colors, header)
                self.seg_pub.publish(seg_msg)
                if len(corner_indices) > 0:
                    corner_msg = _pack_xyzrgb_msg(down_xyz[corner_indices], colors[corner_indices], header)
                    self.corner_pub.publish(corner_msg)
            else:
                geoms = self._build_geoms(down_xyz, colors, corner_indices)
                with self._lock:
                    self._pending = geoms

    def close(self):
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
        if self._vis_open:
            self._vis_open = False
            self.vis.destroy_window()

    def _detect_arrays(self, xyz: np.ndarray):
        """Run voxel downsampling, connected-wall RANSAC extraction, and finite-wall corner detection."""
        pts = xyz.astype(np.float64)
        if HAS_O3D and not HAS_CUDA:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            downpcd = pcd.voxel_down_sample(voxel_size=0.02)
            if len(downpcd.points) < 30:
                return None
            candidate_points = np.asarray(downpcd.points)
        else:
            voxel_coords = np.floor(pts / 0.02).astype(np.int64)
            _, unique_idx = np.unique(voxel_coords, axis=0, return_index=True)
            if unique_idx.size < 30:
                return None
            candidate_points = pts[unique_idx]

        # Track indices into candidate_points so we know the exact physical points of each wall
        remaining_indices = np.arange(len(candidate_points), dtype=np.int64)
        plane_threshold = 0.05
        edge_band = 0.06
        min_plane_points = 120
        segment_planes = []
        wall_point_indices = []

        for _ in range(25):
            if len(remaining_indices) < min_plane_points:
                break
            rem_pts = candidate_points[remaining_indices]

            if HAS_O3D and not HAS_CUDA:
                rem_pcd = o3d.geometry.PointCloud()
                rem_pcd.points = o3d.utility.Vector3dVector(rem_pts)
                plane_model, inliers = rem_pcd.segment_plane(
                    distance_threshold=plane_threshold, ransac_n=3, num_iterations=800
                )
            else:
                plane_model, inliers = _ransac_plane_cuda_or_numpy(
                    rem_pts, distance_threshold=plane_threshold, num_iterations=800
                )

            if plane_model is None or len(inliers) < min_plane_points:
                break  # Largest remaining plane is too small, stop early

            inliers_arr = np.asarray(inliers, dtype=np.int64)
            inlier_pts = rem_pts[inliers_arr]

            # Keep ONLY the single largest physically connected cluster of this plane!
            # This stops an infinite plane from bridging across empty space onto another wall.
            connected_mask = _largest_connected_wall(inlier_pts, eps=0.22)
            connected_local = inliers_arr[connected_mask]

            # If the connected cluster itself is too small (just scattered clutter), discard and continue
            if len(connected_local) < min_plane_points:
                keep_mask = np.ones(len(remaining_indices), dtype=bool)
                keep_mask[inliers_arr] = False
                remaining_indices = remaining_indices[keep_mask]
                continue

            # Only keep this plane if it isn't almost parallel and close to one we already found
            for segmented_planes in segment_planes:
                dot = np.dot(plane_model[:3], segmented_planes[:3])
                if dot > 0.90 and np.abs(plane_model[3] - segmented_planes[3]) < 0.25:
                    break  # Same normal direction and close -> duplicate plane
                elif dot < -0.90 and np.abs(plane_model[3] + segmented_planes[3]) < 0.25:
                    break  # Opposite normal direction and close -> duplicate plane
            else:
                wall_point_indices.append(remaining_indices[connected_local])
                segment_planes.append(plane_model)

            # Remove only the connected wall's points so distant walls aren't sliced
            keep_mask = np.ones(len(remaining_indices), dtype=bool)
            keep_mask[connected_local] = False
            remaining_indices = remaining_indices[keep_mask]

        # Color the base point cloud light gray for unsegmented background points
        colors = np.full((len(candidate_points), 3), 0.75, dtype=np.float64)
        if len(segment_planes) == 0:
            return candidate_points, colors, np.empty(0, dtype=np.int64)

        # Paint ONLY the actual connected physical points of each detected wall (never project across the room)
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
        for idx, wall_idx in enumerate(wall_point_indices):
            colors[wall_idx] = plane_colors[idx % len(plane_colors)]

        if len(segment_planes) < 2:
            return candidate_points, colors, np.empty(0, dtype=np.int64)

        # Measure perpendicular distance to each plane AND require physical proximity (~8 cm) to that wall's real points
        plane_equations = np.asarray(segment_planes)
        equations_abc = plane_equations[:, :3]
        equations_abc = equations_abc / np.linalg.norm(equations_abc, axis=1, keepdims=True)
        equations_d = plane_equations[:, 3]
        distances = np.abs(candidate_points @ equations_abc.T + equations_d)

        near_plane = distances < edge_band
        for idx, wall_idx in enumerate(wall_point_indices):
            wall_pts = candidate_points[wall_idx]
            near_plane[:, idx] &= _near_finite_wall(candidate_points, wall_pts, radius=0.08)

        # Only mark an edge where two finite walls physically meet at a real angle (|cos(theta)| < 0.75, i.e. > ~41 deg)
        cos_angles = np.abs(equations_abc @ equations_abc.T)
        plane_i, plane_j = np.where(np.triu(cos_angles < 0.75, k=1))
        if plane_i.size == 0:
            return candidate_points, colors, np.empty(0, dtype=np.int64)
        is_edge_point = np.any(near_plane[:, plane_i] & near_plane[:, plane_j], axis=1)
        potential_points_indices = np.flatnonzero(is_edge_point)
        if potential_points_indices.size == 0:
            return candidate_points, colors, np.empty(0, dtype=np.int64)

        # Highlight the detected edge/corner points in black
        colors[potential_points_indices] = [0.0, 0.0, 0.0]
        return candidate_points, colors, potential_points_indices

    def _build_geoms(self, down_xyz: np.ndarray, colors: np.ndarray, corner_indices: np.ndarray):
        downpcd = o3d.geometry.PointCloud()
        downpcd.points = o3d.utility.Vector3dVector(down_xyz)
        downpcd.colors = o3d.utility.Vector3dVector(colors)
        geoms = [downpcd]
        if len(corner_indices) > 0:
            edge_cloud = downpcd.select_by_index(corner_indices.tolist())
            edge_cloud.paint_uniform_color([0.0, 0.0, 0.0])
            geoms.append(edge_cloud)
        return geoms

    def _detect(self, xyz):
        res = self._detect_arrays(xyz)
        if res is None:
            return None
        return self._build_geoms(*res)

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
