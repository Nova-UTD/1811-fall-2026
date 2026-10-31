#!/usr/bin/env python3

# Detect lidar and panorama edges on the Jetson for display on the Karbon.
import socket
import sys

def _is_jetson_link():
    # Check the configured Jetson cable address before loading the detector.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            # UDP connect selects a route and local address; it sends no data.
            sock.connect(("192.168.100.1", 9))
            return sock.getsockname()[0] == "192.168.100.2"
    except OSError:
        return False

if __name__ == "__main__" and not _is_jetson_link():
    print("Corner detection requires the Jetson cable address 192.168.100.2; "
          "refusing to process sensor data on this computer.", file=sys.stderr)
    raise SystemExit(2)

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField

EDGE_VOXEL_SIZE = 0.25  # Larger voxels increase the matching tolerance.
EDGE_POINT_SPACING = 0.025  # Maximum distance between interpolated points, in meters.
EDGE_CLOUD_TOPIC = "/lidar_perception/edge_cloud"
EDGE_NEIGHBOR_OFFSETS = tuple(sorted(
    ((dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
     for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0)),
    key=lambda offset: (sum(value * value for value in offset), offset),
))


def find_patches(msg):
    # Only the lidar process needs Open3D and the point cloud decoder.
    import open3d as o3d
    from sensor_msgs_py import point_cloud2

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
    pcd = pcd.voxel_down_sample(voxel_size=0.015)
    if len(pcd.points) < 30:
        return pcd, []

    # Planar patch detection requires a normal for each point.
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.10,  # meters
            max_nn=30,
        )
    )

    # Detect planar patches
    patches = pcd.detect_planar_patches(
        normal_variance_threshold_deg=40,  # Allowed variation in point normals. Lower = stricter. Default: 60.
        coplanarity_deg=70,                # How closely points fit a plane. Higher = stricter. Default: 75.
        outlier_ratio=0.75,                # Maximum tolerated outlier ratio. Lower = stricter. Default: 0.75.
        min_plane_edge_length=0.1,         # Minimum length of a patch's longest edge. 0 = auto (1% of cloud span).
        min_num_points=20,                  # Minimum points needed to fit a plane. 0 = auto (0.1% of cloud points).
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20) # How many neighboring points are examined during plane growing and merging. Default: 30.
    )


    # Return the downsampled point cloud and the detected patches
    return pcd, patches

def detect_lidar_edge_points(pcd,patches):
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
        indices = np.asarray(
            patch.get_point_indices_within_bounding_box(pcd.points), dtype=np.intp
        )
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


def nearest_edge_connections(edge_voxel_groups):
    """Connect each voxel to its nearest other voxel in the same patch pair."""
    connections = set()
    for group in edge_voxel_groups:
        if len(group) < 2:
            continue
        ordered_voxels = sorted(group)
        tree = None
        for voxel in ordered_voxels:
            # Any touching voxel is closer than every non-touching voxel.
            # Offsets are sorted by distance, then by voxel coordinates.
            neighbor = None
            for dx, dy, dz in EDGE_NEIGHBOR_OFFSETS:
                candidate = (voxel[0] + dx, voxel[1] + dy, voxel[2] + dz)
                if candidate in group:
                    neighbor = candidate
                    break

            if neighbor is None:
                # Sparse groups can have gaps. Build this index only when needed.
                if tree is None:
                    import open3d as o3d
                    voxel_cloud = o3d.geometry.PointCloud()
                    voxel_cloud.points = o3d.utility.Vector3dVector(
                        np.asarray(ordered_voxels, dtype=np.float64)
                    )
                    tree = o3d.geometry.KDTreeFlann(voxel_cloud)
                query = np.asarray(voxel, dtype=np.float64)
                _, _, distances = tree.search_knn_vector_3d(query, 2)
                radius = np.sqrt(distances[1]) + 1e-9
                _, indices, squared_distances = tree.search_radius_vector_3d(query, radius)
                # Resolve equal-distance ties by coordinates for stable results.
                neighbor = min(
                    (distance, ordered_voxels[index])
                    for index, distance in zip(indices, squared_distances)
                    if ordered_voxels[index] != voxel
                )[1]

            connections.add(tuple(sorted((voxel, neighbor))))
    return connections


def interpolate_edge_voxels(edge_voxel_groups):
    # Connect voxel pairs only when they share a patch-pair group.
    edge_voxels = set().union(*edge_voxel_groups)
    if not edge_voxels:
        return np.empty((0, 3), dtype=np.float64)

    voxel_indices = np.asarray(sorted(edge_voxels), dtype=np.float64)
    centers = (voxel_indices + 0.5) * EDGE_VOXEL_SIZE
    connections = nearest_edge_connections(edge_voxel_groups)

    point_groups = [centers]
    for start_voxel, end_voxel in sorted(connections):
        start = (np.asarray(start_voxel, dtype=np.float64) + 0.5) * EDGE_VOXEL_SIZE
        delta = (np.asarray(end_voxel) - np.asarray(start_voxel)) * EDGE_VOXEL_SIZE
        steps = int(np.ceil(np.linalg.norm(delta) / EDGE_POINT_SPACING))
        if steps > 1:
            fractions = np.arange(1, steps, dtype=np.float64) / steps
            point_groups.append(start + fractions[:, None] * delta)
    return np.concatenate(point_groups, axis=0)


def color_edge_points(pcd, edge_voxel_groups):
    # Keep the lidar points black and add blue voxel centers and connecting points.
    pcd.paint_uniform_color([0.0, 0.0, 0.0])
    edge_points = interpolate_edge_voxels(edge_voxel_groups)
    if len(edge_points) == 0:
        return 0
    import open3d as o3d

    edge_cloud = o3d.geometry.PointCloud()
    edge_cloud.points = o3d.utility.Vector3dVector(edge_points)
    edge_cloud.paint_uniform_color([0.0, 0.65, 1.0])
    pcd += edge_cloud
    return len(edge_points)


def pack_colored_cloud(pcd, header):
    # Pack XYZ and RGB into the standard PointCloud2 layout.
    points = np.asarray(pcd.points)
    colors = (np.clip(np.asarray(pcd.colors), 0.0, 1.0) * 255).astype(np.uint32)
    rgb = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    packed = np.empty(len(points), dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4"),
    ])
    packed["x"], packed["y"], packed["z"] = points.T
    packed["rgb"] = rgb

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(packed)
    msg.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in (("x", 0), ("y", 4), ("z", 8), ("rgb", 12))]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(packed)
    msg.is_dense = True
    msg.data = packed.tobytes()
    return msg


class CornerDetector(Node):
    def __init__(self):
        if not _is_jetson_link():
            raise RuntimeError("Corner detection only runs on the configured Jetson (192.168.100.2)")
        import open3d as o3d
        if not hasattr(o3d.geometry.PointCloud, "detect_planar_patches"):
            raise RuntimeError("The Jetson's Open3D installation lacks detect_planar_patches")
        super().__init__("corner_detector")
        # Keep only the latest pending scan if detection is slower than the lidar.
        latest_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.edge_pub = self.create_publisher(PointCloud2, EDGE_CLOUD_TOPIC, latest_scan)
        self.pcd_sub = self.create_subscription(
            PointCloud2, "/ouster/points", self.pcd_callback, latest_scan
        )
        self.get_logger().info("Corner detector started on Jetson, listening to /ouster/points")

    def pcd_callback(self, msg):
        try:
            pcd, patches = find_patches(msg)
            if pcd is None or pcd.is_empty():
                return
            edge_voxel_groups = detect_lidar_edge_points(pcd, patches)
            edge_count = color_edge_points(pcd, edge_voxel_groups)
            self.edge_pub.publish(pack_colored_cloud(pcd, msg.header))
            self.get_logger().info(
                f"Published {len(pcd.points) - edge_count} lidar points and "
                f"{edge_count} blue edge points",
                throttle_duration_sec=5.0,
            )
        except Exception as exc:
            self.get_logger().error(f"Edge detection failed on Jetson: {exc}")


def detect_camera_edges(image_bgr):
    # Only the camera process needs OpenCV.
    import cv2
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    return cv2.Canny(blurred, 50, 150)


class CameraEdgeDetector(Node):
    def __init__(self):
        if not _is_jetson_link():
            raise RuntimeError("Panorama edge detection only runs on the configured Jetson")
        from camera_panorama.panorama_node import image_to_bgr
        super().__init__("camera_edge_detector")
        self._image_to_bgr = image_to_bgr
        latest_image = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.panorama_edge_pub = self.create_publisher(Image, "/panorama/edges", latest_image)
        self.panorama_sub = self.create_subscription(
            Image, "/panorama/image", self.camera_callback, latest_image
        )
        self.get_logger().info("Panorama edge detector started on Jetson, listening to /panorama/image")

    def camera_callback(self, msg):
        try:
            image_bgr = self._image_to_bgr(msg)
            edges = detect_camera_edges(image_bgr)
            overlay = image_bgr.copy()
            overlay[edges != 0] = (255, 165, 0)  # Neon blue in BGR.

            out = Image()
            out.header = msg.header
            out.height, out.width = overlay.shape[:2]
            out.encoding = "bgr8"
            out.step = out.width * 3
            out.data = overlay.tobytes()
            self.panorama_edge_pub.publish(out)
        except Exception as exc:
            self.get_logger().error(f"Panorama edge detection failed: {exc}",
                                    throttle_duration_sec=5.0)


def main(args=None):
    # The launch file starts this script twice; remove our mode flag before ROS
    # parses its own arguments and leave all ROS arguments intact.
    ros_args = list(sys.argv if args is None else args)
    camera_mode = "--camera-edges" in ros_args
    if camera_mode:
        ros_args.remove("--camera-edges")
    rclpy.init(args=ros_args)
    node = None
    try:
        node = CameraEdgeDetector() if camera_mode else CornerDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
