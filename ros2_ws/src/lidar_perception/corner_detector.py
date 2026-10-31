#!/usr/bin/env python3

# Detect lidar edges on the Jetson and publish a colored cloud for the Karbon.
import socket
import sys
import time

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
          "refusing to process lidar on this computer.", file=sys.stderr)
    raise SystemExit(2)

import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

if not hasattr(o3d.geometry.PointCloud, "detect_planar_patches"):
    raise RuntimeError("The Jetson's Open3D installation lacks detect_planar_patches")

EDGE_VOXEL_SIZE = 0.05  # Larger values widen the edges.
EDGE_CLOUD_TOPIC = "/lidar_perception/edge_cloud"


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
            radius=0.3,  # 30cm starting value
            max_nn=100,
        )
    )

    # Detect planar patches
    patches = pcd.detect_planar_patches(
        normal_variance_threshold_deg=60,  # Allowed variation in point normals. Lower = stricter. Default: 60.
        coplanarity_deg=75,                # How closely points fit a plane. Higher = stricter. Default: 75.
        outlier_ratio=0.65,                # Maximum tolerated outlier ratio. Lower = stricter. Default: 0.75.
        min_plane_edge_length=0.1,         # Minimum length of a patch's longest edge. 0 = auto (1% of cloud span).
        min_num_points=50,                  # Minimum points needed to fit a plane. 0 = auto (0.1% of cloud points).
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=100) # How many neighboring points are examined during plane growing and merging. Default: 30.
    )


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
            if np.dot(normal_patch_a, normal_patch_b) >= 0.9:
                continue

            shared_voxels = patches_voxels[i] & patches_voxels[j]

            if len(shared_voxels) < 2:
                continue

            edge_points.append(shared_voxels)

    return edge_points


def color_edge_points(pcd, edge_voxel_groups):
    # Color cloud points in the detector's edge voxels.
    pcd.paint_uniform_color([0.6, 0.6, 0.6])
    edge_voxels = set().union(*edge_voxel_groups)
    if not edge_voxels:
        return 0
    points = np.asarray(pcd.points)
    voxel_ids = np.floor(points / EDGE_VOXEL_SIZE).astype(np.int64)
    edge_mask = np.fromiter(
        (tuple(voxel) in edge_voxels for voxel in voxel_ids),
        dtype=bool,
        count=len(voxel_ids),
    )
    np.asarray(pcd.colors)[edge_mask] = [0.0, 0.65, 1.0]
    return int(np.count_nonzero(edge_mask))


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
        super().__init__("corner_detector")
        self.interval = 2.0  # Seconds after each processed scan, at most 0.5 Hz.
        self.next_update = 0.0
        latest_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.edge_pub = self.create_publisher(PointCloud2, EDGE_CLOUD_TOPIC, latest_scan)
        self.pcd_sub = self.create_subscription(
            PointCloud2, "/ouster/points", self.pcd_callback, latest_scan
        )
        self.get_logger().info("Corner detector started on Jetson, listening to /ouster/points")

    def pcd_callback(self, msg):
        if time.monotonic() < self.next_update:
            return
        try:
            pcd, patches = find_patches(msg)
            if pcd is None or pcd.is_empty():
                return
            edge_voxel_groups = detect_edge_points(pcd, patches)
            edge_count = color_edge_points(pcd, edge_voxel_groups)
            self.edge_pub.publish(pack_colored_cloud(pcd, msg.header))
            self.get_logger().info(f"Published {len(pcd.points)} points and {edge_count} edge points")
        except Exception as exc:
            self.get_logger().error(f"Edge detection failed on Jetson: {exc}")
        finally:
            self.next_update = time.monotonic() + self.interval


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CornerDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
