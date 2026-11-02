#!/usr/bin/env python3

# Register four ZED point clouds to the LiDAR scan on the Jetson.
import socket
import sys
from threading import Lock
from typing import NamedTuple


def _is_jetson_link():
    # Check the configured Jetson cable address before loading registration libraries.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            # UDP connect selects a route and local address; it sends no data.
            sock.connect(("192.168.100.1", 9))
            return sock.getsockname()[0] == "192.168.100.2"
    except OSError:
        return False


if __name__ == "__main__" and not _is_jetson_link():
    print("LiDAR and camera registration requires the Jetson cable address 192.168.100.2; "
          "refusing to process sensor data on this computer.", file=sys.stderr)
    raise SystemExit(2)

# NumPy handles point arrays and transforms, and Open3D performs registration.
import numpy as np
import open3d as o3d

# ROS receives sensor messages and publishes the result.
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs_py import point_cloud2
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


REGISTERED_CLOUD_TOPIC = "/lidar_perception/registered_points"  # RViz reads the merged cloud here.
CAMERAS = ("front", "left", "rear", "right")  # Names used in the four ZED topic paths.

# Startup defaults: the launch file loads user settings from registration.yaml.
class RegistrationConfig(NamedTuple):
    icp_voxel_size_m: float = 0.10
    fpfh_voxel_size_m: float = 0.20
    fpfh_normal_radius_multiplier: float = 2.0
    fpfh_feature_radius_multiplier: float = 5.0
    ransac_max_distance_multiplier: float = 1.5
    ransac_max_iterations: int = 100000
    ransac_confidence: float = 0.999
    ransac_min_fitness: float = 0.10
    icp_max_distance_m: float = 0.50
    icp_max_iterations: int = 20
    min_points: int = 30
    icp_min_fitness: float = 0.10
    icp_max_translation_m: float = 0.50
    icp_max_rotation_deg: float = 15.0
    max_icp_failures: int = 3
    max_camera_time_difference_s: float = 0.30
    lidar_rgb: tuple = (230, 230, 230)
    front_rgb: tuple = (40, 160, 255)
    left_rgb: tuple = (50, 220, 80)
    rear_rgb: tuple = (255, 160, 40)
    right_rgb: tuple = (240, 70, 210)

    @property
    def fpfh_normal_radius_m(self):
        return self.fpfh_voxel_size_m * self.fpfh_normal_radius_multiplier

    @property
    def fpfh_feature_radius_m(self):
        return self.fpfh_voxel_size_m * self.fpfh_feature_radius_multiplier

    @property
    def ransac_max_distance_m(self):
        return self.fpfh_voxel_size_m * self.ransac_max_distance_multiplier


DEFAULT_CONFIG = RegistrationConfig()  # Also permits direct execution without a YAML file.


# Read the launch YAML once at startup.
# Settings are startup only - reject runtime changes rather than silently ignoring them.
def load_registration_config(node):
    values = {}
    for name in DEFAULT_CONFIG._fields:
        default = getattr(DEFAULT_CONFIG, name)
        parameter_default = list(default) if name.endswith("_rgb") else default
        value = node.declare_parameter(
            name, parameter_default, ParameterDescriptor(read_only=True),
        ).value
        values[name] = tuple(value) if name.endswith("_rgb") else value
    config = RegistrationConfig(**values)

    positive = (
        "icp_voxel_size_m", "fpfh_voxel_size_m", "fpfh_normal_radius_multiplier",
        "fpfh_feature_radius_multiplier", "ransac_max_distance_multiplier", "icp_max_distance_m",
    )
    for name in positive:
        if not np.isfinite(getattr(config, name)) or getattr(config, name) <= 0:
            raise ValueError(f"{name} must be finite and greater than zero")
    for name in ("ransac_max_iterations", "icp_max_iterations", "max_icp_failures"):
        if getattr(config, name) < 1:
            raise ValueError(f"{name} must be at least 1")
    if config.min_points < 3:
        raise ValueError("min_points must be at least 3")
    for name in ("ransac_min_fitness", "icp_min_fitness"):
        if not np.isfinite(getattr(config, name)) or not 0.0 <= getattr(config, name) <= 1.0:
            raise ValueError(f"{name} must be between 0.0 and 1.0")
    if not np.isfinite(config.ransac_confidence) or not 0.0 < config.ransac_confidence <= 1.0:
        raise ValueError("ransac_confidence must be greater than 0.0 and at most 1.0")
    for name in ("icp_max_translation_m", "icp_max_rotation_deg", "max_camera_time_difference_s"):
        if not np.isfinite(getattr(config, name)) or getattr(config, name) < 0:
            raise ValueError(f"{name} must be finite and at least zero")
    if config.icp_max_rotation_deg > 180.0:
        raise ValueError("icp_max_rotation_deg must be at most 180.0")
    for name in ("lidar_rgb", "front_rgb", "left_rgb", "rear_rgb", "right_rgb"):
        color = getattr(config, name)
        if len(color) != 3 or any(not isinstance(channel, int) or not 0 <= channel <= 255
                                  for channel in color):
            raise ValueError(f"{name} must contain three integers between 0 and 255")
    return config


# Convert LiDAR or ZED PointCloud2 data to an Open3D XYZ cloud.
def point_cloud_from_message(msg):
    # Read XYZ only: ICP does not use intensity or the ZED's packed color field.
    points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    # Turn ROS's named fields into one row of [x, y, z] per point.
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    # Remove invalid coordinates; for example, [NaN, 1, 2] cannot be matched to a surface.
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    # Open3D expects its own point-cloud object with double-precision coordinates.
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    return cloud

# Describe local shapes without needing to know the sensor's mounting pose.
def prepare_fpfh(cloud, config):
    # A coarse cloud reduces feature and RANSAC cost; by default each voxel is 20 cm wide.
    coarse = cloud.voxel_down_sample(config.fpfh_voxel_size_m)
    if len(coarse.points) < config.min_points:
        return None, None  # Too little geometry remains to find a reliable match.

    # FPFH compares local surface shapes, so first estimate a normal at every point.
    # Example: points on level ground have normals roughly perpendicular to the ground.
    coarse.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=config.fpfh_normal_radius_m, max_nn=30,
    ))
    # Face normals toward their own device's origins.
    coarse.orient_normals_towards_camera_location(np.zeros(3))
    # Encode the relationships between nearby normals as 33-value FPFH descriptors.
    # Example: a corner seen by both sensors may produce similar descriptors in both clouds.
    features = o3d.pipelines.registration.compute_fpfh_feature(
        coarse, o3d.geometry.KDTreeSearchParamHybrid(radius=config.fpfh_feature_radius_m, max_nn=100),
    )
    return coarse, features

def estimate_initial_pose(camera, lidar_features, config): # Use FPFH and RANSAC to estimate camera coordinates in LiDAR coordinates.
    camera_coarse, camera_features = prepare_fpfh(camera, config)
    lidar_coarse, lidar_descriptors = lidar_features
    if camera_coarse is None or lidar_coarse is None:
        return None, "too few points for FPFH"

    registration = o3d.pipelines.registration
    # RANSAC tries feature correspondences across the two clouds, without an initial pose (transformation matrix).
    # Example: it can propose how three camera points align with three LiDAR points.
    result = registration.registration_ransac_based_on_feature_matching(
        camera_coarse, lidar_coarse, camera_features, lidar_descriptors,
        True,  # Prefer mutual matches: camera A matches LiDAR B, and B also matches A.
        config.ransac_max_distance_m,
        registration.TransformationEstimationPointToPoint(False),  # Rigid motion (no scaling).
        3,  # Three matched points propose a candidate rigid transform.
        [
            # Reject candidates that change distances between corresponding points.
            # Example: a 1m camera edge cannot match a 0.7m LiDAR edge at the 0.9 ratio.
            registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            registration.CorrespondenceCheckerBasedOnDistance(config.ransac_max_distance_m),
        ],
        registration.RANSACConvergenceCriteria(config.ransac_max_iterations, config.ransac_confidence),
    )
    # This pose (transformation matrix) is RANSAC's initial guess from matching shapes.
    # It describes the camera frame relative to LiDAR as a 4x4 matrix:
    #
    #          [ r00  r01  r02  tx ]
    #   pose = [ r10  r11  r12  ty ]
    #          [ r20  r21  r22  tz ]
    #          [  0    0    0    1 ]
    #
    # The r entries describe rotation.
    # Example: a +90 degree Z rotation and no translation maps camera [1, 0, 0] to LiDAR [0, 1, 0].
    # tx, ty, tz are the camera origin in the LiDAR frame in meters:
    # - tx: translation along the x-axis in the LiDAR frame (meters).
    # - ty: translation along the y-axis in the LiDAR frame (meters).
    # - tz: translation along the z-axis in the LiDAR frame (meters).
    # To transform a point: [x_lidar, y_lidar, z_lidar, 1] = pose @ [x_camera, y_camera, z_camera, 1].
    # Example: no rotation and translation [1, 0, 0] maps camera [2, 3, 4] to LiDAR [3, 3, 4].
    # It is used as the starting guess for ICP.
    pose = np.asarray(result.transformation)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        return None, "invalid RANSAC transform"
    # Reject a candidate with too little overlap before asking ICP to refine it.
    if (not np.isfinite(result.fitness) or len(result.correspondence_set) < config.min_points
            or result.fitness < config.ransac_min_fitness):
        return None, f"insufficient RANSAC overlap (fitness {result.fitness:.2f})"
    return pose.copy(), None  # Maps camera message frame to LiDAR message frame.

# Refine the initial pose with ICP registration.
def register_camera(camera, lidar, initial_pose, config):
    # initial_pose is ICP's starting guess: the RANSAC result or a previously accepted pose (transformation matrix).
    # Example: RANSAC may estimate x = 1.00m, and ICP may refine it to x = 1.05m.
    # Too few points cannot provide a reliable set of spatial correspondences.
    if len(camera.points) < config.min_points or len(lidar.points) < config.min_points:
        return None, None, "too few points"

    # Move the camera cloud toward the fixed LiDAR cloud, starting from the initial pose.
    result = o3d.pipelines.registration.registration_icp(
        camera,  # Source: ICP estimates how this camera should move.
        lidar,  # Target: the LiDAR remains fixed in its own frame.
        config.icp_max_distance_m,  # Ignore potential matches farther apart than this.
        initial_pose,  # Start from the RANSAC estimate or the cached accepted pose.
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),  # Match XYZ points.
        # Stop after the iteration limit if ICP has not converged sooner.
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=config.icp_max_iterations),
    )
    # ICP returns the complete camera-to-LiDAR pose (transformation matrix), including the initial pose, not just the correction.
    # In the example above, pose contains x = 1.05 m, not only the 0.05 m correction.
    pose = np.asarray(result.transformation)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        return None, None, "invalid ICP transform"  # Never publish a non-finite pose.
    # Fitness measures how much of the source point cloud successfully matches the target point cloud.
    # It is the fraction of camera points with a LiDAR match within the limit.
    # For example, 40 matched camera points out of 200 give fitness 0.20; both counts and fitness must pass.
    if (not np.isfinite(result.fitness) or len(result.correspondence_set) < config.min_points
            or result.fitness < config.icp_min_fitness):
        return None, None, f"insufficient ICP overlap (fitness {result.fitness:.2f})"

    # Check whether ICP changed the starting estimate too much before accepting its result.
    # The final pose is already the final transformation matrix for transforming camera points to LiDAR.
    # This comparison does not reconstruct the final pose.
    # Measure how much ICP moved the camera away from the initial pose.
    translation = np.linalg.norm(pose[:3, 3] - initial_pose[:3, 3])
    # Compare the two orientations. Their relative rotation gives the correction angle in degrees.
    rotation = pose[:3, :3] @ initial_pose[:3, :3].T
    # A rotation matrix has trace = 1 + 2*cos(angle). Clamp for floating-point error.
    angle = np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))
    # Reject large corrections that are more likely to be a bad match than calibration.
    # For example, moving x from 1.0m to 1.6m exceeds the default 0.5m limit.
    if translation > config.icp_max_translation_m or angle > config.icp_max_rotation_deg:
        return None, None, f"ICP correction too large ({translation:.2f} m, {angle:.1f} deg)"
    camera.transform(pose)  # Apply the final pose so the points share the LiDAR frame.
    return camera, pose.copy(), None  # Return the final pose and transformed cloud.

# This step is only used for RViz visualization.
# Pack all registered clouds as XYZRGB PointCloud2 for RViz.
def pack_registered_cloud(clouds, header, config):
    count = sum(len(cloud.points) for _, cloud in clouds)
    points = np.empty(count, dtype=[("x", "<f4"), ("y", "<f4"),
                                   ("z", "<f4"), ("rgb", "<u4")])
    start = 0
    for name, cloud in clouds:
        xyz = np.asarray(cloud.points)
        end = start + len(xyz)
        points["x"][start:end], points["y"][start:end], points["z"][start:end] = xyz.T
        red, green, blue = getattr(config, f"{name}_rgb")  # Pick this sensor's configured color.
        points["rgb"][start:end] = (red << 16) | (green << 8) | blue
        start = end

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = count
    msg.fields = [
        PointField(name=name, offset=offset, datatype=datatype, count=1)
        for name, offset, datatype in (("x", 0, PointField.FLOAT32),
                                       ("y", 4, PointField.FLOAT32),
                                       ("z", 8, PointField.FLOAT32),
                                       ("rgb", 12, PointField.UINT32))
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * count
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


# Convert a ROS timestamp to seconds.
def stamp_seconds(stamp):
    # Use one unit so LiDAR and camera header timestamps can be compared directly.
    return stamp.sec + stamp.nanosec * 1e-9


# Main node for LiDAR and camera registration.
class LidarCameraRegistration(Node):
    def __init__(self):
        # The vehicle configuration runs this processing only on the Jetson.
        if not _is_jetson_link():
            raise RuntimeError("LiDAR and camera registration only runs on the configured Jetson (192.168.100.2)")
        super().__init__("lidar_camera_registration")  # Name the ROS node for its current function.
        self.config = load_registration_config(self)  # Read the launch YAML once at startup.

        latest_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.camera_lock = Lock()
        self.camera_clouds = {}
        self.camera_group = MutuallyExclusiveCallbackGroup()
        self.camera_poses = {}
        self.camera_failures = {}

        self.registered_pub = self.create_publisher(PointCloud2, REGISTERED_CLOUD_TOPIC, latest_scan)
        self.pcd_sub = self.create_subscription(
            PointCloud2, "/ouster/points", self.pcd_callback, latest_scan
        )
        self.camera_subs = []
        for camera in CAMERAS:
            topic = f"/zed_{camera}/zed_node/point_cloud/cloud_registered"
            self.camera_subs.append(self.create_subscription(
                PointCloud2, topic, lambda msg, name=camera: self.camera_callback(name, msg),
                latest_scan, callback_group=self.camera_group,
            ))
        self.get_logger().info("LiDAR and camera registration started on Jetson")

    def camera_callback(self, name, msg):
        with self.camera_lock:
            self.camera_clouds[name] = msg

    def snapshot_camera_clouds(self):
        with self.camera_lock:
            return self.camera_clouds.copy()

    def pcd_callback(self, msg):
        cameras = self.snapshot_camera_clouds()
        try:
            lidar = point_cloud_from_message(msg)
            self.publish_registered_cloud(lidar, msg.header, cameras)
        except Exception as exc:
            self.get_logger().error(f"Point cloud registration failed: {exc}",
                                    throttle_duration_sec=5.0)

    def publish_registered_cloud(self, lidar, header, cameras):
        lidar = lidar.voxel_down_sample(self.config.icp_voxel_size_m)
        clouds = [("lidar", lidar)]
        skipped = []
        lidar_time = stamp_seconds(header.stamp)  # Compare camera times against this scan.
        lidar_features = None

        for name in CAMERAS:
            msg = cameras.get(name)
            if msg is None:
                skipped.append(f"{name}: no cloud")
                continue
            camera_time = stamp_seconds(msg.header.stamp)
            if not lidar_time or not camera_time:
                skipped.append(f"{name}: missing timestamp")
                continue
            difference = abs(lidar_time - camera_time)
            if difference > self.config.max_camera_time_difference_s:
                skipped.append(f"{name}: timestamp difference {difference:.2f}s")
                continue
            # Frame IDs identify which camera and LiDAR coordinates a cached pose maps.
            if not header.frame_id or not msg.header.frame_id:
                skipped.append(f"{name}: missing frame ID")
                continue
            try:
                # Downsample camera XYZ to the same scale before estimating its ICP pose.
                camera = point_cloud_from_message(msg).voxel_down_sample(self.config.icp_voxel_size_m)
                frames = (header.frame_id, msg.header.frame_id)
                cached = self.camera_poses.get(name)
                if cached is not None and cached[:2] != frames:
                    # Never apply an estimate to different sensor coordinates.
                    self.camera_poses.pop(name)
                    self.camera_failures.pop(name, None)
                    cached = None

                if cached is None:
                    # The LiDAR features are the same target for every camera in this scan.
                    # For example, compute them once and reuse them for front and left.
                    if lidar_features is None:
                        lidar_features = prepare_fpfh(lidar, self.config)
                    # Search the complete pose; no camera mounting measurement or TF is used.
                    initial_pose, reason = estimate_initial_pose(camera, lidar_features, self.config)
                    if initial_pose is None:
                        skipped.append(f"{name}: {reason}")
                        continue
                else:
                    initial_pose = cached[2]  # Reuse the accepted, fixed sensor mounting estimate.

                registered, pose, reason = register_camera(camera, lidar, initial_pose, self.config)
                if registered is None:
                    if cached is not None:
                        failures = self.camera_failures.get(name, 0) + 1
                        self.camera_failures[name] = failures
                        if failures >= self.config.max_icp_failures:
                            # The scene or mounting may have changed; search globally again.
                            # With the default of three failures, discard the pose after the third failure.
                            self.camera_poses.pop(name)
                            self.camera_failures.pop(name, None)
                            reason += "; retrying FPFH/RANSAC on the next scan"
                    skipped.append(f"{name}: {reason}")  # Do not merge a rejected alignment.
                    continue
                self.camera_failures.pop(name, None)
                if cached is None:
                    # Keep this first refined estimate as an anchor; do not accumulate ICP drift.
                    self.camera_poses[name] = (*frames, pose)
                    # Log the anchor and its direction for inspecting the estimated mounting.
                    self.get_logger().info(
                        f"{name}: initialized with FPFH, RANSAC and ICP; "
                        f"{msg.header.frame_id} -> {header.frame_id} anchor:\n"
                        f"{np.array2string(pose, precision=4)}"
                    )
                clouds.append((name, registered))  # This camera is now in the LiDAR frame.
            except Exception as exc:
                skipped.append(f"{name}: {exc}")  # A cloud or registration error affects only this camera.

        # Publish even a partial result so available sensors remain visible in RViz.
        self.registered_pub.publish(pack_registered_cloud(clouds, header, self.config))
        # Report which cameras were included and why any others were omitted.
        registered_names = ", ".join(name for name, _ in clouds[1:]) or "none"
        self.get_logger().info(f"Registered ZED cameras: {registered_names}",
                               throttle_duration_sec=5.0)
        if skipped:
            self.get_logger().warning("Skipped ZED clouds: " + "; ".join(skipped),
                                      throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS before creating the node.
    node = None
    executor = None
    try:
        node = LidarCameraRegistration()
        # Separate threads let camera callbacks run during LiDAR processing.
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()  # Process sensor messages until shutdown.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C or ROS shutdown should exit without an error traceback.
    finally:
        # Stop callback threads before destroying their node and ROS context.
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
