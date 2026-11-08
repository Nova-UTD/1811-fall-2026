#!/usr/bin/env python3

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

# BLACKWALL feature 1: Targetless automated calibration of the ZED cameras to the LiDAR.

# Architecture: This node receives LiDAR and four ZED point clouds in separate callbacks while a timer checks for 
# time-matched data. At most once per configured interval (5 seconds in the supplied YAML), it crops clouds,
# removes LiDAR and ZED ground and wall-like planar patches for matching, uses Open3D FPFH and RANSAC to estimate an initial
# camera-to-LiDAR transform, then refines it with ICP. RViz can inspect the filtered inputs, or the
# full-resolution, ground-filtered inputs when the wall filter is disabled. Each camera has its own confirmations.
# When enabled, independent estimates must agree before the first pose is published;
# later attempts refine the accepted pose.
# The node combines each accepted cloud transform with TF to find the camera body mount relative to base_link
# and requests a URDF update from robot_state_publisher. Once accepted, the publisher supplies TF and the node saves 
# a copy of the URDF. Inputs are published in their original sensor frames, and accepted camera poses
# are also used to publish a colored overlay in the LiDAR frame.

# Register four ZED point clouds to the LiDAR scan on the Jetson.
import math
import os
import socket
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
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
from rcl_interfaces.srv import SetParameters
from sensor_msgs_py import point_cloud2
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from interactive_calibration import CalibrationAlgorithms, InteractiveCalibration


REGISTERED_CLOUD_TOPIC = "/lidar_perception/registered_points"  # RViz reads the merged cloud here.
CALIBRATION_INPUT_PREFIX = "/lidar_perception/calibration_input"
CAMERAS = ("front", "left", "rear", "right")  # Names used in the four ZED topic paths.
_FLOOR_PLANE_UNSET = object()

# Startup defaults: the launch file loads user settings from registration.yaml.
# See config/registration.yaml for an example of each setting.
class RegistrationConfig(NamedTuple):
    calibration_interval_s: float = 10.0
    max_cloud_age_s: float = 2.0
    cloud_diagnostics_enabled: bool = True
    interactive_calibration: bool = False
    lidar_box_min_m: tuple = (-5.0, -5.0, -0.5)
    lidar_box_max_m: tuple = (5.0, 5.0, 3.5)
    camera_box_min_m: tuple = (0.5, -2.0, -1.5)
    camera_box_max_m: tuple = (3.5, 2.0, 1.5)
    icp_voxel_size_m: float = 0.10
    fpfh_voxel_size_m: float = 0.20
    visualization_voxel_size_m: float = 0.0
    ground_filter_enabled: bool = True
    ground_distance_tolerance_m: float = 0.05
    lidar_ground_filter_enabled: bool = True
    lidar_ground_distance_tolerance_m: float = 0.08
    lidar_ground_search_height_m: float = 0.30
    lidar_ground_max_tilt_deg: float = 25.0
    lidar_ground_min_inliers: int = 50
    lidar_imu_max_time_difference_s: float = 0.30
    wall_filter_enabled: bool = True
    wall_min_height_m: float = 0.75
    wall_max_tilt_deg: float = 30.0
    wall_distance_tolerance_m: float = 0.03
    wall_edge_margin_m: float = 0.0
    fpfh_normal_radius_multiplier: float = 2.0
    fpfh_feature_radius_multiplier: float = 5.0
    ransac_max_distance_multiplier: float = 1.5
    ransac_max_iterations: int = 100000
    ransac_confidence: float = 0.999
    ransac_min_fitness: float = 0.10
    icp_max_distance_m: float = 0.15
    icp_max_iterations: int = 20
    min_points: int = 30
    icp_min_fitness: float = 0.10
    icp_max_rmse_m: float = 0.08
    max_camera_lidar_distance_m: float = 3.0
    icp_max_translation_m: float = 0.50
    icp_max_rotation_deg: float = 15.0
    initial_pose_confirmation_enabled: bool = True
    initial_pose_confirmations: int = 3
    pose_agreement_translation_m: float = 0.05
    pose_agreement_rotation_deg: float = 3.0
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


DEFAULT_CONFIG = RegistrationConfig() 


# Read the launch YAML once at startup.
# Settings are startup only, reject runtime changes.
def load_registration_config(node):
    values = {}
    for name in DEFAULT_CONFIG._fields:
        default = getattr(DEFAULT_CONFIG, name)
        parameter_default = list(default) if isinstance(default, tuple) else default
        value = node.declare_parameter(
            name, parameter_default, ParameterDescriptor(read_only=True),
        ).value
        values[name] = tuple(value) if isinstance(default, tuple) else value
    config = RegistrationConfig(**values)

    if not isinstance(config.initial_pose_confirmation_enabled, bool):
        raise ValueError("initial_pose_confirmation_enabled must be true or false")
    if not isinstance(config.cloud_diagnostics_enabled, bool):
        raise ValueError("cloud_diagnostics_enabled must be true or false")
    if not isinstance(config.interactive_calibration, bool):
        raise ValueError("interactive_calibration must be true or false")
    if not isinstance(config.ground_filter_enabled, bool):
        raise ValueError("ground_filter_enabled must be true or false")
    if not isinstance(config.lidar_ground_filter_enabled, bool):
        raise ValueError("lidar_ground_filter_enabled must be true or false")
    for sensor in ("lidar", "camera"):
        lower_name, upper_name = f"{sensor}_box_min_m", f"{sensor}_box_max_m"
        lower, upper = getattr(config, lower_name), getattr(config, upper_name)
        if (len(lower) != 3 or len(upper) != 3 or not np.isfinite(lower).all()
                or not np.isfinite(upper).all() or np.any(np.asarray(lower) >= np.asarray(upper))):
            raise ValueError(f"{lower_name} and {upper_name} must be finite XYZ bounds with min < max")
        if sensor == "camera" and lower[0] < 0:
            raise ValueError("camera_box_min_m forward bound must be at least zero")
    positive = (
        "icp_voxel_size_m", "fpfh_voxel_size_m", "fpfh_normal_radius_multiplier", "icp_max_rmse_m",
        "max_camera_lidar_distance_m",
        "fpfh_feature_radius_multiplier", "ransac_max_distance_multiplier", "icp_max_distance_m",
        "calibration_interval_s", "max_cloud_age_s", "wall_min_height_m", "wall_distance_tolerance_m",
        "ground_distance_tolerance_m", "lidar_ground_distance_tolerance_m",
        "lidar_ground_search_height_m", "lidar_imu_max_time_difference_s",
    )
    for name in positive:
        if not np.isfinite(getattr(config, name)) or getattr(config, name) <= 0:
            raise ValueError(f"{name} must be finite and greater than zero")
    for name in ("ransac_max_iterations", "icp_max_iterations", "max_icp_failures", "initial_pose_confirmations"):
        if getattr(config, name) < 1:
            raise ValueError(f"{name} must be at least 1")
    if config.min_points < 3:
        raise ValueError("min_points must be at least 3")
    if (not isinstance(config.lidar_ground_min_inliers, int)
            or isinstance(config.lidar_ground_min_inliers, bool) or config.lidar_ground_min_inliers < 3):
        raise ValueError("lidar_ground_min_inliers must be an integer of at least 3")
    for name in ("ransac_min_fitness", "icp_min_fitness"):
        if not np.isfinite(getattr(config, name)) or not 0.0 <= getattr(config, name) <= 1.0:
            raise ValueError(f"{name} must be between 0.0 and 1.0")
    if not np.isfinite(config.ransac_confidence) or not 0.0 < config.ransac_confidence <= 1.0:
        raise ValueError("ransac_confidence must be greater than 0.0 and at most 1.0")
    for name in ("icp_max_translation_m", "icp_max_rotation_deg", "max_camera_time_difference_s",
                 "pose_agreement_translation_m", "pose_agreement_rotation_deg", "wall_max_tilt_deg",
                 "wall_edge_margin_m", "visualization_voxel_size_m", "lidar_ground_max_tilt_deg"):
        if not np.isfinite(getattr(config, name)) or getattr(config, name) < 0:
            raise ValueError(f"{name} must be finite and at least zero")
    if config.icp_max_rotation_deg > 180.0:
        raise ValueError("icp_max_rotation_deg must be at most 180.0")
    if config.pose_agreement_rotation_deg > 180.0:
        raise ValueError("pose_agreement_rotation_deg must be at most 180.0")
    if config.wall_max_tilt_deg >= 90.0:
        raise ValueError("wall_max_tilt_deg must be less than 90.0")
    if config.lidar_ground_max_tilt_deg >= 90.0:
        raise ValueError("lidar_ground_max_tilt_deg must be less than 90.0")
    for name in ("lidar_rgb", "front_rgb", "left_rgb", "rear_rgb", "right_rgb"):
        color = getattr(config, name)
        if len(color) != 3 or any(not isinstance(channel, int) or not 0 <= channel <= 255
                                  for channel in color):
            raise ValueError(f"{name} must contain three integers between 0 and 255")
    return config


# Convert LiDAR or ZED PointCloud2 message to an Open3D XYZ point cloud.
def point_cloud_from_message(msg):
    # msg: a ROS PointCloud2 message from the LiDAR or a ZED camera.
    points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    # Turn ROS's named fields into one row of [x, y, z] per point.
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    # Remove invalid coordinates; for example, [NaN, 1, 2] cannot be matched to a surface.
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    # Open3D expects its own point-cloud object with double-precision coordinates.
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    return cloud


def cloud_diagnostics(name, msg, valid_count, cropped_count, voxel_count, cloud, walls, removed):
    # Describe the points used for matching, in the message's original frame.
    points = np.asarray(cloud.points)
    if len(points):
        def vector(values):
            return "[" + ", ".join(f"{value:.2f}" for value in values) + "]"

        geometry = (f"xyz_min_m={vector(points.min(axis=0))}, "
                    f"xyz_max_m={vector(points.max(axis=0))}, "
                    f"centroid_m={vector(points.mean(axis=0))}")
    else:
        geometry = "xyz_bounds=empty"
    wall_summary = "disabled" if walls is None else f"{walls} patches, {removed} points removed"
    stamp = msg.header.stamp
    return (f"Point cloud {name}: frame={msg.header.frame_id}, "
            f"stamp={stamp.sec}.{stamp.nanosec:09d}, raw={msg.width * msg.height}, "
            f"finite_xyz={valid_count}, cropped={cropped_count}, voxel={voxel_count}, "
            f"wall_filter={wall_summary}, matching={len(points)}, {geometry}")


def log_fpfh_diagnostics(logger, name, features, reason=None):
    # Log the FPFH descriptors already computed for a global search.
    if reason:
        logger.info(f"FPFH {name}: {reason}")
        return
    try:
        descriptors = np.asarray(features.data)
        if descriptors.ndim != 2 or descriptors.shape[0] != 33:
            raise ValueError("expected a 33-by-N descriptor matrix")
        finite = np.isfinite(descriptors).all(axis=0)
        valid = descriptors[:, finite]
        zero_count = int(np.count_nonzero(np.all(valid == 0, axis=0)))
        summary = (f"descriptors={descriptors.shape[1]}, finite={valid.shape[1]}, "
                   f"all_zero={zero_count}")
        if valid.shape[1]:
            mean = np.mean(valid, axis=1)
            spread = float(np.mean(np.std(valid, axis=1)))
            groups = ["[" + ", ".join(f"{value:.1f}" for value in mean[start:start + 11]) + "]"
                      for start in (0, 11, 22)]
            summary += f", mean_bin_std={spread:.2f}, mean_histograms=" + " ".join(groups)
        logger.info(f"FPFH {name}: {summary}")
    except Exception as exc:
        # Diagnostics must never stop a registration attempt.
        logger.info(f"FPFH {name}: descriptor summary unavailable ({exc})")


def remove_wall_patches(cloud, up, config):
    # cloud: a downsampled cloud in its own sensor frame, used only for matching.
    # up: the approximate up direction in that same frame; config: wall thresholds.
    # Return the remaining cloud, wall count, and number of removed points.
    if len(cloud.points) < config.min_points:
        return cloud, 0, 0
    up = np.asarray(up, dtype=float)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-9:
        raise ValueError("wall filter needs a valid up direction")
    up = up / np.linalg.norm(up)

    # Open3D needs normals to detect planar patches. Work on a copy so the
    # input points and their attributes remain available to the caller.
    detection_cloud = o3d.geometry.PointCloud(cloud)
    detection_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=config.fpfh_normal_radius_m, max_nn=30))
    patches = detection_cloud.detect_planar_patches(
        normal_variance_threshold_deg=40, coplanarity_deg=70, outlier_ratio=0.75,
        min_plane_edge_length=config.wall_min_height_m, min_num_points=config.min_points,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))

    points = np.asarray(cloud.points)
    remove = np.zeros(len(points), dtype=bool)
    wall_count = 0
    for patch in patches:
        # A wall normal is nearly horizontal; a floor normal points up or down.
        # Example: a vertical wall has dot(normal, up) = 0, a floor is near 1.
        normal = patch.R[:, 2]
        if abs(np.dot(normal, up)) > math.sin(math.radians(config.wall_max_tilt_deg)):
            continue
        # Ignore small vertical patches such as a narrow object or a sign.
        height = np.dot(np.abs((patch.R.T @ up)[:2]), patch.extent[:2])
        if height < config.wall_min_height_m:
            continue
        wall_count += 1
        # Only remove points inside the detected patch footprint and near its
        # plane. A 3 cm tolerance includes noisy wall points near the plane.
        half_extent = patch.extent / 2.0
        half_extent[:2] += config.wall_edge_margin_m
        half_extent[2] = max(half_extent[2], config.wall_distance_tolerance_m)
        local_points = (points - patch.center) @ patch.R
        remove |= np.all(np.abs(local_points) <= half_extent, axis=1)

    removed = int(np.count_nonzero(remove))
    if not removed:
        return cloud, wall_count, 0
    return cloud.select_by_index(np.flatnonzero(~remove).tolist()), wall_count, removed

# FPFH: Fast Point Feature Histograms - describe local shapes without needing to know the sensor's mounting pose.
# These features help RANSAC estimate each camera's initial position and orientation relative to LiDAR.
def prepare_fpfh(cloud, config):
    # cloud: an Open3D point cloud from one sensor, in that sensor's cloud frame.
    # config: settings for downsampling, normal estimation, and FPFH features.
    # A coarse cloud reduces computation cost by downsampling the original cloud.
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
    # max_nn: max number of nearest neighbors to consider for each point.
    features = o3d.pipelines.registration.compute_fpfh_feature(
        coarse, o3d.geometry.KDTreeSearchParamHybrid(radius=config.fpfh_feature_radius_m, max_nn=100),
    )
    return coarse, features


def overlap_rejection(result, stage, min_fitness, min_points):
    # Both RANSAC and ICP require enough correspondences and a finite fitness.
    matches = len(result.correspondence_set)
    if not np.isfinite(result.fitness) or matches < min_points or result.fitness < min_fitness:
        return (f"insufficient {stage} overlap (fitness {result.fitness:.2f}, "
                f"minimum {min_fitness:.2f}; matches {matches}, minimum {min_points})")
    return None


# Use computed FPFH features and RANSAC (Random Sample Consensus) to estimate camera coordinates in LiDAR coordinates.
def estimate_initial_pose(camera, lidar_features, config, metrics=None, report_features=None):
    # camera: the Open3D camera cloud to align with the LiDAR.
    # lidar_features: the downsampled LiDAR cloud and its FPFH descriptors, as a pair.
    # config: FPFH and RANSAC settings, including match-distance and fitness limits.
    # metrics: optional dictionary for reporting the actual score and match count.
    # report_features: optional callback before the potentially slow RANSAC search.
    lidar_coarse, lidar_descriptors = lidar_features
    # Skip camera feature computation when the LiDAR cannot be matched.
    if lidar_coarse is None:
        if report_features is not None:
            report_features(None, "not computed: too few LiDAR points after FPFH downsampling")
        return None, "too few points for FPFH"
    camera_coarse, camera_features = prepare_fpfh(camera, config)
    if report_features is not None:
        reason = "not computed: too few points after FPFH downsampling" if camera_coarse is None else None
        report_features(camera_features, reason)
    if camera_coarse is None:
        return None, "too few points for FPFH"

    registration = o3d.pipelines.registration
    # RANSAC tries feature correspondences across the two clouds, without an initial pose (transformation matrix).
    # Example: it can propose how three camera points align with three LiDAR points.
    # registration.registration_ransac_based_on_feature_matching: RANSAC based on feature matching computed by FPFH.
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
    if metrics is not None:
        metrics.update(fitness=float(result.fitness), matches=len(result.correspondence_set))
    # result is the RANSAC result containing the transformation matrix and fitness score.
    # This pose (transformation matrix) is RANSAC's initial guess from matching shapes.
    # It describes the camera frame relative to LiDAR as a 4x4 matrix:
    #
    #          [ r00  r01  r02  tx ]
    #   pose = [ r10  r11  r12  ty ]
    #          [ r20  r21  r22  tz ]
    #          [  0    0    0    1 ]
    # The r entries describe rotation.
    # Example: a +90 degree Z rotation and no translation maps camera [1, 0, 0] to LiDAR [0, 1, 0].
    # tx, ty, tz are the camera origin in the LiDAR frame in meters:
    # - tx: translation along the x-axis in the LiDAR frame (meters).
    # - ty: translation along the y-axis in the LiDAR frame (meters).
    # - tz: translation along the z-axis in the LiDAR frame (meters).
    # In short, look at the last column for translation and the top-left 3×3 for rotation.
    # To transform a point from camera frame to LiDAR frame: [x_lidar, y_lidar, z_lidar, 1] = pose @ [x_camera, y_camera, z_camera, 1].
    # Example: no rotation and translation [1, 0, 0] maps camera [2, 3, 4] to LiDAR [3, 3, 4].
    # It is used as the starting guess for ICP.
    pose = np.asarray(result.transformation)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        return None, "invalid RANSAC transform"
    # Reject a candidate with too little overlap before asking ICP to refine it.
    reason = overlap_rejection(result, "RANSAC", config.ransac_min_fitness, config.min_points)
    if reason:
        return None, reason
    return pose.copy(), None  # Maps camera message frame to LiDAR message frame.

# Compare two 4x4 camera-to-LiDAR transforms.
def pose_difference(pose, reference):
    # Return their position distance in meters and rotation difference in degrees.
    # The last column contains XYZ position; measure the distance between them.
    # Example: [1.00, 0, 0] and [1.03, 0, 0] differ by 0.03 m.
    translation = np.linalg.norm(pose[:3, 3] - reference[:3, 3])
    # Compare the 3x3 rotations to find the smallest angle between orientations.
    rotation = pose[:3, :3] @ reference[:3, :3].T
    # Clamp the cosine to [-1, 1] so rounding cannot make arccos invalid.
    angle = np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))
    return float(translation), float(angle)

# Decide whether a camera's initial pose has enough independent, agreeing estimates.
def confirm_initial_pose(pending, name, frames, pose, lidar_stamp, camera_stamp, config):
    # pending: saved estimates for each camera; this function updates it in place.
    # name: camera name; frames: LiDAR and camera cloud frame IDs.
    # pose: this attempt's camera-to-LiDAR matrix; stamps identify the two scans.
    # config: required count and allowed position (m) and rotation (deg) differences.
    # Returns (confirmed, reason); confirmation does not itself update the URDF.
    # Identify the scan pair by its timestamps, including nanoseconds.
    observation = ((lidar_stamp.sec, lidar_stamp.nanosec),
                   (camera_stamp.sec, camera_stamp.nanosec))
    previous = pending.get(name)
    poses, observations = [], []
    reset_reason = None
    # Only reuse this camera's saved estimates when both cloud frames still match.
    if previous is not None and previous["frames"] == frames:
        # Each pose corresponds to a pair of scan timestamps at the same list index.
        poses = previous["poses"]
        observations = previous["observations"]
        # Reusing either scan is not an independent confirmation.
        if any(observation[0] == old[0] or observation[1] == old[1] for old in observations):
            # A confirmed pose may need another URDF/TF update attempt. It can
            # be retried without counting this same scan a second time, provided
            # the newly computed pose still agrees with all prior estimates.
            if len(poses) >= config.initial_pose_confirmations and all(
                    translation <= config.pose_agreement_translation_m
                    and angle <= config.pose_agreement_rotation_deg
                    for translation, angle in (pose_difference(pose, earlier) for earlier in poses)):
                return True, f"{len(poses)} independent estimates agree"
            return False, f"waiting for new LiDAR and camera scans ({len(poses)}/{config.initial_pose_confirmations})"
        # The new pose must agree with every saved pose, not just the latest one.
        for earlier in poses:
            translation, angle = pose_difference(pose, earlier)
            if (translation > config.pose_agreement_translation_m
                    or angle > config.pose_agreement_rotation_deg):
                poses, observations = [], []  # Start again from this different estimate.
                reset_reason = (f"estimate changed {translation:.2f} m, {angle:.1f} deg; "
                                "restarting confirmations")
                break
    # Save a copy of the new pose and its scan pair, keeping at most the required count.
    poses = [*poses, pose.copy()][-config.initial_pose_confirmations:]
    observations = [*observations, observation][-config.initial_pose_confirmations:]
    # Keep the confirmation until robot_state_publisher accepts the URDF.
    # Otherwise a missing TF or rejected update would make 3/3 restart at 1/3.
    pending[name] = {"frames": frames, "poses": poses, "observations": observations}
    if len(poses) >= config.initial_pose_confirmations:
        return True, f"{len(poses)} independent estimates agree"
    reason = reset_reason or "waiting for consistent estimates"
    return False, f"{reason} ({len(poses)}/{config.initial_pose_confirmations})"


# ICP: Iterative Closest Point - refine the initial pose provided by RANSAC with ICP registration.
def register_camera(camera, lidar, initial_pose, config, metrics=None):
    # camera: the Open3D source cloud, lidar: the Open3D target cloud.
    # config: ICP settings and limits for accepting the alignment.
    # metrics: optional dictionary filled with fitness, match count, and error for logging.
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
    if metrics is not None:
        metrics.update(fitness=float(result.fitness), rmse=float(result.inlier_rmse),
                       matches=len(result.correspondence_set))
    # Fitness measures how much of the source point cloud successfully matches the target point cloud.
    # It is the fraction of camera points with a LiDAR match within the limit.
    # For example, 40 matched camera points out of 200 give fitness 0.20; both counts and fitness must pass.
    reason = overlap_rejection(result, "ICP", config.icp_min_fitness, config.min_points)
    if reason:
        return None, None, reason
    # RMSE summarizes matched distances, giving larger errors more weight.
    # Example: 0.12 m exceeds a 0.08 m limit, even if enough points matched.
    if not np.isfinite(result.inlier_rmse) or result.inlier_rmse > config.icp_max_rmse_m:
        return None, None, f"ICP match error too high (RMSE {result.inlier_rmse:.3f} m)"
    # The translation is the camera cloud origin in the LiDAR frame. A camera
    # on this vehicle should not be several metres away from its LiDAR.
    # This is a broad physical limit, not an initial camera pose.
    camera_distance = np.linalg.norm(pose[:3, 3])
    if camera_distance > config.max_camera_lidar_distance_m:
        return None, None, (f"camera-to-LiDAR distance too large ({camera_distance:.2f} m, "
                            f"maximum {config.max_camera_lidar_distance_m:.2f} m)")

    # Check whether ICP changed the starting estimate too much before accepting its result.
    # The final pose is already the final transformation matrix for transforming camera points to LiDAR.
    # This comparison does not reconstruct the final pose.
    # Measure how much ICP moved the camera away from the initial pose.
    translation, angle = pose_difference(pose, initial_pose)
    # Reject large corrections that are more likely to be a bad match than calibration.
    # For example, moving x from 1.0m to 1.6m exceeds the default 0.5m limit.
    if translation > config.icp_max_translation_m or angle > config.icp_max_rotation_deg:
        return None, None, f"ICP correction too large ({translation:.2f} m, {angle:.1f} deg)"
    camera.transform(pose)  # Apply the final pose so the points share the LiDAR frame.
    return camera, pose.copy(), None  # Return the final pose and transformed cloud.

# This step is only used for RViz visualization.
# Pack all registered clouds as XYZRGB PointCloud2 for RViz.
def pack_registered_cloud(clouds, header, config):
    # clouds: (sensor name, Open3D cloud) pairs, with all points in the LiDAR frame.
    # header: the LiDAR message header, providing the output frame and timestamp.
    # config: registration settings containing the RGB color for each sensor.
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
# It is used to compare LiDAR and camera timestamps.
def stamp_seconds(stamp):
    # stamp: a ROS timestamp with separate sec and nanosec fields.
    # Use one unit so LiDAR and camera header timestamps can be compared directly.
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_key(stamp):
    # Integer timestamps avoid losing nanoseconds when pairing a plane and cloud.
    return stamp.sec, stamp.nanosec


def remove_zed_ground(cloud, cloud_header, plane, tolerance_m):
    # The wrapper's floor pose uses the plane center as position and +Z as normal.
    # It is only valid for the exact image and camera frame used by this cloud.
    if plane is None:
        raise ValueError("no ZED floor plane for this cloud timestamp")
    if stamp_key(plane.header.stamp) != stamp_key(cloud_header.stamp) or not any(stamp_key(cloud_header.stamp)):
        raise ValueError("ZED floor plane timestamp does not match the camera cloud")
    if not cloud_header.frame_id or plane.header.frame_id != cloud_header.frame_id:
        raise ValueError("ZED floor plane frame does not match the camera cloud")
    position, orientation = plane.pose.position, plane.pose.orientation
    center = np.array([position.x, position.y, position.z], dtype=float)
    q = np.array([orientation.x, orientation.y, orientation.z, orientation.w], dtype=float)
    length = np.linalg.norm(q)
    if not np.isfinite(center).all() or not np.isfinite(length) or length < 1e-12:
        raise ValueError("ZED floor plane contains an invalid center or orientation")
    x, y, z, w = q / length
    normal = np.array([2 * (x*z + w*y), 2 * (y*z - w*x), 1 - 2 * (x*x + y*y)])
    xyz = np.asarray(cloud.points)
    remove = np.abs((xyz - center) @ normal) <= tolerance_m
    return cloud.select_by_index(np.flatnonzero(~remove).tolist()), int(np.count_nonzero(remove))


def transform_matrix(transform):
    # transform: a ROS Transform containing translation and quaternion rotation.
    # The input already contains a quaternion for rotation and a separate translation.
    # Convert the quaternion to a rotation matrix, then combine it with the translation to map a point from the child frame into the parent frame.

    # A unit quaternion (x, y, z, w) represents rotation around a unit-length axis.
    # For an axis (ax, ay, az) and angle theta:
    #   (x, y, z) = (ax, ay, az) * sin(theta / 2), and w = cos(theta / 2).
    # Example: for 90 degrees around Z, the axis is (0, 0, 1) and half the angle is 45 degrees. 
    # sin(45 degrees) and cos(45 degrees) are both about 0.7071, giving the quaternion (0, 0, 0.7071, 0.7071).
    # With zero rotation, sin(0) = 0 and cos(0) = 1, so the quaternion is (0, 0, 0, 1).
    # Quaternion multiplication q * (vx, vy, vz, 0) * inverse(q) rotates a vector.
    # The equivalent 3x3 matrix below performs that same rotation on point coordinates.
    q = transform.rotation
    x, y, z, w = np.asarray([q.x, q.y, q.z, q.w], dtype=float)

    # A rotation quaternion must have length 1. Reject invalid values before dividing.
    # Example: (0, 0, 0, 0) has zero length and cannot describe a rotation.
    length = np.linalg.norm([x, y, z, w])
    if not np.isfinite(length) or length < 1e-12:
        raise ValueError("TF contains an invalid quaternion")
    # Normalize to unit length so the rotation formula produces a valid matrix.
    x, y, z, w = np.asarray([x, y, z, w]) / length

    # A 4x4 rigid transform combines rotation R and translation t in meters:
    # [ r00 r01 r02 tx ]
    # [ r10 r11 r12 ty ]    parent_point = matrix @ [child_x, child_y, child_z, 1]
    # [ r20 r21 r22 tz ]
    # [  0   0   0   1 ]
    # Start with an identity matrix, which leaves the point unchanged.
    matrix = np.eye(4)

    # Convert the normalized quaternion into the top-left 3x3 rotation block.
    # This is the standard conversion from a unit quaternion (x, y, z, w) to a 3x3 rotation matrix.
    # Example: a 90-degree rotation around Z maps [1, 0, 0] to [0, 1, 0].
    matrix[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    # Put the child frame's position in the parent frame in the last column.
    # With no rotation, a translation of [1, 0, 0] m maps [2, 3, 4] to [3, 3, 4].
    t = transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]

    # Reject NaN or infinity, including invalid translation values, before using TF.
    if not np.isfinite(matrix).all():
        raise ValueError("TF contains a non-finite transform")
    return matrix  # The complete child-to-parent transformation.


def crop_calibration_cloud(cloud, tf_buffer, cloud_frame, reference_frame, lower, upper):
    # Test the box in base_link or the camera body, but retain original cloud coordinates.
    reference_from_cloud = (np.eye(4) if cloud_frame == reference_frame else transform_matrix(
        tf_buffer.lookup_transform(reference_frame, cloud_frame, Time()).transform))
    xyz = np.asarray(cloud.points)
    reference_xyz = xyz @ reference_from_cloud[:3, :3].T + reference_from_cloud[:3, 3]
    keep = np.all((reference_xyz >= lower) & (reference_xyz <= upper), axis=1)
    return cloud.select_by_index(np.flatnonzero(keep).tolist())


def lidar_up_from_imu(samples, cloud_header, tf_buffer, now, config):
    # Average nearby stationary IMU measurements, then express up in base_link.
    cloud_stamp = stamp_key(cloud_header.stamp)
    cloud_ns = cloud_stamp[0] * 1000000000 + cloud_stamp[1]
    if not cloud_ns:
        raise ValueError("LiDAR cloud has no timestamp for IMU matching")
    limit_ns = int(config.lidar_imu_max_time_difference_s * 1000000000)
    accelerations, frames, stamps = [], set(), set()
    for msg, received in samples:
        seconds, nanoseconds = stamp_key(msg.header.stamp)
        stamp_ns = seconds * 1000000000 + nanoseconds
        if (not stamp_ns or stamp_ns in stamps or not msg.header.frame_id
                or abs(stamp_ns - cloud_ns) > limit_ns or now - received > config.max_cloud_age_s):
            continue
        if msg.linear_acceleration_covariance[0] == -1 or msg.angular_velocity_covariance[0] == -1:
            continue
        acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                                 msg.linear_acceleration.z], dtype=float)
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                                     msg.angular_velocity.z], dtype=float)
        # Near 1 g and little rotation are necessary, but not sufficient, to
        # establish gravity. Calibration should be performed while parked.
        if (not np.isfinite(acceleration).all() or not np.isfinite(angular_velocity).all()
                or not 8.0 <= np.linalg.norm(acceleration) <= 11.5
                or np.linalg.norm(angular_velocity) > 0.20):
            continue
        accelerations.append(acceleration)
        frames.add(msg.header.frame_id)
        stamps.add(stamp_ns)
    if len(accelerations) < 3:
        raise ValueError("need at least 3 fresh, stationary /ouster/imu samples near the LiDAR timestamp")
    if len(frames) != 1:
        raise ValueError("matched /ouster/imu samples have different frames")
    up = np.mean(accelerations, axis=0)
    magnitude = np.linalg.norm(up)
    if not np.isfinite(magnitude) or magnitude < 8.0:
        raise ValueError("matched /ouster/imu accelerations do not give a consistent up direction")
    frame = frames.pop()
    if frame != "base_link":
        transform = tf_buffer.lookup_transform("base_link", frame, Time()).transform
        up = transform_matrix(transform)[:3, :3] @ up
    return up / np.linalg.norm(up), len(accelerations)


def remove_lidar_ground(cloud, cloud_frame, tf_buffer, up, config):
    # Fit low, near-horizontal planes in base_link, but keep cloud coordinates
    # in the original LiDAR frame for matching and publication.
    if not cloud_frame:
        raise ValueError("LiDAR cloud has no frame ID")
    base_from_cloud = (np.eye(4) if cloud_frame == "base_link" else transform_matrix(
        tf_buffer.lookup_transform("base_link", cloud_frame, Time()).transform))
    xyz = np.asarray(cloud.points)
    base_xyz = xyz @ base_from_cloud[:3, :3].T + base_from_cloud[:3, 3]
    low = np.abs(base_xyz[:, 2]) <= config.lidar_ground_search_height_m
    candidates = o3d.geometry.PointCloud()
    candidates.points = o3d.utility.Vector3dVector(base_xyz[low])
    candidates = candidates.voxel_down_sample(config.icp_voxel_size_m)
    minimum = config.lidar_ground_min_inliers
    cos_max_tilt = math.cos(math.radians(config.lidar_ground_max_tilt_deg))

    # A wall or low object can be the largest plane. Inspect several candidates
    # and prefer a valid plane closest to the nominal ground at base_link.
    best = None
    for _ in range(4):
        if len(candidates.points) < minimum:
            break
        plane, inliers = candidates.segment_plane(
            distance_threshold=config.lidar_ground_distance_tolerance_m,
            ransac_n=3, num_iterations=500)
        if len(inliers) < minimum:
            break
        normal = np.asarray(plane[:3], dtype=float)
        length = np.linalg.norm(normal)
        if np.isfinite(length) and length > 1e-9 and np.isfinite(plane[3]):
            normal /= length
            offset = plane[3] / length
            if np.dot(normal, up) < 0:
                normal, offset = -normal, -offset
            alignment = float(np.clip(np.dot(normal, up), -1.0, 1.0))
            height = -offset / normal[2] if abs(normal[2]) > 1e-9 else float("inf")
            if alignment >= cos_max_tilt and abs(height) <= config.lidar_ground_search_height_m:
                score = (abs(height), -len(inliers))
                if best is None or score < best[0]:
                    best = (score, normal, offset, len(inliers), height, math.degrees(math.acos(alignment)))
        candidates = candidates.select_by_index(inliers, invert=True)
    if best is None:
        raise ValueError("no reliable LiDAR ground plane near base_link")
    _, normal, offset, inliers, height, tilt = best
    # Remove the estimated road and everything below it. The positive tolerance
    # also removes noisy road points just above the plane.
    remove = (base_xyz @ normal + offset) <= config.lidar_ground_distance_tolerance_m
    return cloud.select_by_index(np.flatnonzero(~remove).tolist()), int(np.count_nonzero(remove)), inliers, height, tilt


def up_in_cloud_frame(tf_buffer, cloud_frame, upright_frame):
    # tf_buffer: ROS transforms; cloud_frame: point coordinates being filtered.
    # upright_frame: base_link for LiDAR or the ZED camera's own body frame.
    # Rotate the reference frame's +Z direction into cloud coordinates. The
    # camera body is assumed roughly upright; no base-to-camera pose is used.
    if cloud_frame == upright_frame:
        return np.array([0.0, 0.0, 1.0])
    transform = tf_buffer.lookup_transform(cloud_frame, upright_frame, Time())
    return transform_matrix(transform.transform)[:3, 2]

# ICP gives the camera cloud’s position and orientation relative to the LiDAR cloud.
# The URDF needs the camera body’s position and orientation relative to the vehicle. 
# This function connects those frames.
def camera_mount_matrix(tf_buffer, lidar_frame, camera_frame, camera_link, camera_to_lidar):
    # tf_buffer: stored ROS TF transforms between the sensor and vehicle frames.
    # lidar_frame: frame ID in the LiDAR cloud message, for example "os_lidar".
    # camera_frame: frame ID in the ZED point cloud message.
    # camera_link: camera body frame, for example "zed_front_camera_link".
    # camera_to_lidar: 4x4 ICP transform mapping camera-cloud points into the LiDAR frame (returned by ICP).
    # Convert ICP's camera-cloud -> LiDAR-cloud pose into a base -> camera-body mount.
    # Example: the LiDAR message may be in os_lidar, not os_sensor. 
    # The driver supplies os_sensor -> os_lidar, while the URDF supplies base_link -> os_sensor.
    base_from_lidar = (np.eye(4) if lidar_frame == "base_link" else transform_matrix(
        tf_buffer.lookup_transform("base_link", lidar_frame, Time()).transform))
    # The ZED cloud can be in a lens optical frame rather than the camera body.
    # Its own driver supplies this internal transform, not the camera mounting pose.
    camera_from_body = (np.eye(4) if camera_frame == camera_link else transform_matrix(
        tf_buffer.lookup_transform(camera_frame, camera_link, Time()).transform))
        
    # camera_from_body: camera body frame → camera point-cloud frame.
    # camera_to_lidar: camera point-cloud frame → LiDAR point-cloud frame, using the Open3D result.
    # base_from_lidar: LiDAR point-cloud frame → base_link.
    return base_from_lidar @ camera_to_lidar @ camera_from_body


def update_camera_joints(description, mounts):
    # description: the vehicle's expanded URDF as an XML string.
    # mounts: camera names mapped to 4x4 transforms from camera body to base_link.
    # Update only accepted camera joints in the expanded vehicle URDF.
    robot = ET.fromstring(description)
    if robot.tag != "robot" or robot.get("name") != "vehicle_1811":
        raise ValueError("robot_description is not the vehicle_1811 URDF")
    for name, matrix in mounts.items():
        matrix = np.asarray(matrix)
        # Check if the matrix is a valid rigid camera mount
        if matrix.shape != (4, 4):
            raise ValueError(f"{name}: invalid rigid camera mount")
        rotation = matrix[:3, :3]
        # Check if the rotation matrix is a valid rotation matrix
        if (not np.isfinite(matrix).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4)
                or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)):
            raise ValueError(f"{name}: invalid rigid camera mount")
        joint = robot.find(f"./joint[@name='zed_{name}_mount']")
        # Check if the joint is valid
        if (joint is None or joint.find("parent") is None or joint.find("child") is None
                or joint.find("parent").get("link") != "base_link"
                or joint.find("child").get("link") != f"zed_{name}_camera_link"):
            raise ValueError(f"{name}: expected base_link -> zed_{name}_camera_link joint not found")

        # URDF uses fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
        # We convert the rotation matrix into roll, pitch, and yaw for the URDF.
        # The matrix describes the camera body’s orientation relative to base_link.
        #             cam X     cam Y     cam Z
        #           ┌                            ┐
        # base X    │ R[0,0]    R[0,1]    R[0,2] │
        # base Y    │ R[1,0]    R[1,1]    R[1,2] │
        # base Z    │ R[2,0]    R[2,1]    R[2,2] │
        #           └                            ┘
        horizontal = math.hypot(rotation[0, 0], rotation[1, 0])
        # Combine the vertical and horizontal components to find pitch in radians.
        # In this convention rotation[2, 0] = -sin(pitch), so negate it.
        # Example: if the x-axis has no vertical component, pitch is 0.
        pitch = math.atan2(-rotation[2, 0], horizontal)
        # Near +/-90 degrees pitch, horizontal is almost 0 and roll and yaw are coupled (gimbal lock). 
        # The small threshold accounts for rounding.
        if horizontal < 1e-9:
            # Choose yaw = 0 and calculate a roll that represents the orientation.
            roll = math.atan2(-rotation[1, 2], rotation[1, 1])
            yaw = 0.0
        else:
            # Otherwise, the remaining matrix entries give roll and yaw in radians.
            # atan2 uses both signs to choose the correct angle quadrant.
            # Example: a pure +90-degree Z rotation gives roll=0 and yaw=pi/2.
            roll = math.atan2(rotation[2, 1], rotation[2, 2])
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        # Find the joint's URDF origin element, or create one if it is missing.
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        # The last matrix column is the camera body's position in base_link, in meters.
        # Example: [1, 2, 3] becomes xyz="1 2 3"; values use up to 12 significant digits.
        origin.set("xyz", " ".join(f"{v:.12g}" for v in matrix[:3, 3]))
        # Write the three angles in radians in roll, pitch, yaw order.
        # Example: angles (0, 0, pi/2) become rpy="0 0 1.57079632679".
        origin.set("rpy", " ".join(f"{v:.12g}" for v in (roll, pitch, yaw)))
        joint.set("type", "fixed")  # robot_state_publisher now publishes this calibrated TF.
    return ET.tostring(robot, encoding="unicode")


# Atomically save the accepted expanded URDF outside the Git checkout.
def save_calibrated_urdf(description, path):
    # description: the accepted, expanded URDF as an XML string.
    # path: pathlib.Path for the file where the URDF will be saved.
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".calibration-", delete=False) as stream:
            temporary = stream.name
            stream.write(description)
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


# Main node for LiDAR and camera registration on the Jetson.
class LidarCameraRegistration(Node):
    def __init__(self):
        if not _is_jetson_link():
            raise RuntimeError("LiDAR and camera registration only runs on the Jetson (192.168.100.2)")
        super().__init__("lidar_camera_registration")
        self.config = load_registration_config(self)

        # Separate callbacks keep receiving current clouds while FPFH/RANSAC runs.
        self.camera_group = MutuallyExclusiveCallbackGroup()
        self.lidar_group = MutuallyExclusiveCallbackGroup()
        self.calibration_group = MutuallyExclusiveCallbackGroup()
        self.description_group = MutuallyExclusiveCallbackGroup()
        self.data_lock = Lock()
        self.state_lock = Lock()
        self.latest_lidar = None  # (message, local receipt time)
        # Measure existing subscriptions; do not duplicate large cloud streams for the UI.
        self.cloud_receipts = {name: deque(maxlen=120) for name in ("lidar", *CAMERAS)}
        self.imu_samples = deque(maxlen=512)  # (message, local receipt time)
        self.camera_clouds = {}
        # A small history allows independently delivered plane and cloud messages
        # to be paired by their exact camera image timestamp.
        self.floor_planes = {name: deque(maxlen=64) for name in CAMERAS}
        self.camera_poses = {}  # name -> (LiDAR frame, cloud frame, camera-to-LiDAR matrix)
        self.camera_anchors = {}  # The accepted global pose, used to limit later drift.
        self.pending_candidates = {}  # Unpublished global estimates awaiting confirmation.
        self.camera_mounts = {}  # name -> calibrated base-from-camera-body matrix
        self.camera_failures = {}
        self.retry_global = set()
        self.robot_description = None
        self.update_in_flight = False
        self.pending_update = None
        self.update_started = 0.0
        self.next_calibration = 0.0
        self.output_urdf = Path.home() / ".ros/lidar_camera_calibration/vehicle_1811_calibrated.urdf"

        latest_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.registered_pub = self.create_publisher(PointCloud2, REGISTERED_CLOUD_TOPIC, latest_scan)
        self.input_pubs = {
            name: self.create_publisher(PointCloud2, f"{CALIBRATION_INPUT_PREFIX}/{name}", latest_scan)
            for name in ("lidar", *CAMERAS)
        }
        self.pcd_sub = self.create_subscription(
            PointCloud2, "/ouster/points", self.pcd_callback, latest_scan,
            callback_group=self.lidar_group,
        )
        self.imu_sub = None
        if self.config.lidar_ground_filter_enabled:
            imu_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.imu_sub = self.create_subscription(
                Imu, "/ouster/imu", self.imu_callback, imu_qos,
                callback_group=self.lidar_group,
            )
        self.camera_subs = []
        self.floor_subs = []
        floor_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        for camera in CAMERAS:
            topic = f"/zed_{camera}/zed_node/point_cloud/cloud_registered"
            self.camera_subs.append(self.create_subscription(
                PointCloud2, topic, lambda msg, name=camera: self.camera_callback(name, msg),
                latest_scan, callback_group=self.camera_group,
            ))
            if self.config.ground_filter_enabled:
                self.floor_subs.append(self.create_subscription(
                    PoseStamped, f"/zed_{camera}/zed_node/floor_plane",
                    lambda msg, name=camera: self.floor_callback(name, msg), floor_qos,
                    callback_group=self.camera_group,
                ))

        # The vehicle publisher remains the only authority for base -> camera mounts.
        # Its description topic is latched, so this also works if Jetson starts later.
        self.description_sub = self.create_subscription(
            String, "/robot_description", self.description_callback, latched,
            callback_group=self.description_group,
        )
        self.description_client = self.create_client(
            SetParameters, "/robot_state_publisher/set_parameters",
            callback_group=self.description_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.hub = None
        if self.config.interactive_calibration:
            algorithms = CalibrationAlgorithms(
                prepare_fpfh, log_fpfh_diagnostics, estimate_initial_pose, register_camera,
                camera_mount_matrix, confirm_initial_pose, update_camera_joints, save_calibrated_urdf)
            self.hub = InteractiveCalibration(self, CAMERAS, algorithms)
        # Poll for the first usable scan promptly; later attempts use the configured interval.
        self.calibration_timer = self.create_timer(0.5, self.calibration_tick,
                                                    callback_group=self.calibration_group)
        self.get_logger().info(
            f"LiDAR and camera registration started on Jetson: ZED ground filter "
            f"{self.config.ground_filter_enabled}, LiDAR ground filter {self.config.lidar_ground_filter_enabled}, "
            f"wall filter {self.config.wall_filter_enabled}, "
            f"RANSAC minimum fitness {self.config.ransac_min_fitness:.2f}, "
            f"ICP minimum fitness {self.config.icp_min_fitness:.2f}, "
            f"RANSAC match distance {self.config.ransac_max_distance_m:.2f} m, "
            f"initial pose confirmation {self.config.initial_pose_confirmation_enabled}, "
            f"interactive approval {self.config.interactive_calibration}")

    def description_callback(self, msg):
        # msg: a ROS String whose data contains the current robot_description URDF.
        with self.state_lock:
            self.robot_description = msg.data

    def camera_callback(self, name, msg):
        # name: camera identifier, such as "front" or "left".
        # msg: that camera's ROS PointCloud2 message.
        with self.data_lock:
            now = time.monotonic()
            self.camera_clouds[name] = (msg, now)
            self.cloud_receipts[name].append(now)
        if self.config.interactive_calibration:
            try:
                self.hub.publish_cloud(name, msg)
            except Exception as exc:
                self.get_logger().warning(f"Hub preview {name} failed: {exc}", throttle_duration_sec=30.0)

    def floor_callback(self, name, msg):
        # Keep each camera's plane separate, including when its cloud arrives later.
        with self.data_lock:
            self.floor_planes[name].append(msg)

    def pcd_callback(self, msg):
        # msg: the latest LiDAR ROS PointCloud2 message.
        with self.data_lock:
            now = time.monotonic()
            self.latest_lidar = (msg, now)
            self.cloud_receipts["lidar"].append(now)

    def imu_callback(self, msg):
        # Keep a short timestamped history so a scan can use its own IMU samples.
        with self.data_lock:
            self.imu_samples.append((msg, time.monotonic()))

    def prepare_calibration_input(self, name, msg, floor_plane=_FLOOR_PLANE_UNSET):
        # name identifies the LiDAR or one ZED; msg is that sensor's ROS cloud.
        # Keep points inside each sensor's box before downsampling or wall filtering.
        started = time.monotonic()
        imu_samples = ()
        if name == "lidar" and self.config.lidar_ground_filter_enabled:
            with self.data_lock:
                imu_samples = tuple(self.imu_samples)
        full_cloud = point_cloud_from_message(msg)
        valid_count = len(full_cloud.points)
        if name == "lidar":
            reference = "base_link"
            lower, upper = self.config.lidar_box_min_m, self.config.lidar_box_max_m
        else:
            reference = f"zed_{name}_camera_link"
            lower, upper = self.config.camera_box_min_m, self.config.camera_box_max_m
        try:
            full_cloud = crop_calibration_cloud(full_cloud, self.tf_buffer, msg.header.frame_id,
                                                reference, lower, upper)
            cropped_count = len(full_cloud.points)
            if name == "lidar" and self.config.lidar_ground_filter_enabled:
                up, imu_count = lidar_up_from_imu(imu_samples, msg.header, self.tf_buffer,
                                                  started, self.config)
                full_cloud, ground_removed, inliers, height, tilt = remove_lidar_ground(
                    full_cloud, msg.header.frame_id, self.tf_buffer, up, self.config)
                if self.config.cloud_diagnostics_enabled:
                    self.get_logger().info(
                        f"LiDAR ground: cropped={cropped_count}, removed={ground_removed}, "
                        f"remaining={len(full_cloud.points)}, plane_inliers={inliers}, "
                        f"height_at_base={height:.3f} m, tilt_from_up={tilt:.1f} deg, "
                        f"imu_samples={imu_count}, tolerance={self.config.lidar_ground_distance_tolerance_m:.3f} m")
            elif name != "lidar" and self.config.ground_filter_enabled:
                if floor_plane is _FLOOR_PLANE_UNSET:
                    with self.data_lock:
                        floor_plane = next((item for item in reversed(self.floor_planes[name])
                                            if stamp_key(item.header.stamp) == stamp_key(msg.header.stamp)), None)
                full_cloud, ground_removed = remove_zed_ground(
                    full_cloud, msg.header, floor_plane, self.config.ground_distance_tolerance_m)
                if self.config.cloud_diagnostics_enabled:
                    self.get_logger().info(
                        f"ZED ground {name}: cropped={cropped_count}, removed={ground_removed}, "
                        f"remaining={len(full_cloud.points)}, tolerance="
                        f"{self.config.ground_distance_tolerance_m:.3f} m")
            cloud = full_cloud.voxel_down_sample(self.config.icp_voxel_size_m)
            voxel_count = len(cloud.points)
            walls, removed = None, 0
            if self.config.wall_filter_enabled:
                upright = "base_link" if name == "lidar" else f"zed_{name}_camera_link"
                up = up_in_cloud_frame(self.tf_buffer, msg.header.frame_id, upright)
                cloud, walls, removed = remove_wall_patches(cloud, up, self.config)
                self.get_logger().debug(
                    f"Wall filter {name}: {walls} patches, {removed} points removed, "
                    f"{len(cloud.points)} remain, {time.monotonic() - started:.2f}s")
        except Exception as exc:
            if self.config.cloud_diagnostics_enabled:
                self.get_logger().info(
                    f"Point cloud {name}: frame={msg.header.frame_id}, raw={msg.width * msg.height}, "
                    f"finite_xyz={valid_count}, processing failed: {exc}")
            raise
        if self.config.cloud_diagnostics_enabled:
            try:
                self.get_logger().info(cloud_diagnostics(
                    name, msg, valid_count, cropped_count, voxel_count, cloud, walls, removed))
            except Exception as exc:
                self.get_logger().info(f"Point cloud {name}: diagnostic summary unavailable ({exc})")
        display = cloud if self.config.wall_filter_enabled else full_cloud
        self.input_pubs[name].publish(pack_registered_cloud([(name, display)], msg.header, self.config))
        return cloud, display

    def publish_registered_inputs(self, header, lidar, cameras, reasons, poses):
        # These display clouds are wall-filtered when enabled, full resolution
        # otherwise. Only previously accepted camera poses are used in the overlay.
        registered = []
        if lidar is not None:
            for name, (msg, cloud) in cameras.items():
                cached = poses.get(name)
                if reasons.get(name) or cached is None or cached[:2] != (header.frame_id, msg.header.frame_id):
                    continue
                camera = o3d.geometry.PointCloud(cloud)
                camera.transform(cached[2])
                registered.append((name, camera))
        clouds = [("lidar", lidar), *registered] if registered else []
        if self.config.visualization_voxel_size_m > 0:
            clouds = [(name, cloud.voxel_down_sample(self.config.visualization_voxel_size_m))
                      for name, cloud in clouds]
        self.registered_pub.publish(pack_registered_cloud(clouds, header, self.config))

    def calibration_tick(self):
        # Process one snapshot at the configured interval. Publish its display
        # inputs first, even if timestamps or the vehicle URDF block calibration.
        interactive = self.config.interactive_calibration
        stopped = set()
        session = None
        if interactive:
            stopped, session = self.hub.calibration_snapshot()
            if stopped is None or len(stopped) == len(CAMERAS):
                return
        now = time.monotonic()
        if now < self.next_calibration:
            return
        self.next_calibration = now + self.config.calibration_interval_s

        with self.data_lock:
            lidar_sample = self.latest_lidar
            cameras = {name: sample for name, sample in self.camera_clouds.items() if name not in stopped}
            # Keep the matching plane with each cloud snapshot while slow cloud
            # processing and incoming callbacks continue independently.
            planes = {}
            if self.config.ground_filter_enabled:
                for name, (msg, _) in cameras.items():
                    planes[name] = next((item for item in reversed(self.floor_planes[name])
                                         if stamp_key(item.header.stamp) == stamp_key(msg.header.stamp)), None)
        with self.state_lock:
            description = self.robot_description
            poses = self.camera_poses.copy()
            mounts = self.camera_mounts.copy()
            retry_global = self.retry_global.copy()
            pending = self.update_in_flight
            pending_future = self.pending_update
            update_started = self.update_started

        lidar_msg = lidar_sample[0] if lidar_sample else None
        lidar = None
        lidar_display = None
        prepared_cameras = {}
        camera_displays = {}
        input_errors = {}
        lidar_input_error = False

        # Each input keeps its original frame and timestamp. The displayed LiDAR
        # can be viewed in base_link even when every camera calibration fails.
        if lidar_sample and now - lidar_sample[1] <= self.config.max_cloud_age_s:
            try:
                lidar, lidar_display = self.prepare_calibration_input("lidar", lidar_msg)
            except Exception as exc:
                self.input_pubs["lidar"].publish(pack_registered_cloud([], lidar_msg.header, self.config))
                lidar_input_error = True
                self.get_logger().warning(f"LiDAR calibration input failed: {exc}",
                                          throttle_duration_sec=30.0)
        elif lidar_msg:
            self.input_pubs["lidar"].publish(pack_registered_cloud([], lidar_msg.header, self.config))

        if interactive and not self.hub.session_active(session):
            return
        # Prepare every fresh camera before RANSAC. This makes all
        # camera inputs available even if one camera's RANSAC takes a long time.
        for name, sample in cameras.items():
            if interactive and not self.hub.session_active(session):
                return
            msg, received = sample
            if now - received > self.config.max_cloud_age_s:
                self.input_pubs[name].publish(pack_registered_cloud([], msg.header, self.config))
                continue
            try:
                if self.config.ground_filter_enabled:
                    cloud, display = self.prepare_calibration_input(name, msg, planes[name])
                else:
                    cloud, display = self.prepare_calibration_input(name, msg)
                prepared_cameras[name] = (msg, cloud)
                camera_displays[name] = (msg, display)
            except Exception as exc:
                self.input_pubs[name].publish(pack_registered_cloud([], msg.header, self.config))
                input_errors[name] = f"calibration input failed: {exc}"

        reasons = {name: pair_error(lidar_msg, cameras.get(name), now, self.config)
                   if lidar is not None else "no usable LiDAR cloud" for name in CAMERAS}
        for name, error in input_errors.items():
            reasons[name] = error
        for name, (_, cloud) in prepared_cameras.items():
            if not reasons[name] and len(cloud.points) < self.config.min_points:
                reasons[name] = f"too few camera points after filtering ({len(cloud.points)})"
        if interactive:
            self.hub.note_reasons(reasons, stopped, session)
        usable = {name: prepared_cameras[name] for name in prepared_cameras if not reasons[name]}
        if lidar_msg:
            try:
                self.publish_registered_inputs(lidar_msg.header, lidar_display, camera_displays, reasons, poses)
            except Exception as exc:
                self.get_logger().error(f"Could not publish registration overlay: {exc}")

        # Wait for the previous URDF update before sending another one; otherwise
        # an older response could replace a newer calibration.
        if pending and not interactive:
            if now - update_started <= max(30.0, 3 * self.config.calibration_interval_s):
                return  # Never send overlapping robot_description updates.

            # Stop waiting if the service does not respond (30 s by default).
            # Recheck under the lock in case the response arrived just now.
            expired = False
            with self.state_lock:
                if self.update_in_flight and self.pending_update is pending_future:
                    self.pending_update = None
                    self.update_in_flight = False
                    expired = True
            if expired:
                # Cancel the local wait and allow a later timer tick to retry.
                if pending_future is not None:
                    pending_future.cancel()
                self.get_logger().warning("Vehicle URDF update timed out; will retry")
            return

        if lidar is None:
            if not lidar_input_error:
                self.get_logger().warning("Waiting for a fresh LiDAR cloud for calibration",
                                          throttle_duration_sec=30.0)
            return
        if len(lidar.points) < self.config.min_points:
            self.get_logger().warning(f"Too few LiDAR points after filtering ({len(lidar.points)})",
                                      throttle_duration_sec=30.0)
            return
        if not usable:
            details = "; ".join(f"{name}: {reasons[name]}" for name in CAMERAS if reasons[name] != "no cloud")
            message = "Waiting for ZED clouds" + (f": {details}" if details else "")
            self.get_logger().warning(message, throttle_duration_sec=30.0)
            return

        # A pose must be written into the live vehicle URDF before it can supply TF.
        # Wait until both the current description and its update service are available.
        if not interactive and (not description or not self.description_client.service_is_ready()):
            self.get_logger().warning("Waiting for vehicle robot_description and parameter service",
                                      throttle_duration_sec=30.0)
            return

        try:
            if interactive:
                self.hub.calibrate(lidar, lidar_msg.header, usable, reasons, session)
            else:
                self.calibrate(lidar, lidar_msg.header, usable, reasons, poses, mounts,
                               retry_global, description)
        except Exception as exc:
            # Report the failure; a later timer tick can try again.
            self.get_logger().error(f"Calibration failed: {exc}")

    def calibrate(self, lidar, header, cameras, reasons, poses, previous_mounts, retry_global, description):
        # lidar: the downsampled Open3D LiDAR cloud used as the fixed reference.
        # header: the LiDAR message header, including its frame ID and timestamp.
        # cameras: names mapped to (original ROS message, filtered Open3D cloud).
        # reasons: camera names mapped to a pairing error, or None when usable.
        # poses: names mapped to (LiDAR frame, camera frame, camera-to-LiDAR matrix).
        # previous_mounts: accepted camera-body-to-base_link matrices by camera name.
        # retry_global: cameras needing a new FPFH/RANSAC guess before ICP.
        # description: the current expanded vehicle URDF as an XML string.
        lidar_features = None
        updated = {}
        mounts = {}
        confirmed_names = set()
        skipped = []

        def global_failure(name, frames, reason):
            # A failed search is not a new estimate. Keep this camera's earlier
            # valid observations so an intermittent failure does not erase 1/3.
            with self.state_lock:
                previous = self.pending_candidates.get(name)
                count = (len(previous["poses"])
                         if previous is not None and previous["frames"] == frames else 0)
            if count:
                reason += f"; keeping {count}/{self.config.initial_pose_confirmations} confirmations"
            return reason

        for name in CAMERAS:
            if name not in cameras:
                if reasons[name] != "no cloud":
                    skipped.append(f"{name}: {reasons[name]}")
                continue
            msg, camera = cameras[name]
            frames = (header.frame_id, msg.header.frame_id)
            cached = poses.get(name)
            try:
                global_search = cached is None or cached[:2] != frames or name in retry_global
                if global_search:
                    # A changed cloud frame changes the meaning of the pose.
                    # Never reuse this camera's observations from another frame.
                    with self.state_lock:
                        previous = self.pending_candidates.get(name)
                        if previous is not None and previous["frames"] != frames:
                            self.pending_candidates.pop(name)
                    if lidar_features is None:
                        started = time.monotonic()
                        self.get_logger().debug("Calibration quality: preparing LiDAR FPFH")
                        lidar_features = prepare_fpfh(lidar, self.config)
                        if self.config.cloud_diagnostics_enabled:
                            reason = ("not computed: too few points after FPFH downsampling"
                                      if lidar_features[0] is None else None)
                            log_fpfh_diagnostics(self.get_logger(), "lidar", lidar_features[1], reason)
                        self.get_logger().debug(
                            f"Calibration quality: LiDAR FPFH took {time.monotonic() - started:.2f}s")
                    # No manual ZED mount enters FPFH/RANSAC: geometry gives the initial pose.
                    metrics = {}
                    started = time.monotonic()
                    self.get_logger().debug(f"Calibration quality {name}: starting FPFH/RANSAC")
                    report_features = None
                    if self.config.cloud_diagnostics_enabled:
                        report_features = lambda features, why: log_fpfh_diagnostics(
                            self.get_logger(), name, features, why)
                    initial_pose, reason = estimate_initial_pose(
                        camera, lidar_features, self.config, metrics, report_features)
                    if metrics:
                        log_quality = (self.get_logger().info if self.config.cloud_diagnostics_enabled
                                       else self.get_logger().debug)
                        log_quality(
                            f"Calibration quality {name}: RANSAC fitness {metrics['fitness']:.3f} "
                            f"(minimum {self.config.ransac_min_fitness:.3f}), "
                            f"{metrics['matches']} matches (minimum {self.config.min_points}), "
                            f"{time.monotonic() - started:.2f}s")
                    if initial_pose is None:
                        skipped.append(f"{name}: {global_failure(name, frames, reason)}")
                        continue
                else:
                    initial_pose = cached[2]
                metrics = {}
                registered, pose, reason = register_camera(camera, lidar, initial_pose, self.config, metrics)
                if metrics:
                    log_quality = (self.get_logger().info if self.config.cloud_diagnostics_enabled
                                   else self.get_logger().debug)
                    log_quality(
                        f"Calibration quality {name}: ICP fitness {metrics['fitness']:.3f} "
                        f"(minimum {self.config.icp_min_fitness:.3f}), "
                        f"RMSE {metrics['rmse']:.3f} m (maximum {self.config.icp_max_rmse_m:.3f} m), "
                        f"{metrics['matches']} matches (minimum {self.config.min_points})")

                # Keep small ICP updates near the accepted global mount.
                # Comparing with the original anchor also prevents many small
                # updates from drifting a long way over time.
                if registered is not None and not global_search:
                    with self.state_lock:
                        anchor = self.camera_anchors.get(name)
                    if anchor is not None and anchor[:2] == frames:
                        translation, angle = pose_difference(pose, anchor[2])
                        if (translation > self.config.pose_agreement_translation_m
                                or angle > self.config.pose_agreement_rotation_deg):
                            registered = None
                            reason = f"ICP drift from confirmed pose too large ({translation:.2f} m, {angle:.1f} deg)"
                if registered is None:
                    if global_search:
                        reason = global_failure(name, frames, reason)
                    if cached is not None and not global_search:
                        with self.state_lock:
                            failures = self.camera_failures.get(name, 0) + 1
                            self.camera_failures[name] = failures
                            if failures >= self.config.max_icp_failures:
                                self.retry_global.add(name)
                                reason += "; next attempt will retry FPFH/RANSAC"
                    skipped.append(f"{name}: {reason}")
                    continue

                if global_search and self.config.initial_pose_confirmation_enabled:
                    # RANSAC is random: require fresh, independently computed
                    # estimates to agree before publishing the first pose.
                    with self.state_lock:
                        confirmed, reason = confirm_initial_pose(
                            self.pending_candidates, name, frames, pose,
                            header.stamp, msg.header.stamp, self.config)
                    if not confirmed:
                        skipped.append(f"{name}: {reason}")
                        continue
                with self.state_lock:
                    self.camera_failures.pop(name, None)  # ICP succeeded, even if a TF is still missing.

                # ICP maps cloud -> LiDAR. URDF needs body -> base, including
                # the driver's internal optical/body and LiDAR frame transforms.
                mount = camera_mount_matrix(self.tf_buffer, *frames, f"zed_{name}_camera_link", pose)
                updated[name] = (*frames, pose)
                mounts[name] = mount
                if global_search:
                    confirmed_names.add(name)
            except (TransformException, ValueError) as exc:
                skipped.append(f"{name}: {exc}")
            except Exception as exc:
                skipped.append(f"{name}: registration error: {exc}")

        if skipped:
            self.get_logger().warning("Skipped ZED clouds: " + "; ".join(skipped),
                                      throttle_duration_sec=30.0)
        if not updated:
            return
        try:
            # Include earlier accepted mounts even if the description topic was
            # delayed or the vehicle publisher restarted with its base URDF.
            new_description = update_camera_joints(description, {**previous_mounts, **mounts})
        except (ET.ParseError, ValueError) as exc:
            self.get_logger().error(f"Could not update camera joints in robot_description: {exc}")
            return
        request = SetParameters.Request()
        request.parameters = [Parameter("robot_description", Parameter.Type.STRING,
                                        new_description).to_parameter_msg()]
        with self.state_lock:
            self.update_in_flight = True
            self.update_started = time.monotonic()
        try:
            future = self.description_client.call_async(request)
            with self.state_lock:
                self.pending_update = future
            future.add_done_callback(
                lambda result: self.description_update_done(result, new_description, updated, mounts, confirmed_names))
        except Exception as exc:
            with self.state_lock:
                self.update_in_flight = False
            self.get_logger().error(f"Could not request URDF update: {exc}")

    def description_update_done(self, future, description, updated, mounts, confirmed_names):
        # future: the pending robot_state_publisher parameter-update response.
        # description: the proposed updated URDF as an XML string.
        # updated: names mapped to (LiDAR frame, camera frame, camera-to-LiDAR matrix).
        # mounts: camera names mapped to the proposed camera-body-to-base_link matrices.
        # confirmed_names: cameras whose new global pose becomes the drift anchor.
        with self.state_lock:
            if future is not self.pending_update:
                return  # A timed-out request must not replace a newer estimate.
        try:
            result = future.result()
            if len(result.results) != 1 or not result.results[0].successful:
                reason = result.results[0].reason if result.results else "no parameter result"
                raise RuntimeError(reason)
            # Use a pose for visualization and the next ICP only after the
            # vehicle publisher accepts that same pose in its URDF.
            calibrated = []
            with self.state_lock:
                if future is not self.pending_update:
                    return
                self.robot_description = description
                self.camera_poses.update(updated)
                self.camera_mounts.update(mounts)
                for name in updated:
                    if name in confirmed_names or name not in self.camera_anchors:
                        lidar_frame, camera_frame, pose = updated[name]
                        self.camera_anchors[name] = (lidar_frame, camera_frame, pose.copy())
                        calibrated.append(name)
                    self.camera_failures.pop(name, None)
                    self.retry_global.discard(name)
                    self.pending_candidates.pop(name, None)
            try:
                save_calibrated_urdf(description, self.output_urdf)
            except OSError as exc:
                self.get_logger().error(f"Live URDF updated but could not save {self.output_urdf}: {exc}")
            if calibrated:
                self.get_logger().info("Calibrated ZED cameras: " + ", ".join(calibrated))
            for name, (lidar_frame, camera_frame, pose) in updated.items():
                self.get_logger().debug(f"{name}: {camera_frame} -> {lidar_frame} camera-to-LiDAR pose:\n"
                                        f"{np.array2string(pose, precision=4)}")
        except Exception as exc:
            self.get_logger().error(f"Vehicle robot_state_publisher rejected calibration: {exc}")
        finally:
            with self.state_lock:
                if self.pending_update is future:
                    self.pending_update = None
                    self.update_in_flight = False


def pair_error(lidar_msg, sample, now, config):
    # lidar_msg: the LiDAR PointCloud2 message to pair with a camera cloud.
    # sample: (camera PointCloud2 message, monotonic receipt time), or None if missing.
    # now: current monotonic time in seconds, used to check the sample's age.
    # config: limits for cloud age and LiDAR-to-camera timestamp difference.
    # Explain why a camera sample cannot be paired with a LiDAR scan.
    if sample is None:
        return "no cloud"
    msg, received = sample
    if now - received > config.max_cloud_age_s:
        return "camera cloud is stale"
    lidar_time = stamp_seconds(lidar_msg.header.stamp)
    camera_time = stamp_seconds(msg.header.stamp)
    if not lidar_time or not camera_time:
        return "missing timestamp"
    if not lidar_msg.header.frame_id or not msg.header.frame_id:
        return "missing frame ID"
    difference = abs(lidar_time - camera_time)
    if difference > config.max_camera_time_difference_s:
        return f"timestamp difference {difference:.2f}s"
    return None


def main(args=None):
    # args: optional ROS command-line arguments.
    rclpy.init(args=args)  # Initialize ROS before creating the node.
    node = None
    executor = None
    try:
        node = LidarCameraRegistration()
        # Separate threads let camera callbacks run during LiDAR processing.
        executor = MultiThreadedExecutor(num_threads=4)
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
