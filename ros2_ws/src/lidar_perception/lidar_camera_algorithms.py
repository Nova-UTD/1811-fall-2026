# Estimate camera-cloud-to-LiDAR poses from prepared Open3D clouds.
#
# FPFH and RANSAC estimate the initial 4 x 4 transform. ICP refines it against
# the LiDAR reference. These functions are usable without ROS.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

import numpy as np
import open3d as o3d


# FPFH: Fast Point Feature Histograms - describe local shapes without needing to know the sensor's mounting pose.
# These features help RANSAC estimate each camera's initial position and orientation relative to LiDAR.
def prepare_fpfh(sensor_cloud, config):
    # sensor_cloud is an Open3D point cloud in its original frame.
    # Downsample it for feature matching using the voxel size in config.
    feature_cloud = sensor_cloud.voxel_down_sample(config.fpfh_voxel_size_m)
    if len(feature_cloud.points) < config.min_points:
        return None, None  # Too little geometry remains to find a reliable match.

    # FPFH compares local surface shapes, so first estimate a normal at every point.
    # Example: points on level ground have normals roughly perpendicular to the ground.
    feature_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=config.fpfh_normal_radius_m, max_nn=30,
    ))
    # Face normals toward their own device's origins.
    feature_cloud.orient_normals_towards_camera_location(np.zeros(3))
    # Encode the relationships between nearby normals as 33-value FPFH descriptors.
    # Example: a corner seen by both sensors may produce similar descriptors in both clouds.
    # max_nn: max number of nearest neighbors to consider for each point.
    fpfh_descriptors = o3d.pipelines.registration.compute_fpfh_feature(
        feature_cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=config.fpfh_feature_radius_m, max_nn=100),
    )
    return feature_cloud, fpfh_descriptors


# Both RANSAC and ICP require enough correspondences and a finite fitness.
def overlap_rejection(registration_result, stage, min_fitness, min_points):
    match_count = len(registration_result.correspondence_set)
    if (not np.isfinite(registration_result.fitness)
            or match_count < min_points or registration_result.fitness < min_fitness):
        return (f"insufficient {stage} overlap (fitness {registration_result.fitness:.2f}, "
                f"minimum {min_fitness:.2f}; matches {match_count}, minimum {min_points})")
    return None


# Use FPFH matches and RANSAC (Random Sample Consensus) to find an initial pose.
def estimate_initial_pose(camera_cloud, lidar_feature_pair, config, metrics=None, report_features=None):
    # lidar_feature_pair holds the downsampled LiDAR cloud and its FPFH descriptors.
    # If provided, report_features runs before RANSAC and metrics receives the
    # score and match count. Return the camera-to-LiDAR matrix or a rejection reason.
    lidar_feature_cloud, lidar_descriptors = lidar_feature_pair
    # Skip camera feature computation when the LiDAR cannot be matched.
    if lidar_feature_cloud is None:
        if report_features is not None:
            report_features(None, "not computed: too few LiDAR points after FPFH downsampling")
        return None, "too few points for FPFH"
    camera_feature_cloud, camera_descriptors = prepare_fpfh(camera_cloud, config)
    if report_features is not None:
        rejection_reason = (
            "not computed: too few points after FPFH downsampling" if camera_feature_cloud is None else None
        )
        report_features(camera_descriptors, rejection_reason)
    if camera_feature_cloud is None:
        return None, "too few points for FPFH"

    registration = o3d.pipelines.registration
    # RANSAC proposes a pose from triples of matching camera and LiDAR points.
    ransac_result = registration.registration_ransac_based_on_feature_matching(
        camera_feature_cloud, lidar_feature_cloud, camera_descriptors, lidar_descriptors,
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
        metrics.update(fitness=float(ransac_result.fitness), matches=len(ransac_result.correspondence_set))
    # RANSAC's 4x4 result maps points from the camera cloud frame to LiDAR.
    #
    #                     [ r00  r01  r02  tx ]
    #   camera_to_lidar = [ r10  r11  r12  ty ]
    #                     [ r20  r21  r22  tz ]
    #                     [  0    0    0    1 ]
    #
    # The top-left 3x3 rotates coordinates into the LiDAR frame. The last
    # column adds the camera origin in LiDAR coordinates, in meters.
    # For example, identity rotation and translation [1, 0, 0] map the
    # camera point [2, 3, 4, 1] to the LiDAR point [3, 3, 4, 1].
    camera_to_lidar = np.asarray(ransac_result.transformation)
    if camera_to_lidar.shape != (4, 4) or not np.isfinite(camera_to_lidar).all():
        return None, "invalid RANSAC transform"
    # Reject a candidate with too little overlap before asking ICP to refine it.
    rejection_reason = overlap_rejection(ransac_result, "RANSAC", config.ransac_min_fitness, config.min_points)
    if rejection_reason:
        return None, rejection_reason
    return camera_to_lidar.copy(), None  # Maps camera message frame to LiDAR message frame.


# Compare two estimates of the same camera-to-LiDAR transform.
def pose_difference(camera_to_lidar, reference_camera_to_lidar):
    # Both 4x4 matrices must describe the same pair of cloud frames.
    # The reference can be an earlier estimate, the ICP guess, or a confirmed pose.
    # Compare the XYZ translations in the last column. For example, positions
    # [1.00, 0, 0] and [1.03, 0, 0] are 0.03 m apart.
    translation_difference_m = np.linalg.norm(camera_to_lidar[:3, 3] - reference_camera_to_lidar[:3, 3])
    # Compare the 3x3 rotations to find the smallest angle between orientations.
    relative_rotation = camera_to_lidar[:3, :3] @ reference_camera_to_lidar[:3, :3].T
    # Clamp the cosine to [-1, 1] so rounding cannot make arccos invalid.
    rotation_difference_deg = np.degrees(np.arccos(np.clip((np.trace(relative_rotation) - 1) / 2, -1, 1)))
    return float(translation_difference_m), float(rotation_difference_deg)


# Decide whether a camera's initial pose has enough independent, agreeing estimates.
def confirm_initial_pose(
    pending_by_camera, camera_name, cloud_frame_ids, camera_to_lidar,
    lidar_stamp, camera_stamp, config,
):
    # Update pending_by_camera in place. cloud_frame_ids lists the LiDAR frame
    # and then the camera frame. Stamps identify an independent pair of scans.
    # Return (confirmed, reason) for the caller to decide whether to apply the pose.
    scan_stamp_pair = ((lidar_stamp.sec, lidar_stamp.nanosec),
                       (camera_stamp.sec, camera_stamp.nanosec))
    previous_candidate = pending_by_camera.get(camera_name)
    saved_poses, saved_scan_pairs = [], []
    reset_reason = None
    # Only reuse this camera's saved estimates when both cloud frames still match.
    if previous_candidate is not None and previous_candidate["frames"] == cloud_frame_ids:
        # Each pose corresponds to a pair of scan timestamps at the same list index.
        saved_poses = previous_candidate["poses"]
        saved_scan_pairs = previous_candidate["observations"]
        # Reusing either scan is not an independent confirmation.
        if any(
            scan_stamp_pair[0] == earlier_scan_pair[0] or scan_stamp_pair[1] == earlier_scan_pair[1]
            for earlier_scan_pair in saved_scan_pairs
        ):
            # A confirmed pose may need another URDF/TF update attempt. It can
            # be retried without counting this same scan a second time, provided
            # the newly computed pose still agrees with all prior estimates.
            if len(saved_poses) >= config.initial_pose_confirmations and all(
                    translation_difference_m <= config.pose_agreement_translation_m
                    and rotation_difference_deg <= config.pose_agreement_rotation_deg
                    for translation_difference_m, rotation_difference_deg in (
                        pose_difference(camera_to_lidar, earlier_pose) for earlier_pose in saved_poses
                    )):
                return True, f"{len(saved_poses)} independent estimates agree"
            return False, f"waiting for new LiDAR and camera scans ({len(saved_poses)}/{config.initial_pose_confirmations})"
        # The new pose must agree with every saved pose, not just the latest one.
        for earlier_pose in saved_poses:
            translation_difference_m, rotation_difference_deg = pose_difference(camera_to_lidar, earlier_pose)
            if (translation_difference_m > config.pose_agreement_translation_m
                    or rotation_difference_deg > config.pose_agreement_rotation_deg):
                saved_poses, saved_scan_pairs = [], []  # Start again from this different estimate.
                reset_reason = (f"estimate changed {translation_difference_m:.2f} m, {rotation_difference_deg:.1f} deg; "
                                "restarting confirmations")
                break
    # Save a copy of the new pose and its scan pair, keeping at most the required count.
    saved_poses = [*saved_poses, camera_to_lidar.copy()][-config.initial_pose_confirmations:]
    saved_scan_pairs = [*saved_scan_pairs, scan_stamp_pair][-config.initial_pose_confirmations:]
    # Keep the confirmation until robot_state_publisher accepts the URDF.
    # Otherwise a missing TF or rejected update would make 3/3 restart at 1/3.
    pending_by_camera[camera_name] = {
        "frames": cloud_frame_ids, "poses": saved_poses, "observations": saved_scan_pairs,
    }
    if len(saved_poses) >= config.initial_pose_confirmations:
        return True, f"{len(saved_poses)} independent estimates agree"
    reason = reset_reason or "waiting for consistent estimates"
    return False, f"{reason} ({len(saved_poses)}/{config.initial_pose_confirmations})"


# ICP: Iterative Closest Point - refine the initial pose provided by RANSAC with ICP registration.
def register_camera(camera_cloud, lidar_cloud, initial_camera_to_lidar, config, metrics=None):
    # initial_camera_to_lidar is the 4x4 RANSAC guess or a previously accepted pose.
    # config supplies the ICP acceptance limits. The optional metrics dictionary
    # receives fitness, match count, and distance error for logging.
    if len(camera_cloud.points) < config.min_points or len(lidar_cloud.points) < config.min_points:
        return None, "too few points"

    # Estimate the camera-to-LiDAR pose without changing either input cloud.
    icp_result = o3d.pipelines.registration.registration_icp(
        camera_cloud,  # Source: ICP estimates how this camera should move.
        lidar_cloud,  # Target: the LiDAR remains fixed in its own frame.
        config.icp_max_distance_m,  # Ignore potential matches farther apart than this.
        initial_camera_to_lidar,  # Start from the RANSAC estimate or the cached accepted pose.
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),  # Match XYZ points.
        # Stop after the iteration limit if ICP has not converged sooner.
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=config.icp_max_iterations),
    )
    # ICP returns a complete camera-to-LiDAR matrix. If the guess was x = 1.00 m
    # and ICP adds 0.05 m, the returned translation is x = 1.05 m.
    camera_to_lidar = np.asarray(icp_result.transformation)
    if camera_to_lidar.shape != (4, 4) or not np.isfinite(camera_to_lidar).all():
        return None, "invalid ICP transform"  # Never publish a non-finite pose.
    if metrics is not None:
        metrics.update(fitness=float(icp_result.fitness), rmse=float(icp_result.inlier_rmse),
                       matches=len(icp_result.correspondence_set))
    # Fitness measures how much of the source point cloud successfully matches the target point cloud.
    # It is the fraction of camera points with a LiDAR match within the limit.
    # For example, 40 matched camera points out of 200 give fitness 0.20; both counts and fitness must pass.
    rejection_reason = overlap_rejection(icp_result, "ICP", config.icp_min_fitness, config.min_points)
    if rejection_reason:
        return None, rejection_reason
    # RMSE summarizes matched distances, giving larger errors more weight.
    # Example: 0.12 m exceeds a 0.08 m limit, even if enough points matched.
    if not np.isfinite(icp_result.inlier_rmse) or icp_result.inlier_rmse > config.icp_max_rmse_m:
        return None, f"ICP match error too high (RMSE {icp_result.inlier_rmse:.3f} m)"
    # The translation is the camera cloud origin in the LiDAR frame. A camera
    # on this vehicle should not be several metres away from its LiDAR.
    # This is a broad physical limit, not an initial camera pose.
    camera_distance_m = np.linalg.norm(camera_to_lidar[:3, 3])
    if camera_distance_m > config.max_camera_lidar_distance_m:
        return None, (f"camera-to-LiDAR distance too large ({camera_distance_m:.2f} m, "
                      f"maximum {config.max_camera_lidar_distance_m:.2f} m)")

    # Check how far the final camera pose moved from the starting estimate.
    correction_distance_m, correction_angle_deg = pose_difference(camera_to_lidar, initial_camera_to_lidar)
    # Reject large corrections that are more likely to be a bad match than calibration.
    # For example, moving x from 1.0m to 1.6m exceeds the default 0.5m limit.
    if correction_distance_m > config.icp_max_translation_m or correction_angle_deg > config.icp_max_rotation_deg:
        return None, f"ICP correction too large ({correction_distance_m:.2f} m, {correction_angle_deg:.1f} deg)"
    return camera_to_lidar.copy(), None  # The input cloud stays in its camera frame.
