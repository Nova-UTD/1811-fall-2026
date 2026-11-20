# Diagnostic summaries for LiDAR-camera calibration.
#
# The registration and interactive nodes decide when to emit these messages.
# This module formats the data already computed by the calibration pipeline.

import numpy as np


# Describe the points used for matching, in the message's original frame.
def cloud_diagnostics(name, msg, valid_count, cropped_count, voxel_count, cloud, walls, removed):
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


# Log the FPFH descriptors already computed for a global search.
def log_fpfh_diagnostics(logger, name, features, reason=None):
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


def lidar_ground_diagnostics(cropped, removed, remaining, inliers, height, tilt, imu_count, tolerance):
    # Height and tolerance are meters; tilt is degrees from the IMU-derived up direction.
    return (f"LiDAR ground: cropped={cropped}, removed={removed}, "
            f"remaining={remaining}, plane_inliers={inliers}, "
            f"height_at_base={height:.3f} m, tilt_from_up={tilt:.1f} deg, "
            f"imu_samples={imu_count}, tolerance={tolerance:.3f} m")


def zed_ground_diagnostics(name, cropped, removed, remaining, tolerance):
    return (f"ZED ground {name}: cropped={cropped}, removed={removed}, "
            f"remaining={remaining}, tolerance={tolerance:.3f} m")


def wall_filter_diagnostics(name, walls, removed, remaining, elapsed):
    return (f"Wall filter {name}: {walls} patches, {removed} points removed, "
            f"{remaining} remain, {elapsed:.2f}s")


def cloud_processing_failure_diagnostics(name, msg, valid_count, exc):
    return (f"Point cloud {name}: frame={msg.header.frame_id}, raw={msg.width * msg.height}, "
            f"finite_xyz={valid_count}, processing failed: {exc}")


def cloud_summary_unavailable_diagnostics(name, exc):
    return f"Point cloud {name}: diagnostic summary unavailable ({exc})"


def log_fpfh_preparation_started(logger, name):
    if name == "lidar":
        logger.debug("Calibration quality: preparing LiDAR FPFH")
    else:
        logger.debug(f"Calibration quality {name}: starting FPFH/RANSAC")


def log_lidar_fpfh_preparation_time(logger, elapsed):
    logger.debug(f"Calibration quality: LiDAR FPFH took {elapsed:.2f}s")


def ransac_quality_diagnostics(name, metrics, config, elapsed):
    return (f"Calibration quality {name}: RANSAC fitness {metrics['fitness']:.3f} "
            f"(minimum {config.ransac_min_fitness:.3f}), "
            f"{metrics['matches']} matches (minimum {config.min_points}), "
            f"{elapsed:.2f}s")


def icp_quality_diagnostics(name, metrics, config=None):
    # Interactive review reports the scores without the automatic-mode limits.
    if config is None:
        return (f"Calibration quality {name}: ICP fitness {metrics['fitness']:.3f}, "
                f"RMSE {metrics['rmse']:.3f} m, {metrics['matches']} matches")
    return (f"Calibration quality {name}: ICP fitness {metrics['fitness']:.3f} "
            f"(minimum {config.icp_min_fitness:.3f}), "
            f"RMSE {metrics['rmse']:.3f} m (maximum {config.icp_max_rmse_m:.3f} m), "
            f"{metrics['matches']} matches (minimum {config.min_points})")
