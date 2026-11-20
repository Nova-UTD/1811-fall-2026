# Calibration settings and checks performed at startup.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

import socket
from typing import NamedTuple

CAMERAS = ("front", "left", "rear", "right")  # Names used in the four ZED topic paths.


def is_jetson_link():
    # Check the configured Jetson cable address before loading registration libraries.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            # UDP connect selects a route and local address; it sends no data.
            sock.connect(("192.168.100.1", 9))
            return sock.getsockname()[0] == "192.168.100.2"
    except OSError:
        return False


# Startup defaults: the launch file loads user settings from registration.yaml.
# See config/registration.yaml for an example of each setting.
class RegistrationConfig(NamedTuple):
    calibration_interval_s: float = 10.0
    max_cloud_age_s: float = 2.0
    cloud_diagnostics_enabled: bool = True
    interactive_calibration: bool = False
    restore_calibration_on_startup: bool = True
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
    import numpy as np
    from rcl_interfaces.msg import ParameterDescriptor

    parameter_values = {}
    for parameter_name in DEFAULT_CONFIG._fields:
        default_value = getattr(DEFAULT_CONFIG, parameter_name)
        parameter_default = list(default_value) if isinstance(default_value, tuple) else default_value
        parameter_value = node.declare_parameter(
            parameter_name, parameter_default, ParameterDescriptor(read_only=True),
        ).value
        parameter_values[parameter_name] = (
            tuple(parameter_value) if isinstance(default_value, tuple) else parameter_value
        )
    config = RegistrationConfig(**parameter_values)

    if not isinstance(config.initial_pose_confirmation_enabled, bool):
        raise ValueError("initial_pose_confirmation_enabled must be true or false")
    if not isinstance(config.cloud_diagnostics_enabled, bool):
        raise ValueError("cloud_diagnostics_enabled must be true or false")
    if not isinstance(config.interactive_calibration, bool):
        raise ValueError("interactive_calibration must be true or false")
    if not isinstance(config.restore_calibration_on_startup, bool):
        raise ValueError("restore_calibration_on_startup must be true or false")
    if not isinstance(config.ground_filter_enabled, bool):
        raise ValueError("ground_filter_enabled must be true or false")
    if not isinstance(config.lidar_ground_filter_enabled, bool):
        raise ValueError("lidar_ground_filter_enabled must be true or false")
    for sensor in ("lidar", "camera"):
        lower_name, upper_name = f"{sensor}_box_min_m", f"{sensor}_box_max_m"
        lower_bounds, upper_bounds = getattr(config, lower_name), getattr(config, upper_name)
        if (len(lower_bounds) != 3 or len(upper_bounds) != 3 or not np.isfinite(lower_bounds).all()
                or not np.isfinite(upper_bounds).all() or np.any(np.asarray(lower_bounds) >= np.asarray(upper_bounds))):
            raise ValueError(f"{lower_name} and {upper_name} must be finite XYZ bounds with min < max")
        if sensor == "camera" and lower_bounds[0] < 0:
            raise ValueError("camera_box_min_m forward bound must be at least zero")
    positive_parameter_names = (
        "icp_voxel_size_m", "fpfh_voxel_size_m", "fpfh_normal_radius_multiplier", "icp_max_rmse_m",
        "max_camera_lidar_distance_m",
        "fpfh_feature_radius_multiplier", "ransac_max_distance_multiplier", "icp_max_distance_m",
        "calibration_interval_s", "max_cloud_age_s", "wall_min_height_m", "wall_distance_tolerance_m",
        "ground_distance_tolerance_m", "lidar_ground_distance_tolerance_m",
        "lidar_ground_search_height_m", "lidar_imu_max_time_difference_s",
    )
    for parameter_name in positive_parameter_names:
        if not np.isfinite(getattr(config, parameter_name)) or getattr(config, parameter_name) <= 0:
            raise ValueError(f"{parameter_name} must be finite and greater than zero")
    for parameter_name in ("ransac_max_iterations", "icp_max_iterations", "max_icp_failures", "initial_pose_confirmations"):
        if getattr(config, parameter_name) < 1:
            raise ValueError(f"{parameter_name} must be at least 1")
    if config.min_points < 3:
        raise ValueError("min_points must be at least 3")
    if (not isinstance(config.lidar_ground_min_inliers, int)
            or isinstance(config.lidar_ground_min_inliers, bool) or config.lidar_ground_min_inliers < 3):
        raise ValueError("lidar_ground_min_inliers must be an integer of at least 3")
    for parameter_name in ("ransac_min_fitness", "icp_min_fitness"):
        if not np.isfinite(getattr(config, parameter_name)) or not 0.0 <= getattr(config, parameter_name) <= 1.0:
            raise ValueError(f"{parameter_name} must be between 0.0 and 1.0")
    if not np.isfinite(config.ransac_confidence) or not 0.0 < config.ransac_confidence <= 1.0:
        raise ValueError("ransac_confidence must be greater than 0.0 and at most 1.0")
    for parameter_name in (
        "icp_max_translation_m", "icp_max_rotation_deg", "max_camera_time_difference_s",
        "pose_agreement_translation_m", "pose_agreement_rotation_deg", "wall_max_tilt_deg",
        "wall_edge_margin_m", "visualization_voxel_size_m", "lidar_ground_max_tilt_deg",
    ):
        if not np.isfinite(getattr(config, parameter_name)) or getattr(config, parameter_name) < 0:
            raise ValueError(f"{parameter_name} must be finite and at least zero")
    if config.icp_max_rotation_deg > 180.0:
        raise ValueError("icp_max_rotation_deg must be at most 180.0")
    if config.pose_agreement_rotation_deg > 180.0:
        raise ValueError("pose_agreement_rotation_deg must be at most 180.0")
    if config.wall_max_tilt_deg >= 90.0:
        raise ValueError("wall_max_tilt_deg must be less than 90.0")
    if config.lidar_ground_max_tilt_deg >= 90.0:
        raise ValueError("lidar_ground_max_tilt_deg must be less than 90.0")
    for parameter_name in ("lidar_rgb", "front_rgb", "left_rgb", "rear_rgb", "right_rgb"):
        color = getattr(config, parameter_name)
        if len(color) != 3 or any(not isinstance(channel, int) or not 0 <= channel <= 255
                                  for channel in color):
            raise ValueError(f"{parameter_name} must contain three integers between 0 and 255")
    return config
