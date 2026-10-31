"""Live lidar odometry via KISS-ICP, wrapped for the 1811 stack.

Runs PRBonn kiss-icp against the Ouster and republishes its output as the
stack's canonical /odometry, with the odom->base_link TF. Uses base_link so the
pose is the VEHICLE's, not the sensor's — that mapping comes from the URDF
(vehicle_1811_description: base_link -> os_sensor -> os_lidar).

    ros2 launch localization localization.launch.py                     # live
    ros2 launch localization localization.launch.py use_sim_time:=true  # bag replay (with --clock)

Locked to the KISS-ICP version vendored in ros2_ws/src/kiss-icp:
  - odom frame arg is `lidar_odom_frame` (not `odom_frame`)
  - `use_sim_time` MUST be false for a live sensor (default true = waits for /clock)
  - kiss_icp publishes on /kiss/odometry -> remapped to /odometry here
If you bump the kiss-icp submodule and it stops working, re-check the arg names
in ros2_ws/src/kiss-icp/ros/launch/odometry.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")

    kiss_icp_launch = PathJoinSubstitution(
        [FindPackageShare("kiss_icp"), "launch", "odometry.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/ouster/points",
            description="Input lidar point cloud topic (from ouster-ros).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="false for a live sensor; true for bag replay (ros2 bag play --clock).",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("localization"), "config", "kiss_icp.yaml"]
            ),
            description="KISS-ICP params. Defaults to the 1811 tuning in this package.",
        ),

        # kiss_icp publishes odometry on /kiss/odometry; make it the stack's /odometry.
        SetRemap(src="/kiss/odometry", dst="/odometry"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([kiss_icp_launch]),
            launch_arguments={
                "topic": pointcloud_topic,          # remapped to the node's pointcloud_topic
                "base_frame": "base_link",          # report the VEHICLE pose (URDF)
                "lidar_odom_frame": "odom",         # fixed odometry frame
                "publish_odom_tf": "true",          # kiss_icp owns odom->base_link
                "visualize": "false",               # headless on the Karbon; RViz on the Jetson
                "use_sim_time": use_sim_time,
                "config_file": config_file,
            }.items(),
        ),
    ])
