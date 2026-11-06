"""Run LiDAR and ZED point-cloud registration on the Jetson."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lidar_perception'), 'config', 'registration.yaml',
            ]),
            description='ROS parameter file for LiDAR and camera registration.',
        ),
        DeclareLaunchArgument('interactive_calibration', default_value='false',
                              description='Wait for UI Hub start and per-camera approval.'),
        Node(
            package='lidar_perception',
            executable='lidar_camera_registration.py',
            name='lidar_camera_registration',
            output='screen',
            parameters=[config_file, {'interactive_calibration': ParameterValue(
                LaunchConfiguration('interactive_calibration'), value_type=bool)}],
        ),
    ])
