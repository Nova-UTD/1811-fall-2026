"""Corner and plane detector on /ouster/points.

    ros2 launch lidar_perception corner_detector.launch.py
    ros2 launch lidar_perception corner_detector.launch.py headless:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true: run heavy RANSAC headless (e.g. on Jetson) and publish /lidar_perception/segmented_points',
        ),
        DeclareLaunchArgument(
            'refresh_interval',
            default_value='2.0',
            description='Seconds between scan updates.',
        ),
        Node(
            package='lidar_perception',
            executable='corner_detector.py',
            name='corner_detector',
            output='screen',
            parameters=[{
                'headless': LaunchConfiguration('headless'),
                'refresh_interval': LaunchConfiguration('refresh_interval'),
            }],
        ),
    ])
