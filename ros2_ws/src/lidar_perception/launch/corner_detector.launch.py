"""Run the headless edge detector on the Jetson."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_perception',
            executable='corner_detector.py',
            name='corner_detector',
            output='screen',
        ),
    ])
