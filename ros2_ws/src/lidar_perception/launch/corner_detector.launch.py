"""Run the headless lidar and panorama edge detectors on the Jetson."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_edges', default_value='true',
                              description='Publish panorama images with detected edges.'),
        Node(
            package='lidar_perception',
            executable='corner_detector.py',
            name='corner_detector',
            output='screen',
        ),
        Node(
            package='lidar_perception',
            executable='corner_detector.py',
            arguments=['--camera-edges'],
            name='camera_edge_detector',
            output='screen',
            condition=IfCondition(LaunchConfiguration('camera_edges')),
            # Match the panorama node's system NumPy/OpenCV environment.
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
    ])
