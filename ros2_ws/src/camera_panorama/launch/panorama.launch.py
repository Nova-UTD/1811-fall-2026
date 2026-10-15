"""
360-degree panorama from the four ZED X cameras -- run on the Jetson.

    source ~/1811-fall-2026/ros2_ws/install/local_setup.bash
    ros2 launch camera_panorama panorama.launch.py

Needs the cameras running (jetson_bringup zed_cameras.launch.py) and bringup on
the Karbon (it publishes the URDF that places each camera on the car).
Output: /panorama/image. Settings: config/panorama.yaml.
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('camera_panorama'), 'config', 'panorama.yaml'])
    return LaunchDescription([
        Node(package='camera_panorama', executable='panorama_node', name='panorama_node',
             parameters=[config], output='screen'),
    ])
