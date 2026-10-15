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
             parameters=[config], output='screen',
             # The Jetson has a pip-installed NumPy 2 in ~/.local, but ROS Humble and
             # the system OpenCV are built for the system NumPy 1.x. Ignoring the
             # user's site-packages for this node only gives it the matching pair
             # (and OpenCV's ~4x faster warping) without touching anything else.
             additional_env={'PYTHONNOUSERSITE': '1'}),
    ])
