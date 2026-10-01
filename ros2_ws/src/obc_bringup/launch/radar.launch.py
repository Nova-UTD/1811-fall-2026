"""1811 radar bring-up — Smartmicro DRVEGRD 169 via umrr_ros2_driver.

Uses the team-owned params in this package (not the vendored template).

Host (once per boot, outside the container):
    sudo ip addr add 192.168.11.17/24 dev enp1s0
    sudo ip link set enp1s0 up

Then inside the container:
    ros2 launch obc_bringup radar.launch.py

Optional overrides:
    iface:=enp1s0   (must still match the yaml / NIC you configured)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('obc_bringup'),
        'config',
        'radar_1811.yaml',
    )
    return LaunchDescription([
        Node(
            package='umrr_ros2_driver',
            executable='smartmicro_radar_node_exe',
            name='smart_radar',
            parameters=[params],
            output='screen',
        ),
    ])
