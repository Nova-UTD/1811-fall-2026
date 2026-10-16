"""1811 radar bring-up — Smartmicro DRVEGRD 169 via umrr_ros2_driver.

Uses the team-owned params in this package (not the vendored template).

Host (once per boot, outside the container):
    sudo ip addr add 192.168.11.17/24 dev enp1s0
    sudo ip link set enp1s0 up

Then inside the container:
    ros2 launch obc_bringup radar.launch.py

Optional overrides:
    iface:=<nic>          NIC the radar is on, instead of the yaml's
                          hw_iface_name (e.g. a laptop on the bench). The NIC
                          still needs 192.168.11.17/24, as above.
    params_file:=<path>   a whole different params yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def radar_node(context):
    parameters = [LaunchConfiguration('params_file').perform(context)]
    # Only override when given: the yaml stays the single source of truth.
    iface = LaunchConfiguration('iface').perform(context)
    if iface:
        parameters.append({'adapters.adapter_0.hw_iface_name': iface})
    return [Node(
        package='umrr_ros2_driver',
        executable='smartmicro_radar_node_exe',
        name='smart_radar',
        parameters=parameters,
        output='screen',
    )]


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('obc_bringup'),
        'config',
        'radar_1811.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params,
                              description='Radar params yaml.'),
        DeclareLaunchArgument('iface', default_value='',
                              description='Override the NIC the radar is on '
                                          '(default: hw_iface_name in the yaml).'),
        OpaqueFunction(function=radar_node),
    ])
