"""Launch pure_pursuit_node, optionally against the bicycle_sim_node instead
of real odometry/hardware.

    ros2 launch control pure_pursuit.launch.py
    ros2 launch control pure_pursuit.launch.py use_sim:=true
    ros2 launch control pure_pursuit.launch.py path_file:=/path/to/route.csv

By default path_file points at the bundled sample loop so this runs
standalone with no other nodes. Once route_publisher exists, launch with
path_file:='' to fall back to subscribing /planning/path instead.

    ros2 launch control pure_pursuit.launch.py cmd_topic:=/vehicle_command

>>> SAFETY <<< cmd_topic:=/vehicle_command sends pure_pursuit's output
straight into serial_bridge_node -- the real Arduino, no mode_manager, no
deadman switch. Wheels off the ground for the first run of any new code;
spotter present; hand on the kill switch. See README.md "Bench test through
serial_bridge".
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_share = FindPackageShare('control')
    config = PathJoinSubstitution([control_share, 'config', 'pure_pursuit.yaml'])
    default_path_file = PathJoinSubstitution([control_share, 'config', 'sample_loop.csv'])

    path_file = LaunchConfiguration('path_file')
    cmd_topic = LaunchConfiguration('cmd_topic')
    use_sim = LaunchConfiguration('use_sim')

    path_file_desc = "CSV path to load. Empty string ('') subscribes path_topic instead."
    cmd_topic_desc = ("VehicleCommand output topic. Default /cmd/auto (through "
                      "mode_manager's deadman gate). Set to /vehicle_command to bypass "
                      "mode_manager and drive serial_bridge_node directly -- see README safety "
                      "notes before doing that on hardware.")
    use_sim_desc = 'Also launch bicycle_sim_node so this runs closed-loop with no hardware.'

    return LaunchDescription([
        DeclareLaunchArgument('path_file', default_value=default_path_file,
                              description=path_file_desc),
        DeclareLaunchArgument('cmd_topic', default_value='/cmd/auto', description=cmd_topic_desc),
        DeclareLaunchArgument('use_sim', default_value='false', description=use_sim_desc),

        Node(
            package='control', executable='pure_pursuit_node', name='pure_pursuit_node',
            parameters=[config, {'path_file': path_file, 'cmd_topic': cmd_topic}],
            output='screen',
        ),
        Node(
            package='control', executable='bicycle_sim_node', name='bicycle_sim_node',
            condition=IfCondition(use_sim),
            output='screen',
        ),
    ])
