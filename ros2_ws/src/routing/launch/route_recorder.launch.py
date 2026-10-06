"""Launch route_recorder_node.

    ros2 launch routing route_recorder.launch.py                    # saves to /vehicle_1811/routes
    ros2 launch routing route_recorder.launch.py output_dir:=/some/other/dir
    ros2 launch routing route_recorder.launch.py record_on_start:=false

Requires /odometry already publishing (localization.launch.py) in the SAME
session you intend to repeat in -- see the package README before recording
a route you plan to actually drive back.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odometry'),
        DeclareLaunchArgument('min_spacing_m', default_value='0.15'),
        DeclareLaunchArgument('output_dir', default_value='/vehicle_1811/routes',
                              description="Gitignored. Empty string -> <cwd>/routes instead"),
        DeclareLaunchArgument('output_file', default_value='',
                              description='Empty -> auto-generated timestamped filename'),
        DeclareLaunchArgument('record_on_start', default_value='true'),

        Node(
            package='routing', executable='route_recorder_node', name='route_recorder_node',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'min_spacing_m': LaunchConfiguration('min_spacing_m'),
                'output_dir': LaunchConfiguration('output_dir'),
                'output_file': LaunchConfiguration('output_file'),
                'record_on_start': LaunchConfiguration('record_on_start'),
            }],
            output='screen',
        ),
    ])
