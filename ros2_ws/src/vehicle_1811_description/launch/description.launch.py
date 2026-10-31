"""Publish the 1811 robot_description + TF tree. No GUI, no RViz.

Include this from vehicle_1811_bringup so the whole stack shares one TF tree:

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    ...
    IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare('vehicle_1811_description'),
        '/launch/description.launch.py']))
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    xacro_path = PathJoinSubstitution([
        FindPackageShare("vehicle_1811_description"),
        "urdf", "vehicle_1811.urdf.xacro",
    ])
    # Run xacro through xacro_wrapper.py, not the bare `xacro` command --
    # see that file for why: a bug in this image's xacro==2.1.1 crashes in
    # its own exception handler on a fresh subprocess (which every launch
    # is), even though the .xacro files themselves are fine.
    xacro_wrapper = PathJoinSubstitution([
        FindPackageShare("vehicle_1811_description"),
        "launch", "xacro_wrapper.py",
    ])
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": ParameterValue(
                Command(["python3 ", xacro_wrapper, " ", xacro_path]), value_type=str)}],
        ),
        # No encoders yet -> publish a static zero joint state so wheel TFs exist.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
        ),
    ])
