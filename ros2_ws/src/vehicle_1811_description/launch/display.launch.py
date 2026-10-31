"""Visualize the 1811 URDF in RViz with a joint slider GUI.

    ros2 launch vehicle_1811_description display.launch.py

Use this to eyeball frame placement. For the real placement check (roadmap
step 1 "done when"), also start the ouster driver and add a PointCloud2 display
on /ouster/points in the os_lidar frame — the cloud should sit correctly against
this model. Set gui:=false to drop the slider window.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = FindPackageShare("vehicle_1811_description")
    xacro_path = PathJoinSubstitution([pkg, "urdf", "vehicle_1811.urdf.xacro"])
    rviz_path = PathJoinSubstitution([pkg, "rviz", "vehicle.rviz"])

    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_path]), value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Start joint_state_publisher_gui sliders"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
