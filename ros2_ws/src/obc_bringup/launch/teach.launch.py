"""TEACH: drive the loop by hand with the gamepad.

    ros2 launch obc_bringup teach.launch.py

Needs bringup.launch.py running (it owns the serial bridge and the recorder).
Ctrl-C this when the loop is done -- it MUST be stopped before
`repeat.launch.py live:=true`, or the gamepad and pure pursuit both publish
/vehicle_command and the Arduino acts on whichever arrived last.

Not for bringup's use_mode_manager:=true mode -- there the gamepad buttons
handle TEACH/REPEAT, and this would add a second /vehicle_command publisher.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'deadzone': 0.05}]),
        Node(package='teleop_bridge', executable='gamepad_node', name='gamepad_node'),
    ])
