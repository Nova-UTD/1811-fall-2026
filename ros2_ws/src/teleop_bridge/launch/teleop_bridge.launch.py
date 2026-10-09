"""Teleop bringup: gamepad -> /vehicle_command -> Arduino serial.

    # full manual teleop (joy_node + gamepad_node + serial_bridge_node)
    ros2 launch teleop_bridge teleop_bridge.launch.py

    # serial bridge ONLY -- no gamepad publishing to /vehicle_command.
    # This is what obc_bringup uses: mode_manager_node is the one publisher on
    # /vehicle_command and nothing else competes with it for the topic.
    ros2 launch teleop_bridge teleop_bridge.launch.py use_gamepad:=false

    # override the auto-detected serial port
    ros2 launch teleop_bridge teleop_bridge.launch.py port:=/dev/ttyACM0

SERIAL PORT: `port` defaults to 'auto', which resolves the Arduino through
/dev/serial/by-id/ (stable across replugs) instead of a /dev/ttyACM<N> number
(assigned in plug order, changes on every reboot -- that's what silently
pointed the bridge at a Linux serial console and produced the
`{speed:: command not found` errors). Pass an explicit path to override.
See `ls -l /dev/serial/by-id/`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    use_gamepad = LaunchConfiguration('use_gamepad')

    use_gamepad_desc = (
        'Start joy_node + gamepad_node alongside the serial bridge. Set false '
        'when something else owns /vehicle_command (obc_bringup\'s '
        'mode_manager_node) -- two publishers on the same topic means the '
        'Arduino acts on whichever message arrived last.')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Arduino serial port. Enumeration-order dependent; prefer a '
                        'stable /dev/serial/by-id/... path (see header).',
            # default_value='auto',
            # description="Arduino serial port. 'auto' resolves via "
            #             '/dev/serial/by-id/; or pass an explicit path.',
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='57600',
            description='Serial baud rate; must match the firmware Serial.begin().',
        ),
        DeclareLaunchArgument(
            'use_gamepad', default_value='true', description=use_gamepad_desc,
        ),

        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{'deadzone': 0.05}],
            condition=IfCondition(use_gamepad),
        ),
        Node(
            package='teleop_bridge', executable='gamepad_node', name='gamepad_node',
            condition=IfCondition(use_gamepad),
        ),
        Node(
            package='teleop_bridge', executable='serial_bridge_node',
            name='serial_bridge_node',
            parameters=[{
                'port': port,
                'baud': ParameterValue(baud, value_type=int),
            }],
            output='screen',
        ),
    ])
