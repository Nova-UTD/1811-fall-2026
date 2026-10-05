"""Everything that stays up for the whole session, in one terminal.

    ros2 launch obc_bringup bringup.launch.py lidar_ip:=<sensor-ip>

Starts: URDF + TF tree, Ouster driver, KISS-ICP localization, route recorder
(recording from startup), and the serial bridge to the Arduino (no gamepad --
that's teach.launch.py, so it can be stopped without stopping this).

Leave this running from TEACH through the end of REPEAT. Restarting it
restarts KISS-ICP, which moves the `odom` origin and silently invalidates the
route you just recorded.

    port:=/dev/serial/by-id/...   override the Arduino serial port

ONE-LAUNCH MODE (use_mode_manager:=true) -- EXPERIMENTAL, not yet driven on the car:

    ros2 launch obc_bringup bringup.launch.py lidar_ip:=<sensor-ip> use_mode_manager:=true

Also starts the gamepad, pure_pursuit_node and mode_manager_node, and the
recorder waits for the record button instead of recording from startup. The
whole teach-and-repeat cycle is then driven from the gamepad -- teach.launch.py
and repeat.launch.py are NOT used (running them alongside would put a second
publisher on /vehicle_command). See ros2_ws/src/mode_manager/README.md.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition=None, **args):
    path = PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
    return IncludeLaunchDescription(AnyLaunchDescriptionSource(path),
                                    launch_arguments=args.items(),
                                    condition=condition)


def generate_launch_description():
    use_mode_manager = LaunchConfiguration('use_mode_manager')
    with_mode_manager = IfCondition(use_mode_manager)
    # Legacy flow records from startup; with the mode manager, the record button starts it.
    record_on_start = PythonExpression(["'false' if '", use_mode_manager, "' == 'true' else 'true'"])
    mode_manager_config = PathJoinSubstitution(
        [FindPackageShare('mode_manager'), 'config', 'mode_manager.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('lidar_ip', description='Ouster sensor IP or hostname.'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Arduino serial port.'),
        DeclareLaunchArgument('use_mode_manager', default_value='false',
                              description='true: one-launch mode, gamepad buttons switch '
                                          'DISABLED / MANUAL / AUTONOMOUS. EXPERIMENTAL.'),

        include('vehicle_1811_description', 'description.launch.py'),
        # viz:=false -- no monitor on the Karbon. LEGACY -- sensor fw 3.0.1 bug.
        include('ouster_ros', 'sensor.launch.xml',
                sensor_hostname=LaunchConfiguration('lidar_ip'),
                viz='false',
                udp_profile_lidar='LEGACY'),
        include('localization', 'localization.launch.py'),
        include('routing', 'route_recorder.launch.py', record_on_start=record_on_start),
        include('teleop_bridge', 'teleop_bridge.launch.py',
                use_gamepad='false',
                port=LaunchConfiguration('port')),

        # --- one-launch mode only -------------------------------------------
        # Gamepad publishes to /cmd/manual; only mode_manager_node writes
        # /vehicle_command.
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'deadzone': 0.05}], condition=with_mode_manager),
        Node(package='teleop_bridge', executable='gamepad_node', name='gamepad_node',
             remappings=[('/vehicle_command', '/cmd/manual')], condition=with_mode_manager),
        # path_file:='' -> follows whatever route the recorder publishes on save.
        include('control', 'pure_pursuit.launch.py', condition=with_mode_manager,
                path_file='', cmd_topic='/cmd/auto'),
        Node(package='mode_manager', executable='mode_manager_node', name='mode_manager_node',
             parameters=[mode_manager_config], output='screen', condition=with_mode_manager),
    ])
