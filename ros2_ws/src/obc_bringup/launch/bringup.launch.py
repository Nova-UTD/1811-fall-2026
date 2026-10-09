"""The whole vehicle stack, in one terminal -- manual driving and teach & repeat.

    ros2 launch obc_bringup bringup.launch.py lidar_ip:=<sensor-ip> port:=/dev/serial/by-id/...

Starts: URDF + TF tree, Ouster driver, KISS-ICP localization, route recorder,
serial bridge to the Arduino, the gamepad, pure_pursuit_node, and
mode_manager_node -- the only publisher on /vehicle_command. Everything after
launch is on the gamepad: A = MANUAL, Y = record a route, Start + hold RB =
repeat it, B = stop. See ros2_ws/src/mode_manager/README.md.

Leave this running from TEACH through the end of REPEAT. Restarting it
restarts KISS-ICP, which moves the `odom` origin and silently invalidates the
route you just recorded.

    port:=/dev/serial/by-id/...   the Arduino serial port (default /dev/ttyACM0)
    use_mode_manager:=false       sensors + serial bridge only: no gamepad, no
                                  autonomy, recorder records from startup
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
    # Without the mode manager, record from startup; with it, the record button starts it.
    record_on_start = PythonExpression(
        ["'false' if '", use_mode_manager, "' == 'true' else 'true'"])
    mode_manager_config = PathJoinSubstitution(
        [FindPackageShare('mode_manager'), 'config', 'mode_manager.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('lidar_ip', description='Ouster sensor IP or hostname.'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Arduino serial port.'),
        DeclareLaunchArgument('use_mode_manager', default_value='true',
                              description='true: gamepad + pure pursuit + mode_manager '
                                          '(DISABLED / MANUAL / AUTONOMOUS). false: sensors '
                                          'and serial bridge only.'),

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

        # --- mode manager (default; off with use_mode_manager:=false) ---------
        # Gamepad publishes to /cmd/manual; only mode_manager_node writes
        # /vehicle_command.
        # autorepeat_rate: /joy must keep arriving while the sticks are held
        # still -- mode_manager treats 0.5 s of silence as a lost gamepad.
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}],
             condition=with_mode_manager),
        Node(package='teleop_bridge', executable='gamepad_node', name='gamepad_node',
             remappings=[('/vehicle_command', '/cmd/manual')], condition=with_mode_manager),
        # path_file:='' -> follows whatever route the recorder publishes on save.
        include('control', 'pure_pursuit.launch.py', condition=with_mode_manager,
                path_file='', cmd_topic='/cmd/auto'),
        Node(package='mode_manager', executable='mode_manager_node', name='mode_manager_node',
             parameters=[mode_manager_config], output='screen', condition=with_mode_manager),
    ])
