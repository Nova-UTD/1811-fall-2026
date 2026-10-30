"""The whole vehicle stack, in one terminal -- manual driving and teach & repeat.

    ros2 launch obc_bringup bringup.launch.py lidar_ip:=<sensor-ip> port:=/dev/serial/by-id/...

Starts: URDF + TF tree, Ouster driver, KISS-ICP localization, the corner
detector's lidar and edge-point viewer, route recorder,
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
    cameras:=false                don't start the Jetson's four ZED cameras
    cameras:=front,rear           start only some of them
    panorama:=true                also start the 360-degree panorama on the Jetson

CAMERAS: by default this also starts the four ZED X cameras on the Jetson, over
SSH (scripts/jetson_cameras.sh), and Ctrl-C here stops them too. Needs the
one-time key setup on the Karbon: bash scripts/setup_jetson_ssh.sh. If the
Jetson can't be reached, the car still starts and this says why.
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
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


def jetson_cameras(context):
    """Start the cameras, panorama, and/or corner_detector on the Jetson over SSH."""
    cameras = LaunchConfiguration('cameras').perform(context).strip().lower()
    corners_on_jetson = LaunchConfiguration('corners_on_jetson').perform(context).strip().lower() == 'true'
    if cameras == 'false' and not corners_on_jetson:
        return []
    host = LaunchConfiguration('jetson').perform(context)
    remote = f"bash {LaunchConfiguration('jetson_repo').perform(context)}/scripts/jetson_cameras.sh"
    if LaunchConfiguration('panorama').perform(context).lower() == 'true':
        remote += ' --panorama'
    if corners_on_jetson:
        remote += ' --corners'
    if cameras == 'false':
        remote += ' --no-cameras'
    elif cameras != 'true':  # a list such as front,rear
        if not re.fullmatch(r'[a-z]+(,[a-z]+)*', cameras.replace(' ', '')):
            return [LogInfo(msg=f'Jetson cameras not started: cameras:={cameras} should be '
                                f'true, false, or a list such as front,rear.')]
        remote += f" --cameras={cameras.replace(' ', '')}"
    script = os.path.join(get_package_share_directory('obc_bringup'),
                          'scripts', 'jetson_cameras_ssh.sh')
    if not os.path.exists(script):  # pulled but not rebuilt
        return [LogInfo(msg='Jetson cameras not started: rebuild first '
                            '(colcon build --symlink-install --packages-ignore '
                            'umrr_ros2_driver umrr_ros2_msgs smart_rviz_plugin).')]
    # The script stops the Jetson side cleanly on Ctrl-C, which can take a few
    # seconds while the cameras close -- hence the longer sigterm_timeout.
    ssh = ExecuteProcess(
        name='jetson_cameras', output='screen', sigterm_timeout='20',
        cmd=['bash', script, host, LaunchConfiguration('jetson_key').perform(context), remote])

    def explain(event, ctx):
        if ctx.is_shutdown or event.returncode in (0, -2, -15):
            return []
        return [LogInfo(msg=(
            f'Jetson cameras did not start (exit {event.returncode}); the car is fine. '
            f'Check: the Jetson is on (ping 192.168.100.2), the one-time key setup was done '
            f'(bash scripts/setup_jetson_ssh.sh on the Karbon), and the Jetson repo is pulled. '
            f'To skip the cameras: cameras:=false.'))]

    return [ssh, RegisterEventHandler(OnProcessExit(target_action=ssh, on_exit=explain))]


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
        DeclareLaunchArgument('cameras', default_value='true',
                              description='Start the ZED cameras on the Jetson over SSH: '
                                          'true (all four), false, or a list such as '
                                          'front,rear.'),
        DeclareLaunchArgument('panorama', default_value='false',
                              description='Also start the 360-degree panorama on the Jetson.'),
        DeclareLaunchArgument('corners_on_jetson', default_value='false',
                              description='Also run corner_detector on the Jetson (requires a display).'),
        DeclareLaunchArgument('jetson', default_value='nova@192.168.100.2',
                              description='SSH user@host of the Jetson (the cable address).'),
        DeclareLaunchArgument('jetson_repo', default_value='~/1811-fall-2026',
                              description='Where this repo is cloned on the Jetson.'),
        DeclareLaunchArgument('jetson_key', default_value='/root/.ssh/id_ed25519_jetson',
                              description='Key from scripts/setup_jetson_ssh.sh, as seen '
                                          'inside the container.'),

        include('vehicle_1811_description', 'description.launch.py'),
        # viz:=false -- no monitor on the Karbon. LEGACY -- sensor fw 3.0.1 bug.
        include('ouster_ros', 'sensor.launch.xml',
                sensor_hostname=LaunchConfiguration('lidar_ip'),
                viz='false',
                udp_profile_lidar='LEGACY'),
        include('localization', 'localization.launch.py'),
        include('lidar_perception', 'corner_detector.launch.py'),
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

        # --- Jetson: cameras (+ panorama), started and stopped with the car -----
        OpaqueFunction(function=jetson_cameras),
    ])
