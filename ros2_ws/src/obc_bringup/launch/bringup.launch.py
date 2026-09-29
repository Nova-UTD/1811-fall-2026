"""Everything that stays up for the whole session, in one terminal.

    ros2 launch obc_bringup bringup.launch.py lidar_ip:=<sensor-ip>

Starts: URDF + TF tree, Ouster driver, KISS-ICP localization, route recorder
(recording from startup), and the serial bridge to the Arduino (no gamepad --
that's teach.launch.py, so it can be stopped without stopping this).

Leave this running from TEACH through the end of REPEAT. Restarting it
restarts KISS-ICP, which moves the `odom` origin and silently invalidates the
route you just recorded.

    port:=/dev/serial/by-id/...   override the Arduino serial port
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, **args):
    path = PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
    return IncludeLaunchDescription(AnyLaunchDescriptionSource(path),
                                    launch_arguments=args.items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_ip', description='Ouster sensor IP or hostname.'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Arduino serial port.'),

        include('vehicle_1811_description', 'description.launch.py'),
        # viz:=false -- no monitor on the Karbon. LEGACY -- sensor fw 3.0.1 bug.
        include('ouster_ros', 'sensor.launch.xml',
                sensor_hostname=LaunchConfiguration('lidar_ip'),
                viz='false',
                udp_profile_lidar='LEGACY'),
        include('localization', 'localization.launch.py'),
        include('routing', 'route_recorder.launch.py'),
        include('teleop_bridge', 'teleop_bridge.launch.py',
                use_gamepad='false',
                port=LaunchConfiguration('port')),
    ])
