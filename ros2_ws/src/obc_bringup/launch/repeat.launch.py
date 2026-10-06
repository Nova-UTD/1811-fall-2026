"""REPEAT: save the route just driven, then follow it with pure pursuit.

    ros2 launch obc_bringup repeat.launch.py              # dry run -- wheels do NOT move
    ros2 launch obc_bringup repeat.launch.py live:=true   # drives the car
    ros2 launch obc_bringup repeat.launch.py route:=/vehicle_1811/routes/route_<ts>.csv

Needs bringup.launch.py still running from TEACH (same KISS-ICP session), and
teach.launch.py stopped before live:=true.

With no route:=, this stops the recorder, saves the take, and repeats the
newest route in /vehicle_1811/routes. Dry run publishes to /cmd/auto (nothing
listens) and echoes it here: push the car by hand and check steer stays
smooth and well inside +-1 before going live.

>>> live:=true sends straight to the Arduino -- no mode_manager, no deadman.
Spotter present, hand on the kill switch. See README "Safety". <<<

Not for bringup's use_mode_manager:=true mode -- there the gamepad buttons
handle TEACH/REPEAT, and this would add a second /vehicle_command publisher.
"""
import glob
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ROUTES_DIR = '/vehicle_1811/routes'

# Exits non-zero if the recorder isn't up or had nothing to save, so a stale
# route from an earlier session (different odom frame) is never repeated.
SAVE_TAKE = r"""
call() { timeout 10 ros2 service call /route_recorder_node/$1 std_srvs/srv/Trigger; }
call stop_recording > /dev/null || { echo 'route_recorder_node not reachable -- is bringup running?'; exit 1; }
out=$(call save) || exit 1
echo "$out"
grep -q 'success=True' <<< "$out"
"""


def follow(route, live):
    cmd_topic = '/vehicle_command' if live else '/cmd/auto'
    pure_pursuit = PathJoinSubstitution(
        [FindPackageShare('control'), 'launch', 'pure_pursuit.launch.py'])
    actions = [
        LogInfo(msg=f'Repeating {route} -> {cmd_topic}'
                    + ('' if live else '  (DRY RUN: wheels will not move)')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pure_pursuit),
            launch_arguments={'path_file': route, 'cmd_topic': cmd_topic}.items()),
    ]
    if not live:
        actions.append(ExecuteProcess(cmd=['ros2', 'topic', 'echo', '/cmd/auto'], output='screen'))
    return actions


def after_save(event, _context, live):
    routes = sorted(glob.glob(os.path.join(ROUTES_DIR, 'route_*.csv')))
    if event.returncode != 0 or not routes:
        return [Shutdown(reason='No route saved -- drive a TEACH loop with bringup running first.')]
    return follow(routes[-1], live)


def setup(context):
    route = LaunchConfiguration('route').perform(context)
    live = LaunchConfiguration('live').perform(context).lower() == 'true'
    if route:
        return follow(route, live)

    save = ExecuteProcess(cmd=['bash', '-c', SAVE_TAKE], output='screen')
    return [
        save,
        RegisterEventHandler(OnProcessExit(
            target_action=save,
            on_exit=lambda event, ctx: after_save(event, ctx, live))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('live', default_value='false',
                              description='true drives the car; false is a dry run.'),
        DeclareLaunchArgument('route', default_value='',
                              description='Route CSV. Empty -> save the current take and use it.'),
        OpaqueFunction(function=setup),
    ])
