"""The four ZED X cameras on the 1811 Jetson -- run natively on the Jetson, not in Docker.

    source ~/zed_ws/install/local_setup.bash
    source ~/1811-fall-2026/ros2_ws/install/local_setup.bash
    ros2 launch jetson_bringup zed_cameras.launch.py

    cameras:=front                 start only some cameras (comma-separated)
    front_serial:=12345678         pick a camera by serial number (from its sticker)
    front_id:=0                    ...or by ZED Link port order (used when serial is 0)

One zed_wrapper instance per camera, named zed_<position>. That makes each
camera's TF tree start at zed_<position>_camera_link -- the exact frame
vehicle_1811_description's URDF already places on the car -- so the wrapper's
frames hang off the URDF instead of duplicating it (see the zed_x_camera macro
note in vehicle_1811.macros.xacro). publish_tf / publish_map_tf are off: the
ZED's own odometry must not compete with KISS-ICP for the vehicle's pose.

Image topics: /zed_<position>/zed_node/rgb/color/rect/image (zed-ros2-wrapper
v5). Resolution, frame rate and depth settings are in config/zed_1811.yaml.

The <position>_id defaults (front=0, left=1, rear=2, right=3) are a guess at
the ZED Link port order. Until serial numbers are filled in, check each
camera's image against where it points and swap ids if needed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

POSITIONS = ('front', 'left', 'rear', 'right')
DEFAULT_IDS = {'front': '0', 'left': '1', 'rear': '2', 'right': '3'}


def camera(position, context):
    serial = LaunchConfiguration(f'{position}_serial').perform(context)
    camera_id = LaunchConfiguration(f'{position}_id').perform(context)
    # A serial number pins the exact camera; otherwise fall back to the port id.
    if serial not in ('', '0'):
        camera_id = '-1'
    else:
        serial = '0'

    zed_launch = PathJoinSubstitution(
        [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])
    overrides = PathJoinSubstitution(
        [FindPackageShare('jetson_bringup'), 'config', 'zed_1811.yaml'])
    return [
        LogInfo(msg=f'zed_{position}: serial={serial} camera_id={camera_id}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_name': f'zed_{position}',
                'camera_model': LaunchConfiguration('camera_model'),
                'serial_number': serial,
                'camera_id': camera_id,
                'ros_params_override_path': overrides,
                'publish_urdf': 'true',
                'publish_tf': 'false',
                'publish_map_tf': 'false',
            }.items()),
    ]


def setup(context):
    wanted = [c.strip() for c in LaunchConfiguration('cameras').perform(context).split(',')]
    unknown = [c for c in wanted if c not in POSITIONS]
    if unknown:
        raise RuntimeError(f'Unknown camera(s) {unknown}; choose from {", ".join(POSITIONS)}')
    actions = []
    for position in POSITIONS:
        if position in wanted:
            actions += camera(position, context)
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('cameras', default_value=','.join(POSITIONS),
                              description='Comma-separated subset of front,left,rear,right.'),
        DeclareLaunchArgument('camera_model', default_value='zedx',
                              description='ZED model, as zed_wrapper names it.'),
    ]
    for position in POSITIONS:
        args += [
            DeclareLaunchArgument(f'{position}_serial', default_value='0',
                                  description=f'{position} camera serial number (0 = use id).'),
            DeclareLaunchArgument(f'{position}_id', default_value=DEFAULT_IDS[position],
                                  description=f'{position} camera ZED Link port id.'),
        ]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
