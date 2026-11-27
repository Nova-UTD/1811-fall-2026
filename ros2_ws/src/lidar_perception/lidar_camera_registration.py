#!/usr/bin/env python3
# Start the LiDAR-camera registration node on the Jetson.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

import socket
import sys


def is_jetson_link():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as link:
            # UDP connect selects the local cable address without sending a packet.
            link.connect(('192.168.100.1', 9))
            return link.getsockname()[0] == '192.168.100.2'
    except OSError:
        return False


if __name__ == '__main__':
    if not is_jetson_link():
        print('LiDAR and camera registration requires the Jetson cable address 192.168.100.2; '
              'refusing to process sensor data on this computer.', file=sys.stderr)
        raise SystemExit(2)

    # Load ROS and Open3D only after the host check has passed.
    from pathlib import Path

    from ament_index_python.packages import get_package_share_directory

    from lidar_camera_node import main

    # Both ros2 run and ros2 launch load 1811 wiring before any user overrides.
    package_share = Path(get_package_share_directory('lidar_perception'))
    vehicle_profile = package_share / 'config' / 'registration.yaml'
    main(['--ros-args', '--params-file', str(vehicle_profile), '--', *sys.argv[1:]])
