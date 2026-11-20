#!/usr/bin/env python3
# Start the LiDAR-camera registration node on the Jetson.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

import sys

from lidar_camera_config import is_jetson_link


if __name__ == "__main__":
    if not is_jetson_link():
        print("LiDAR and camera registration requires the Jetson cable address 192.168.100.2; "
              "refusing to process sensor data on this computer.", file=sys.stderr)
        raise SystemExit(2)

    # Load ROS and Open3D only after the host check has passed.
    from lidar_camera_node import main

    main()
