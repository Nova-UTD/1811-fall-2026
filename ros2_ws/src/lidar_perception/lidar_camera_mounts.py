# Compose camera mount transforms, update URDF joints, and persist accepted mounts.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

import math
import os
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from rclpy.time import Time

from lidar_camera_config import CAMERAS
from lidar_perception_transforms import transform_matrix


# ICP gives the camera cloud’s position and orientation relative to the LiDAR cloud.
# The URDF needs the camera body’s position and orientation relative to the vehicle.
# This function connects those frames.
def camera_mount_matrix(
    tf_buffer, lidar_cloud_frame, camera_cloud_frame, camera_body_frame, camera_to_lidar,
):
    # Read cloud frame IDs from the ROS headers, for example "os_lidar".
    # camera_body_frame is the URDF link, for example "zed_front_camera_link".
    # camera_to_lidar maps camera cloud points to LiDAR points. Compose it with
    # TF to get the mount matrix that maps camera body coordinates into base_link.
    # Example: the LiDAR message may be in os_lidar, not os_sensor.
    # The driver supplies os_sensor -> os_lidar, while the URDF supplies base_link -> os_sensor.
    base_from_lidar = (np.eye(4) if lidar_cloud_frame == "base_link" else transform_matrix(
        tf_buffer.lookup_transform("base_link", lidar_cloud_frame, Time()).transform))
    # The ZED cloud can be in a lens optical frame rather than the camera body.
    # Its own driver supplies this internal transform, not the camera mounting pose.
    camera_from_body = (np.eye(4) if camera_cloud_frame == camera_body_frame else transform_matrix(
        tf_buffer.lookup_transform(camera_cloud_frame, camera_body_frame, Time()).transform))

    # camera_from_body: camera body frame → camera point-cloud frame.
    # camera_to_lidar: camera point-cloud frame → LiDAR point-cloud frame, using the Open3D result.
    # base_from_lidar: LiDAR point-cloud frame → base_link.
    return base_from_lidar @ camera_to_lidar @ camera_from_body


def update_camera_joints(description, mounts):
    # description: the vehicle's expanded URDF as an XML string.
    # mounts: camera names mapped to 4x4 transforms from camera body to base_link.
    # Update only accepted camera joints in the expanded vehicle URDF.
    robot = ET.fromstring(description)
    if robot.tag != "robot" or robot.get("name") != "vehicle_1811":
        raise ValueError("robot_description is not the vehicle_1811 URDF")
    for name, matrix in mounts.items():
        matrix = np.asarray(matrix)
        # Check if the matrix is a valid rigid camera mount
        if matrix.shape != (4, 4):
            raise ValueError(f"{name}: invalid rigid camera mount")
        rotation = matrix[:3, :3]
        # Check if the rotation matrix is a valid rotation matrix
        if (not np.isfinite(matrix).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4)
                or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)):
            raise ValueError(f"{name}: invalid rigid camera mount")
        joint = robot.find(f"./joint[@name='zed_{name}_mount']")
        # Check if the joint is valid
        if (joint is None or joint.find("parent") is None or joint.find("child") is None
                or joint.find("parent").get("link") != "base_link"
                or joint.find("child").get("link") != f"zed_{name}_camera_link"):
            raise ValueError(f"{name}: expected base_link -> zed_{name}_camera_link joint not found")

        # URDF uses fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
        # We convert the rotation matrix into roll, pitch, and yaw for the URDF.
        # The matrix describes the camera body’s orientation relative to base_link.
        #             cam X     cam Y     cam Z
        #           ┌                            ┐
        # base X    │ R[0,0]    R[0,1]    R[0,2] │
        # base Y    │ R[1,0]    R[1,1]    R[1,2] │
        # base Z    │ R[2,0]    R[2,1]    R[2,2] │
        #           └                            ┘
        horizontal = math.hypot(rotation[0, 0], rotation[1, 0])
        # Combine the vertical and horizontal components to find pitch in radians.
        # In this convention rotation[2, 0] = -sin(pitch), so negate it.
        # Example: if the x-axis has no vertical component, pitch is 0.
        pitch = math.atan2(-rotation[2, 0], horizontal)
        # Near +/-90 degrees pitch, horizontal is almost 0 and roll and yaw are coupled (gimbal lock).
        # The small threshold accounts for rounding.
        if horizontal < 1e-9:
            # Choose yaw = 0 and calculate a roll that represents the orientation.
            roll = math.atan2(-rotation[1, 2], rotation[1, 1])
            yaw = 0.0
        else:
            # Otherwise, the remaining matrix entries give roll and yaw in radians.
            # atan2 uses both signs to choose the correct angle quadrant.
            # Example: a pure +90-degree Z rotation gives roll=0 and yaw=pi/2.
            roll = math.atan2(rotation[2, 1], rotation[2, 2])
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        # Find the joint's URDF origin element, or create one if it is missing.
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        # The last matrix column is the camera body's position in base_link, in meters.
        # Example: [1, 2, 3] becomes xyz="1 2 3"; values use up to 12 significant digits.
        origin.set("xyz", " ".join(f"{v:.12g}" for v in matrix[:3, 3]))
        # Write the three angles in radians in roll, pitch, yaw order.
        # Example: angles (0, 0, pi/2) become rpy="0 0 1.57079632679".
        origin.set("rpy", " ".join(f"{v:.12g}" for v in (roll, pitch, yaw)))
        joint.set("type", "fixed")  # robot_state_publisher now publishes this calibrated TF.
    return ET.tostring(robot, encoding="unicode")


def calibrated_camera_mounts(description):
    # Read accepted body -> base_link matrices from a saved expanded URDF.
    robot = ET.fromstring(description)
    if robot.tag != "robot" or robot.get("name") != "vehicle_1811":
        raise ValueError("saved calibration is not the vehicle_1811 URDF")
    mounts = {}
    for name in CAMERAS:
        joints = robot.findall(f"./joint[@name='zed_{name}_mount']")
        if not joints:
            continue  # Older saved models may contain only some cameras.
        if len(joints) != 1:
            raise ValueError(f"{name}: duplicate saved camera joints")
        joint = joints[0]
        if joint.get("type") == "floating":
            continue  # A proposal that was never accepted supplies no mount.
        parent, child, origin = joint.find("parent"), joint.find("child"), joint.find("origin")
        if (joint.get("type") != "fixed" or parent is None or child is None or origin is None
                or parent.get("link") != "base_link"
                or child.get("link") != f"zed_{name}_camera_link"):
            raise ValueError(f"{name}: invalid saved camera joint")
        xyz = np.array([float(value) for value in origin.get("xyz", "0 0 0").split()])
        rpy = np.array([float(value) for value in origin.get("rpy", "0 0 0").split()])
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
            raise ValueError(f"{name}: invalid saved camera origin")
        roll, pitch, yaw = rpy
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                math.sin(pitch), math.cos(yaw), math.sin(yaw))
        # cr and sr are cosine and sine of roll. cp/sp and cy/sy are the
        # same for pitch and yaw. The saved angles are in radians.
        mount = np.eye(4)
        mount[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]]
        mount[:3, 3] = xyz
        mounts[name] = mount
    return mounts


# Atomically save the accepted expanded URDF outside the Git checkout.
def save_calibrated_urdf(urdf_xml, output_path):
    # Write the accepted URDF text to a temporary sibling of output_path,
    # then replace the published file after the temporary file is flushed.
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=output_path.parent,
                                         prefix=".calibration-", delete=False) as stream:
            temporary_path = stream.name
            stream.write(urdf_xml)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, output_path)
        directory_fd = os.open(output_path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary_path and os.path.exists(temporary_path):
            os.unlink(temporary_path)
