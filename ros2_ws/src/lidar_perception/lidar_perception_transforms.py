"""Reusable coordinate-transform helpers for the vehicle perception stack."""

import numpy as np


# Convert a ROS translation and quaternion into a 4x4 matrix mapping child-frame points to the parent frame.
# TF supplies quaternion + translation; Open3D RANSAC/ICP output 4x4 poses, so conversion lets TF combine with the ICP pose.
def transform_matrix(transform):
    # transform: a ROS Transform containing translation and quaternion rotation.
    # The input already contains a quaternion for rotation and a separate translation.
    # Convert the quaternion to a rotation matrix, then combine it with the translation to map a point from the child frame into the parent frame.

    # A unit quaternion (x, y, z, w) represents rotation around a unit-length axis.
    # For an axis (ax, ay, az) and angle theta:
    #   (x, y, z) = (ax, ay, az) * sin(theta / 2), and w = cos(theta / 2).
    # Example: for 90 degrees around Z, the axis is (0, 0, 1) and half the angle is 45 degrees.
    # sin(45 degrees) and cos(45 degrees) are both about 0.7071, giving the quaternion (0, 0, 0.7071, 0.7071).
    # With zero rotation, sin(0) = 0 and cos(0) = 1, so the quaternion is (0, 0, 0, 1).
    # Quaternion multiplication q * (vx, vy, vz, 0) * inverse(q) rotates a vector.
    # The equivalent 3x3 matrix below performs that same rotation on point coordinates.
    q = transform.rotation
    x, y, z, w = np.asarray([q.x, q.y, q.z, q.w], dtype=float)

    # A rotation quaternion must have length 1. Reject invalid values before dividing.
    # Example: (0, 0, 0, 0) has zero length and cannot describe a rotation.
    length = np.linalg.norm([x, y, z, w])
    if not np.isfinite(length) or length < 1e-12:
        raise ValueError("TF contains an invalid quaternion")
    # Normalize to unit length so the rotation formula produces a valid matrix.
    x, y, z, w = np.asarray([x, y, z, w]) / length

    # A 4x4 rigid transform combines rotation R and translation t in meters:
    # [ r00 r01 r02 tx ]
    # [ r10 r11 r12 ty ]    parent_point = matrix @ [child_x, child_y, child_z, 1]
    # [ r20 r21 r22 tz ]
    # [  0   0   0   1 ]
    # Start with an identity matrix, which leaves the point unchanged.
    matrix = np.eye(4)

    # Convert the normalized quaternion into the top-left 3x3 rotation block.
    # This is the standard conversion from a unit quaternion (x, y, z, w) to a 3x3 rotation matrix.
    # Example: a 90-degree rotation around Z maps [1, 0, 0] to [0, 1, 0].
    matrix[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    # Put the child frame's position in the parent frame in the last column.
    # With no rotation, a translation of [1, 0, 0] m maps [2, 3, 4] to [3, 3, 4].
    t = transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]

    # Reject NaN or infinity, including invalid translation values, before using TF.
    if not np.isfinite(matrix).all():
        raise ValueError("TF contains a non-finite transform")
    return matrix  # The complete child-to-parent transformation.
