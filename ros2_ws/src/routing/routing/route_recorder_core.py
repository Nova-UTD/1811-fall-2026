"""Route-recording logic, with zero ROS imports.

Mirrors control.pure_pursuit_core's split: the downsampling and file-format
decisions live here as plain Python, so they're unit-testable without
booting ROS. route_recorder_node.py is the thin rclpy wrapper around this
module.

yaw_from_quaternion is intentionally duplicated from
control.pure_pursuit_core rather than imported, so this package builds and
runs standalone without depending on `control` at runtime -- the same
reasoning pure_pursuit_core uses for not depending on tf_transformations.
"""
import math
from dataclasses import dataclass, field
from typing import List, Tuple

Point = Tuple[float, float]


def yaw_from_quaternion(q) -> float:
    """Yaw (rotation about +z) from a geometry_msgs/Quaternion-like object.
    See control.pure_pursuit_core.yaw_from_quaternion for the same formula.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class RecordedPoint:
    x: float
    y: float
    yaw: float


@dataclass
class RouteBuffer:
    """Accumulates downsampled waypoints in memory. No file/ROS I/O -- just
    the decision of whether a new pose is worth keeping.

    Downsampling is purely distance-based: a point is kept only if it's the
    first one, or at least min_spacing_m from the last *kept* point. This
    also naturally "skips near-duplicate poses" (the guide's phrasing) --
    a stationary vehicle produces a stream of near-identical poses, all of
    which fall under the threshold and get dropped, so pausing mid-teach
    doesn't pile up redundant points.
    """
    min_spacing_m: float
    points: List[RecordedPoint] = field(default_factory=list)
    length_m: float = 0.0  # cumulative distance between *kept* points

    def maybe_add(self, x: float, y: float, yaw: float) -> bool:
        """Append (x, y, yaw) if far enough from the last kept point.
        Returns True if it was added."""
        if self.points:
            last = self.points[-1]
            step = distance((last.x, last.y), (x, y))
            if step < self.min_spacing_m:
                return False
            self.length_m += step
        self.points.append(RecordedPoint(x, y, yaw))
        return True

    def clear(self):
        self.points.clear()
        self.length_m = 0.0

    def __len__(self):
        return len(self.points)


def format_csv(points: List[RecordedPoint]) -> str:
    """Render points as `x,y,yaw` rows -- exactly the format
    control.pure_pursuit_core.load_csv_path expects, so a recorded route can
    be handed straight to pure_pursuit_node's path_file param with no
    conversion step.
    """
    lines = ['# x,y,yaw -- recorded by routing/route_recorder_node']
    for p in points:
        lines.append(f'{p.x:.4f},{p.y:.4f},{p.yaw:.6f}')
    return '\n'.join(lines) + '\n'
