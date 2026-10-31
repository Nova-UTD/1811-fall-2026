"""Pure pursuit geometry, with zero ROS imports.

Every function here is plain Python + math, deliberately kept free of
rclpy/message types so it can be unit-tested (test/test_pure_pursuit_core.py)
without booting ROS, and reused from a bag-replay script or a notebook.
pure_pursuit_node.py is the thin rclpy wrapper around this module.
"""
import csv
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float  # radians; from the data if given, else inferred from bearing

    @property
    def xy(self) -> Point:
        return (self.x, self.y)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _infer_yaws(xy: Sequence[Point]) -> List[float]:
    """Bearing to the next point; the last point repeats the final bearing."""
    yaws = []
    for i in range(len(xy)):
        if i + 1 < len(xy):
            dx, dy = xy[i + 1][0] - xy[i][0], xy[i + 1][1] - xy[i][1]
        else:
            dx, dy = xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1]
        yaws.append(math.atan2(dy, dx))
    return yaws


def path_from_xy(points: Sequence[Point]) -> List[Waypoint]:
    """Build a path from bare (x, y) pairs (e.g. a nav_msgs/Path), inferring yaw."""
    yaws = _infer_yaws(points)
    return [Waypoint(x, y, yaw) for (x, y), yaw in zip(points, yaws)]


def load_csv_path(path_file: str) -> List[Waypoint]:
    """Load ordered (x, y[, yaw]) waypoints from a CSV file.

    Accepts an optional header row (skipped if its first field doesn't parse
    as a float), blank lines, and '#' comment lines. If no yaw column is
    present, yaw is inferred from bearing between consecutive waypoints.
    """
    rows: List[Tuple[float, float, Optional[float]]] = []
    header_skipped = False
    with open(path_file, newline='') as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if not row or not row[0].strip() or row[0].strip().startswith('#'):
                continue
            try:
                x, y = float(row[0]), float(row[1])
            except ValueError:
                # Header row, e.g. "x,y,yaw": the first non-comment line, which
                # may come after comments or blank lines -- not just line 1.
                if not rows and not header_skipped:
                    header_skipped = True
                    continue
                raise ValueError(f'{path_file}:{i + 1}: could not parse {row!r} as x,y[,yaw]')
            has_yaw = len(row) > 2 and row[2].strip() != ''
            rows.append((x, y, float(row[2]) if has_yaw else None))

    if len(rows) < 2:
        raise ValueError(f'{path_file}: need at least 2 waypoints, found {len(rows)}')

    if all(yaw is not None for _, _, yaw in rows):
        return [Waypoint(x, y, yaw) for x, y, yaw in rows]
    # Missing yaw on any row -- infer the whole path from bearings instead
    # of mixing recorded and inferred values point-to-point.
    xy = [(x, y) for x, y, _ in rows]
    return path_from_xy(xy)


def yaw_from_quaternion(q) -> float:
    """Yaw (rotation about +z) from a geometry_msgs/Quaternion-like object.

    Ignores roll/pitch -- fine for a ground vehicle assumed roughly planar.
    Written out by hand instead of depending on tf_transformations, which
    isn't installed in this workspace's Docker image.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def is_near_path_end(idx: int, path_len: int, margin: int = 1) -> bool:
    """True once the forward-search index has actually reached the tail of
    the path (within `margin` points of the last one).

    Goal detection must require this, not just spatial distance to the
    final waypoint -- a loop-shaped route's last waypoint sits close to its
    first one, so a vehicle starting REPEAT from the same spot TEACH just
    ended (the documented workflow: no restart between them) would
    otherwise already be "at the goal" before ever following the path.
    Requiring the index to have actually walked to the end forces genuine
    progress, not just spatial coincidence.
    """
    return idx >= path_len - 1 - margin


def closest_index(path: Sequence[Waypoint], pose: Point, start_idx: int, window: int = 25) -> int:
    """Index of the path point closest to `pose`, searched forward only.

    Scans a bounded window ahead of start_idx (cheap -- no full-path rescan
    every tick) and never looks backward, so it can't snap onto a later
    self-intersection of the path behind where the vehicle actually is. If
    the best point found is right at the window's edge, extends once more
    in case the true closest point is further out still (e.g. after a
    pause) rather than silently under-searching.
    """
    n = len(path)
    idx = clamp(start_idx, 0, n - 1)
    best_idx = int(idx)
    best_d = distance(path[best_idx].xy, pose)
    end = min(n, best_idx + window)
    for i in range(best_idx + 1, end):
        d = distance(path[i].xy, pose)
        if d < best_d:
            best_d, best_idx = d, i
    if best_idx == end - 1 and end < n:
        return closest_index(path, pose, best_idx, window)
    return best_idx


def _interpolate_to_radius(p0: Point, p1: Point, center: Point, radius: float) -> Point:
    """Point on segment p0->p1 exactly `radius` from `center`.

    Assumes p0 is inside the radius and p1 is outside it (the caller
    guarantees this), so the quadratic below always has a root in [0, 1].
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    fx, fy = p0[0] - center[0], p0[1] - center[1]
    a = dx * dx + dy * dy
    if a < 1e-12:
        return p1
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = max(0.0, b * b - 4.0 * a * c)
    t = (-b + math.sqrt(disc)) / (2.0 * a)
    t = clamp(t, 0.0, 1.0)
    return (p0[0] + t * dx, p0[1] + t * dy)


def find_lookahead_point(
    path: Sequence[Waypoint], pose: Point, start_idx: int, lookahead_dist: float,
) -> Tuple[Point, int, bool]:
    """Walk forward from start_idx to the point ~lookahead_dist ahead of pose.

    Returns (point, index, reached_end). Interpolates between the last
    waypoint inside the lookahead radius and the first one outside it, so
    the lookahead point sits precisely at the requested distance instead of
    snapping to whichever recorded waypoint happens to be nearest.
    reached_end=True means the path ran out before reaching lookahead_dist
    (near the goal) -- the caller should use the actual achieved distance,
    not lookahead_dist, in the curvature formula.
    """
    n = len(path)
    idx = clamp(start_idx, 0, n - 1)
    prev = path[idx].xy
    if distance(prev, pose) >= lookahead_dist:
        return prev, idx, False

    for i in range(idx + 1, n):
        cur = path[i].xy
        if distance(cur, pose) >= lookahead_dist:
            return _interpolate_to_radius(prev, cur, pose, lookahead_dist), i, False
        prev = cur

    return path[-1].xy, n - 1, True


def world_to_body(pose: Point, yaw: float, point: Point) -> Point:
    """Rotate `point` from the world/odom frame into the vehicle body frame."""
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(yaw), math.sin(yaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    return local_x, local_y


def curvature_from_local_y(local_y: float, lookahead_dist: float) -> float:
    """kappa = 2*y_local / L**2. Positive kappa curves left (REP103: +y = left)."""
    if lookahead_dist < 1e-6:
        return 0.0
    return 2.0 * local_y / (lookahead_dist * lookahead_dist)


def steering_angle(curvature: float, wheelbase: float, max_steer_angle: float) -> float:
    """Bicycle-model steering angle (radians), clamped to +/- max_steer_angle."""
    delta = math.atan(curvature * wheelbase)
    return clamp(delta, -max_steer_angle, max_steer_angle)


def normalize_steer(delta: float, max_steer_angle: float, sign: float = 1.0) -> float:
    """Radians -> the normalized -1..1 vehicle_msgs/VehicleCommand.steer field.

    `sign` exists purely for blocks calibration (see gamepad_node's
    INVERT_STEER) -- flip it with a launch param if the wheels steer away
    from the path instead of toward it, no code change needed.
    """
    if max_steer_angle < 1e-6:
        return 0.0
    return clamp(sign * delta / max_steer_angle, -1.0, 1.0)


def scaled_lookahead(
    speed: float, base: float, gain: float, min_ld: float, max_ld: float,
) -> float:
    """Ld = base + gain*speed, clamped. gain=0 -> fixed lookahead (the v1 default).

    A fixed Ld oscillates at low speed and cuts corners at high speed; once
    target_speed_mps varies, set lookahead_speed_gain > 0 to scale with it.
    """
    return clamp(base + gain * max(speed, 0.0), min_ld, max_ld)


def curvature_speed(target_speed: float, curvature: float, gain: float, min_speed: float) -> float:
    """Optional speed reduction on tight curves. gain=0 -> constant target_speed."""
    if gain <= 0.0:
        return target_speed
    return max(min_speed, target_speed / (1.0 + gain * abs(curvature)))


def step_bicycle_model(
    x: float, y: float, yaw: float, throttle: float, steer: float, dt: float,
    wheelbase: float, max_speed_mps: float, max_steer_angle: float,
) -> Tuple[float, float, float, float]:
    """Integrate one dt of a kinematic bicycle model from a VehicleCommand.

    Used by bicycle_sim_node to close the loop around pure_pursuit_node
    without hardware. Returns (x, y, yaw, speed_mps).
    """
    v = clamp(throttle, -1.0, 1.0) * max_speed_mps
    delta = clamp(steer, -1.0, 1.0) * max_steer_angle
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += (v / wheelbase) * math.tan(delta) * dt
    return x, y, yaw, v
