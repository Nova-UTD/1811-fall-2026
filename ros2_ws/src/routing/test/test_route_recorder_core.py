"""Unit tests for routing.route_recorder_core -- no ROS required.

Run with `pytest test/test_route_recorder_core.py`, or as part of
`colcon test --packages-select routing`.
"""
import math

import pytest

from routing import route_recorder_core as rrc


class FakeQuaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


def quat_from_yaw(yaw: float) -> FakeQuaternion:
    return FakeQuaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def test_yaw_from_quaternion_round_trips():
    for yaw in (0.0, 0.5, -1.2, math.pi / 2):
        recovered = rrc.yaw_from_quaternion(quat_from_yaw(yaw))
        assert recovered == pytest.approx(yaw, abs=1e-9)


def test_route_buffer_always_keeps_the_first_point():
    buf = rrc.RouteBuffer(min_spacing_m=0.15)
    assert buf.maybe_add(0.0, 0.0, 0.0) is True
    assert len(buf) == 1


def test_route_buffer_skips_near_duplicate_poses():
    buf = rrc.RouteBuffer(min_spacing_m=0.15)
    buf.maybe_add(0.0, 0.0, 0.0)
    # simulates a stationary vehicle -- tiny GPS/ICP jitter, no real travel
    assert buf.maybe_add(0.01, 0.0, 0.0) is False
    assert buf.maybe_add(0.05, 0.02, 0.0) is False
    assert len(buf) == 1


def test_route_buffer_keeps_points_past_the_spacing_threshold():
    buf = rrc.RouteBuffer(min_spacing_m=0.15)
    buf.maybe_add(0.0, 0.0, 0.0)
    assert buf.maybe_add(0.2, 0.0, 0.0) is True
    assert len(buf) == 2


def test_route_buffer_tracks_cumulative_length():
    buf = rrc.RouteBuffer(min_spacing_m=0.1)
    buf.maybe_add(0.0, 0.0, 0.0)
    buf.maybe_add(1.0, 0.0, 0.0)
    buf.maybe_add(1.0, 1.0, 0.0)
    assert buf.length_m == pytest.approx(2.0)


def test_route_buffer_clear_resets_length_too():
    buf = rrc.RouteBuffer(min_spacing_m=0.1)
    buf.maybe_add(0.0, 0.0, 0.0)
    buf.maybe_add(1.0, 0.0, 0.0)
    buf.clear()
    assert len(buf) == 0
    assert buf.length_m == 0.0


def test_format_csv_round_trips_through_pure_pursuit_core(tmp_path):
    # The point of this test: verify routing and control actually agree on
    # the file format, not just by convention/documentation.
    from control import pure_pursuit_core as ppc

    buf = rrc.RouteBuffer(min_spacing_m=0.1)
    buf.maybe_add(0.0, 0.0, 0.0)
    buf.maybe_add(1.0, 0.0, 0.1)
    buf.maybe_add(1.0, 1.0, 1.5708)

    csv_text = rrc.format_csv(buf.points)
    csv_file = tmp_path / 'recorded.csv'
    csv_file.write_text(csv_text)

    loaded = ppc.load_csv_path(str(csv_file))
    assert len(loaded) == 3
    for recorded, waypoint in zip(buf.points, loaded):
        assert waypoint.x == pytest.approx(recorded.x, abs=1e-3)
        assert waypoint.y == pytest.approx(recorded.y, abs=1e-3)
        assert waypoint.yaw == pytest.approx(recorded.yaw, abs=1e-5)
