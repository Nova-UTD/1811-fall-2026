"""Unit tests for control.pure_pursuit_core -- no ROS required.

Run directly with `pytest test/test_pure_pursuit_core.py`, or as part of
`colcon test --packages-select control`. This is the "unit-test against a
hand-written path" step: it exercises the same geometry pure_pursuit_node
runs, against synthetic straight and curved paths, without needing odometry,
a path topic, or hardware.
"""
import math

import pytest

from control import pure_pursuit_core as ppc


def straight_path(n=20, spacing=0.5):
    xy = [(i * spacing, 0.0) for i in range(n)]
    return ppc.path_from_xy(xy)


def quarter_circle_path(radius=5.0, n=30):
    # Vehicle travels +x, path curves to the left (+y) -- a left turn.
    xy = [(radius * math.sin(t), radius * (1 - math.cos(t)))
          for t in (i * (math.pi / 2) / (n - 1) for i in range(n))]
    return ppc.path_from_xy(xy)


# --- CSV loading -------------------------------------------------------

def test_load_csv_path_infers_yaw_when_missing(tmp_path):
    csv_file = tmp_path / 'path.csv'
    csv_file.write_text('x,y\n0,0\n1,0\n1,1\n')
    path = ppc.load_csv_path(str(csv_file))
    assert len(path) == 3
    assert path[0].yaw == pytest.approx(0.0)          # heading toward (1,0)
    assert path[1].yaw == pytest.approx(math.pi / 2)  # heading toward (1,1)


def test_load_csv_path_uses_recorded_yaw(tmp_path):
    csv_file = tmp_path / 'path.csv'
    csv_file.write_text('0,0,1.0\n1,0,2.0\n')
    path = ppc.load_csv_path(str(csv_file))
    assert path[0].yaw == pytest.approx(1.0)
    assert path[1].yaw == pytest.approx(2.0)


def test_load_csv_path_skips_comments_and_blanks(tmp_path):
    csv_file = tmp_path / 'path.csv'
    csv_file.write_text('# comment\n\nx,y\n0,0\n1,0\n')
    path = ppc.load_csv_path(str(csv_file))
    assert len(path) == 2


def test_load_csv_path_rejects_too_short(tmp_path):
    csv_file = tmp_path / 'path.csv'
    csv_file.write_text('x,y\n0,0\n')
    with pytest.raises(ValueError):
        ppc.load_csv_path(str(csv_file))


# --- closest_index: forward-only search --------------------------------

def test_closest_index_tracks_a_pose_walking_along_the_path():
    path = straight_path(n=40, spacing=0.5)
    idx = 0
    for step in range(0, 40, 3):
        pose = (step * 0.5 + 0.05, 0.02)  # near waypoint `step`, slightly off
        new_idx = ppc.closest_index(path, pose, idx)
        assert new_idx >= idx, 'closest_index must never move backward'
        idx = new_idx
    assert idx > 30  # actually made it most of the way down the path


def test_closest_index_does_not_jump_backward_on_self_intersection():
    # A path that crosses back near its own start (a loop) -- pose sits near
    # the crossing but search must stay anchored to start_idx's neighborhood.
    xy = [(i * 0.5, 0.0) for i in range(10)] + [(4.5 - i * 0.5, 0.1) for i in range(10)]
    path = ppc.path_from_xy(xy)
    idx = ppc.closest_index(path, (4.5, 0.0), start_idx=9, window=25)
    assert idx >= 9


# --- find_lookahead_point: precise interpolation ------------------------

def test_lookahead_point_is_exactly_lookahead_distance_away_on_a_straight_line():
    path = straight_path(n=20, spacing=0.5)
    pose = (0.0, 0.0)
    pt, idx, reached_end = ppc.find_lookahead_point(path, pose, 0, lookahead_dist=2.0)
    assert not reached_end
    assert ppc.distance(pose, pt) == pytest.approx(2.0, abs=1e-9)
    assert pt == pytest.approx((2.0, 0.0))


def test_lookahead_point_flags_reached_end_near_the_goal():
    path = straight_path(n=5, spacing=0.5)  # spans 0.0 .. 2.0
    pose = (1.9, 0.0)
    pt, idx, reached_end = ppc.find_lookahead_point(path, pose, 3, lookahead_dist=2.0)
    assert reached_end
    assert pt == pytest.approx((2.0, 0.0))


# --- steering geometry ---------------------------------------------------

def test_steering_is_zero_when_pose_is_on_path_pointing_along_it():
    lookahead_pt = (2.0, 0.0)
    local_x, local_y = ppc.world_to_body(pose=(0.0, 0.0), yaw=0.0, point=lookahead_pt)
    assert local_y == pytest.approx(0.0)
    curvature = ppc.curvature_from_local_y(local_y, lookahead_dist=2.0)
    delta = ppc.steering_angle(curvature, wheelbase=0.937, max_steer_angle=0.35)
    assert delta == pytest.approx(0.0)


def test_steering_turns_toward_a_path_curving_left():
    path = quarter_circle_path(radius=5.0)
    pose, yaw = (0.0, 0.0), 0.0
    lookahead_pt, idx, _ = ppc.find_lookahead_point(path, pose, 0, lookahead_dist=1.5)
    local_x, local_y = ppc.world_to_body(pose, yaw, lookahead_pt)
    assert local_y > 0, 'a left-curving path should land to the left in the body frame'
    curvature = ppc.curvature_from_local_y(local_y, lookahead_dist=1.5)
    delta = ppc.steering_angle(curvature, wheelbase=0.937, max_steer_angle=0.35)
    assert delta > 0, 'steering toward a left curve should be a positive (left) angle'
    steer_cmd = ppc.normalize_steer(delta, max_steer_angle=0.35, sign=1.0)
    assert 0 < steer_cmd <= 1.0


def test_steering_angle_is_clamped_to_max():
    # A huge lateral offset relative to L drives curvature far past what the
    # bicycle model's max steer angle can express.
    curvature = ppc.curvature_from_local_y(local_y=5.0, lookahead_dist=1.0)
    delta = ppc.steering_angle(curvature, wheelbase=0.937, max_steer_angle=0.35)
    assert delta == pytest.approx(0.35)
    steer_cmd = ppc.normalize_steer(delta, max_steer_angle=0.35)
    assert steer_cmd == pytest.approx(1.0)


def test_normalize_steer_sign_flip_for_calibration():
    delta = 0.1
    assert ppc.normalize_steer(delta, max_steer_angle=0.35, sign=1.0) > 0
    assert ppc.normalize_steer(delta, max_steer_angle=0.35, sign=-1.0) < 0


# --- speed shaping ---------------------------------------------------------

def test_scaled_lookahead_is_fixed_when_gain_is_zero():
    assert ppc.scaled_lookahead(speed=3.0, base=1.0, gain=0.0, min_ld=0.5, max_ld=3.0) == 1.0


def test_scaled_lookahead_clamps_to_bounds():
    assert ppc.scaled_lookahead(speed=10.0, base=1.0, gain=1.0, min_ld=0.5, max_ld=3.0) == 3.0
    assert ppc.scaled_lookahead(speed=0.0, base=0.2, gain=1.0, min_ld=0.5, max_ld=3.0) == 0.5


def test_curvature_speed_constant_when_gain_is_zero():
    assert ppc.curvature_speed(target_speed=2.0, curvature=5.0, gain=0.0, min_speed=0.3) == 2.0


def test_curvature_speed_reduces_on_tight_curves():
    slowed = ppc.curvature_speed(target_speed=2.0, curvature=5.0, gain=1.0, min_speed=0.3)
    assert 0.3 <= slowed < 2.0


# --- goal detection ---------------------------------------------------------

def test_goal_detection_distance():
    path = straight_path(n=10, spacing=0.5)  # last point at (4.5, 0.0)
    assert ppc.distance((4.4, 0.0), path[-1].xy) < 0.3
    assert ppc.distance((3.0, 0.0), path[-1].xy) > 0.3


# --- bicycle model (used by bicycle_sim_node) -------------------------------

def test_bicycle_model_drives_straight_with_zero_steer():
    x, y, yaw = 0.0, 0.0, 0.0
    for _ in range(10):
        x, y, yaw, speed = ppc.step_bicycle_model(
            x, y, yaw, throttle=1.0, steer=0.0, dt=0.1,
            wheelbase=0.937, max_speed_mps=2.0, max_steer_angle=0.35)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert x == pytest.approx(2.0, abs=1e-9)  # 2.0 m/s * 1.0 s


def test_bicycle_model_turns_left_with_positive_steer():
    x, y, yaw = 0.0, 0.0, 0.0
    for _ in range(20):
        x, y, yaw, speed = ppc.step_bicycle_model(
            x, y, yaw, throttle=1.0, steer=1.0, dt=0.1,
            wheelbase=0.937, max_speed_mps=1.0, max_steer_angle=0.35)
    assert yaw > 0, 'positive steer should yaw the vehicle left (CCW)'
    assert y > 0
