"""
Unit tests for camera_panorama.panorama_core -- no ROS required.

The cameras are the four 1811 ZED X mounts from vehicle_1811.urdf.xacro
(positions, yaws, measured down-tilts) with a ZED-X-like lens. A synthetic
scene painted on a cylinder around the car is rendered into each camera, then
stitched back; the panorama should reproduce the scene.

Run with `pytest test/test_panorama_core.py`, or as part of
`colcon test --packages-select camera_panorama`.
"""
import math

import numpy as np
import pytest

from camera_panorama import panorama_core as pc

# (x, y, z, pitch, yaw) from vehicle_1811.urdf.xacro -- pitch + = nose down.
URDF_MOUNTS = {
    'front': (0.215, -0.105, 1.045, 0.087, 0.0),
    'left': (0.030, 0.020, 1.045, 0.367, math.pi / 2),
    'rear': (-0.130, -0.105, 1.045, 0.244, math.pi),
    'right': (0.030, -0.300, 1.045, 0.244, -math.pi / 2),
}
# Half-resolution ZED X (960x600 published) scaled down again for test speed:
# ~105 deg horizontal field of view, like the real cameras.
IMG_W, IMG_H = 480, 300
FX = 182.5
K = np.array([[FX, 0.0, (IMG_W - 1) / 2], [0.0, FX, (IMG_H - 1) / 2], [0.0, 0.0, 1.0]])


def car_cameras():
    cams = []
    for name, (x, y, z, pitch, yaw) in URDF_MOUNTS.items():
        R_body = pc.rpy_matrix(0.0, pitch, yaw)
        cams.append(pc.Camera(name, K, IMG_W, IMG_H, R_body @ pc.BODY_FROM_OPTICAL,
                              np.array([x, y, z])))
    return cams


def scene_color(azimuth, height):
    """Smooth, direction-dependent colours painted on the scene cylinder."""
    r = 128 + 100 * np.sin(6 * azimuth)
    g = 128 + 100 * np.cos(9 * azimuth + 3 * height)
    b = 128 + 100 * np.sin(12 * height + 2 * azimuth)
    return np.stack([r, g, b], axis=-1)


def render(cam, center, scene_distance):
    """Ray-cast one camera image of the scene cylinder (radius scene_distance)."""
    u, v = np.meshgrid(np.arange(cam.width), np.arange(cam.height))
    rays = np.stack([(u - cam.K[0, 2]) / cam.K[0, 0], (v - cam.K[1, 2]) / cam.K[1, 1],
                     np.ones_like(u, dtype=float)], axis=-1) @ cam.R.T
    q = cam.t[:2] - center[:2]
    dxy = rays[..., :2]
    a = (dxy ** 2).sum(-1)
    b = 2 * (dxy @ q)
    c = q @ q - scene_distance ** 2
    s = (-b + np.sqrt(np.maximum(b * b - 4 * a * c, 0))) / np.maximum(2 * a, 1e-12)
    p = cam.t + s[..., None] * rays
    az = np.arctan2(p[..., 1] - center[1], p[..., 0] - center[0])
    h = (p[..., 2] - center[2]) / scene_distance
    return np.clip(scene_color(az, h), 0, 255).astype(np.uint8)


def expected_panorama(spec, center, scene_distance):
    """Return what the panorama should look like for a scene at the stitch distance."""
    pts = pc.cylinder_points(spec, center)
    az = np.arctan2(pts[..., 1] - center[1], pts[..., 0] - center[0])
    h = (pts[..., 2] - center[2]) / scene_distance
    return np.clip(scene_color(az, h), 0, 255)


SPEC = pc.PanoramaSpec(width=1024, stitch_distance=8.0)


@pytest.fixture(scope='module')
def rig():
    cams = car_cameras()
    stitcher = pc.Stitcher(cams, SPEC, use_cv2=False)
    center = pc.panorama_center(cams, SPEC)
    images = {c.name: render(c, center, SPEC.stitch_distance) for c in cams}
    return cams, stitcher, center, images


def mean_error(pano, expected, mask):
    return float(np.abs(pano.astype(float) - expected)[mask].mean())


# --- geometry ---------------------------------------------------------------

def test_panorama_height_matches_elevation_range():
    spec = pc.PanoramaSpec(width=2048, elev_min_deg=-35, elev_max_deg=15)
    f = 2048 / (2 * math.pi)
    assert pc.panorama_height(spec) == round(f * (math.tan(math.radians(15))
                                                  + math.tan(math.radians(35))))


def test_middle_column_looks_forward_and_left_is_left():
    azimuth, _ = pc.pixel_angles(pc.PanoramaSpec(width=1000))
    assert abs(azimuth[500]) < 0.01                 # middle = forward
    assert azimuth[250] == pytest.approx(math.pi / 2, abs=0.01)   # left quarter = +y
    assert azimuth[750] == pytest.approx(-math.pi / 2, abs=0.01)  # right quarter = -y


def test_point_straight_ahead_lands_near_front_camera_centre():
    front = car_cameras()[0]
    level = front.t + np.array([20.0, 0.0, 0.0])
    px, py, ok = pc.project(front, level[None, :])
    assert ok[0]
    assert px[0] == pytest.approx(front.K[0, 2], abs=1.0)
    # The front camera is tilted 5 deg nose-down, so the horizon sits above centre.
    assert py[0] < front.K[1, 2]


def test_quaternion_matches_rpy():
    yaw = 0.7
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert np.allclose(pc.quat_to_matrix(*q), pc.rpy_matrix(0, 0, yaw))


def test_zero_correction_is_identity_and_yaw_pans_the_camera():
    front = car_cameras()[0]
    assert np.allclose(pc.correct_rotation(front.R, 0, 0, 0), front.R)
    target = (front.t + np.array([20.0, 0.0, 0.0]))[None, :]
    panned = pc.Camera('f', front.K, front.width, front.height,
                       pc.correct_rotation(front.R, 0, 0, 5.0), front.t)
    px0 = pc.project(front, target)[0][0]
    px1 = pc.project(panned, target)[0][0]
    assert px1 > px0          # camera panned left -> a fixed point moves right in the image


# --- coverage ---------------------------------------------------------------

def test_the_four_cameras_cover_the_whole_horizon(rig):
    _, stitcher, _, _ = rig
    _, height = pc.pixel_angles(SPEC)
    horizon_row = int(np.argmin(np.abs(height)))
    assert stitcher.coverage[horizon_row].all()


def test_corners_overlap_so_seams_can_blend(rig):
    _, stitcher, _, _ = rig
    _, height = pc.pixel_angles(SPEC)
    horizon = stitcher.overlap[int(np.argmin(np.abs(height)))]
    # four corner overlaps, each a run of columns
    starts = np.flatnonzero(np.diff(horizon.astype(int)) == 1)
    assert len(starts) + int(horizon[0]) >= 4


def test_default_band_is_almost_fully_covered(rig):
    _, stitcher, _, _ = rig
    assert stitcher.coverage_fraction() > 0.97


# --- stitching ----------------------------------------------------------------

def test_stitch_reconstructs_scene_at_stitch_distance(rig):
    _, stitcher, center, images = rig
    pano = stitcher.stitch(images)
    assert pano.shape == (stitcher.height, SPEC.width, 3)
    expected = expected_panorama(SPEC, center, SPEC.stitch_distance)
    assert mean_error(pano, expected, stitcher.coverage) < 3.0


def test_seams_line_up_at_stitch_distance(rig):
    _, stitcher, center, images = rig
    pano = stitcher.stitch(images)
    expected = expected_panorama(SPEC, center, SPEC.stitch_distance)
    assert mean_error(pano, expected, stitcher.overlap) < 3.0


def test_closer_objects_show_parallax_at_seams(rig):
    cams, stitcher, center, _ = rig
    near = {c.name: render(c, center, 1.5) for c in cams}
    pano = stitcher.stitch(near)
    expected = expected_panorama(pc.PanoramaSpec(width=SPEC.width, stitch_distance=1.5),
                                 center, 1.5)
    err_seam = mean_error(pano, expected, stitcher.overlap)
    assert err_seam > 3.0     # documented limitation: not a bug


def test_missing_camera_leaves_black_without_breaking_others(rig):
    _, stitcher, center, images = rig
    partial = dict(images, rear=None)
    pano = stitcher.stitch(partial)
    W = SPEC.width
    _, height = pc.pixel_angles(SPEC)
    row = int(np.argmin(np.abs(height)))
    assert pano[row, 0].sum() == 0 and pano[row, W - 1].sum() == 0  # straight back is black
    expected = expected_panorama(SPEC, center, SPEC.stitch_distance)
    front_band = np.zeros_like(stitcher.coverage)
    front_band[:, 3 * W // 8: 5 * W // 8] = True
    assert mean_error(pano, expected, front_band & stitcher.coverage) < 3.0


# --- exposure -----------------------------------------------------------------

def test_gains_undo_a_darker_camera(rig):
    _, stitcher, center, images = rig
    dark = dict(images, left=(images['left'] * 0.6).astype(np.uint8))
    gains = pc.estimate_gains(stitcher, dark)
    assert gains['left'] / gains['front'] == pytest.approx(1 / 0.6, rel=0.1)
    expected = expected_panorama(SPEC, center, SPEC.stitch_distance)
    err_before = mean_error(stitcher.stitch(dark), expected, stitcher.coverage)
    err_after = mean_error(stitcher.stitch(dark, gains), expected, stitcher.coverage)
    assert err_after < err_before / 2


def test_equal_exposure_gives_unit_gains(rig):
    _, stitcher, _, images = rig
    gains = pc.estimate_gains(stitcher, images)
    assert all(g == pytest.approx(1.0, abs=0.03) for g in gains.values())


# --- OpenCV path ----------------------------------------------------------------

@pytest.mark.skipif(pc.cv2 is None, reason='OpenCV not installed')
def test_cv2_and_numpy_paths_agree(rig):
    cams, stitcher, _, images = rig
    fast = pc.Stitcher(cams, SPEC, use_cv2=True)
    a = stitcher.stitch(images).astype(int)
    b = fast.stitch(images).astype(int)
    assert np.abs(a - b)[stitcher.coverage].mean() < 1.0
