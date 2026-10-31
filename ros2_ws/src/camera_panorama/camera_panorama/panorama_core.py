"""
Cylindrical 360-degree panorama from fixed cameras -- no ROS imports.

The cameras are bolted to the car, so the mapping from every panorama pixel
to the camera pixel(s) it comes from never changes. Stitcher works it out once
(from each camera's intrinsics and pose) and then each frame is just a lookup
and a weighted blend. panorama_node.py is the thin rclpy wrapper.

Geometry, in the vehicle's base frame (x forward, y left, z up):
  - The panorama is a cylinder of radius `stitch_distance` around `center`
    (default: the middle of the cameras). Column 0 looks straight back,
    the middle column looks forward (+x), left is left of centre.
  - Rows run from `elev_max_deg` (top) to `elev_min_deg` (bottom) above the
    cylinder's horizontal plane.
  - Each panorama pixel is a point on that cylinder; it is projected into every
    camera that sees it, and overlapping cameras are feather-blended.
Objects at `stitch_distance` line up exactly across seams. Closer ones show
parallax (doubling at the seams) because the cameras sit in different places --
inherent to any multi-camera surround view.

Cameras are described in their OPTICAL frame (z out of the lens, x right,
y down) -- the frame_id the ZED images carry.
"""
from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

try:  # OpenCV is much faster at warping; numpy alone works too (and is what tests use).
    import cv2
except Exception:  # pragma: no cover - depends on the machine
    # Not just ImportError: an OpenCV built for NumPy 1.x under a pip-installed
    # NumPy 2 fails with AttributeError (_ARRAY_API not found). Fall back to numpy.
    cv2 = None

# Camera body frame (x fwd, y left, z up) -> optical frame (z fwd, x right, y down).
BODY_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0]])


@dataclass
class Camera:
    name: str
    K: np.ndarray        # 3x3 intrinsics of the (rectified) image
    width: int
    height: int
    R: np.ndarray        # 3x3 rotation: optical frame -> base frame
    t: np.ndarray        # optical-frame origin in the base frame (metres)


@dataclass
class PanoramaSpec:
    width: int = 2048                 # pixels for the full 360 degrees
    elev_min_deg: float = -35.0       # bottom row, degrees above the horizon
    elev_max_deg: float = 15.0        # top row
    stitch_distance: float = 8.0      # metres; objects here line up across seams
    feather: float = 0.15             # blend ramp, as a fraction of the image's short side
    center: Optional[Tuple[float, float, float]] = None  # base frame; None = mean camera position


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the URDF/REP-103 fixed-axis roll-pitch-yaw rotation (Rz @ Ry @ Rx)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the rotation matrix of a unit quaternion (e.g. a TF transform's rotation)."""
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def correct_rotation(R: np.ndarray, roll_deg: float, pitch_deg: float,
                     yaw_deg: float) -> np.ndarray:
    """
    Apply a small mounting correction to an optical-frame rotation.

    The correction is in the camera's own body frame (x forward, y left, z up),
    matching the URDF: yaw pans the camera, + pitch tilts it nose-down, roll
    turns it about its lens axis. Used to refine tape-measured mounts.
    """
    Rc = rpy_matrix(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    return R @ BODY_FROM_OPTICAL.T @ Rc @ BODY_FROM_OPTICAL


def panorama_height(spec: PanoramaSpec) -> int:
    f = spec.width / (2.0 * math.pi)
    span = math.tan(math.radians(spec.elev_max_deg)) - math.tan(math.radians(spec.elev_min_deg))
    return max(1, int(round(f * span)))


def panorama_center(cameras: List[Camera], spec: PanoramaSpec) -> np.ndarray:
    if spec.center is not None:
        return np.asarray(spec.center, dtype=float)
    return np.mean([c.t for c in cameras], axis=0)


def pixel_angles(spec: PanoramaSpec) -> Tuple[np.ndarray, np.ndarray]:
    """Return (azimuth, height) per panorama column/row; azimuth 0 = forward, + = left."""
    W, H = spec.width, panorama_height(spec)
    f = W / (2.0 * math.pi)
    azimuth = (0.5 - (np.arange(W) + 0.5) / W) * 2.0 * math.pi
    height = math.tan(math.radians(spec.elev_max_deg)) - (np.arange(H) + 0.5) / f
    return azimuth, height


def cylinder_points(spec: PanoramaSpec, center: np.ndarray) -> np.ndarray:
    """Return the base-frame point (H x W x 3) each panorama pixel represents."""
    azimuth, height = pixel_angles(spec)
    D = spec.stitch_distance
    pts = np.empty((len(height), len(azimuth), 3))
    pts[..., 0] = center[0] + D * np.cos(azimuth)[None, :]
    pts[..., 1] = center[1] + D * np.sin(azimuth)[None, :]
    pts[..., 2] = center[2] + D * height[:, None]
    return pts


def project(cam: Camera, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project base-frame points into a camera; return (px, py, in_front)."""
    p = (pts - cam.t) @ cam.R  # == R^T (pts - t), row-vector form
    z = p[..., 2]
    in_front = z > 1e-3
    zs = np.where(in_front, z, 1.0)
    px = cam.K[0, 0] * p[..., 0] / zs + cam.K[0, 2]
    py = cam.K[1, 1] * p[..., 1] / zs + cam.K[1, 2]
    return px, py, in_front


def edge_weight(px: np.ndarray, py: np.ndarray, cam: Camera, feather: float) -> np.ndarray:
    """Return a 0..1 weight: 0 at the image border, 1 once `feather` inside it (smoothstep)."""
    ramp = max(1.0, feather * min(cam.width, cam.height))
    d = np.minimum.reduce([px, cam.width - 1 - px, py, cam.height - 1 - py])
    e = np.clip(d / ramp, 0.0, 1.0)
    return e * e * (3.0 - 2.0 * e)


@dataclass
class _CamMap:
    name: str
    pix: np.ndarray       # flat panorama indices this camera contributes to
    px: np.ndarray        # source x for each of those pixels (float32)
    py: np.ndarray        # source y (float32)
    w: np.ndarray         # normalized blend weight
    map_x: Optional[np.ndarray] = None  # dense H x W maps for cv2.remap
    map_y: Optional[np.ndarray] = None
    w_dense: Optional[np.ndarray] = None  # dense H x W x 1 weights for the cv2 path


class Stitcher:
    """Precomputed warp + blend of fixed cameras into one cylindrical panorama."""

    def __init__(self, cameras: List[Camera], spec: PanoramaSpec, use_cv2: bool = True):
        self.spec = spec
        self.cameras = cameras
        self.height = panorama_height(spec)
        self.width = spec.width
        self.use_cv2 = bool(use_cv2 and cv2 is not None)
        pts = cylinder_points(spec, panorama_center(cameras, spec))

        raw = {}
        for cam in cameras:
            px, py, front = project(cam, pts)
            inside = front & (px >= 0) & (px <= cam.width - 1) & (py >= 0) & (py <= cam.height - 1)
            w = np.where(inside, edge_weight(px, py, cam, spec.feather), 0.0)
            raw[cam.name] = (px, py, inside, w)

        total = sum(r[3] for r in raw.values())
        inside_count = sum(r[2].astype(int) for r in raw.values())
        self.coverage = inside_count > 0
        self.overlap = inside_count > 1

        self._maps: List[_CamMap] = []
        for cam in cameras:
            px, py, inside, w = raw[cam.name]
            # Normalize so overlapping cameras' weights sum to 1. Where the summed
            # feather weight is ~0 (a pixel only seen right at an image border),
            # take the camera at full weight instead of leaving a dark rim.
            wn = np.where(total > 1e-9, w / np.maximum(total, 1e-9), inside.astype(float))
            wn = np.where(inside, wn, 0.0)
            pix = np.flatnonzero(wn > 0)
            m = _CamMap(cam.name, pix,
                        px.ravel()[pix].astype(np.float32), py.ravel()[pix].astype(np.float32),
                        wn.ravel()[pix].astype(np.float32))
            if self.use_cv2:
                m.map_x = np.where(inside, px, -1).astype(np.float32)
                m.map_y = np.where(inside, py, -1).astype(np.float32)
                m.w_dense = wn.astype(np.float32)[..., None]
            self._maps.append(m)

    def coverage_fraction(self) -> float:
        return float(self.coverage.mean())

    def warp(self, name: str, image: np.ndarray) -> np.ndarray:
        """Return one camera's contribution, already weighted (H x W x C float32)."""
        m = next(mm for mm in self._maps if mm.name == name)
        H, W = self.height, self.width
        if self.use_cv2:
            warped = cv2.remap(image, m.map_x, m.map_y, cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT)
            if warped.ndim == 2:
                warped = warped[..., None]
            return warped.astype(np.float32) * m.w_dense
        out = np.zeros((H * W, image.shape[2]), np.float32)
        out[m.pix] = bilinear(image, m.px, m.py) * m.w[:, None]
        return out.reshape(H, W, image.shape[2])

    def stitch(self, images: Dict[str, Optional[np.ndarray]],
               gains: Optional[Dict[str, float]] = None) -> np.ndarray:
        """
        Blend the cameras' latest images into one panorama (uint8).

        Missing cameras (None or absent) leave their area black; their overlap
        with neighbours still shows the neighbour, at reduced brightness.
        """
        channels = next((im.shape[2] for im in images.values() if im is not None), 3)
        acc = np.zeros((self.height, self.width, channels), np.float32)
        for m in self._maps:
            img = images.get(m.name)
            if img is None:
                continue
            contrib = self.warp(m.name, img)
            g = (gains or {}).get(m.name, 1.0)
            acc += contrib * g if g != 1.0 else contrib
        return np.clip(acc, 0, 255).astype(np.uint8)


def bilinear(image: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Sample an H x W x C image at float coordinates (numpy fallback for cv2.remap)."""
    h, w = image.shape[:2]
    x0 = np.clip(np.floor(x).astype(np.int64), 0, w - 1)
    y0 = np.clip(np.floor(y).astype(np.int64), 0, h - 1)
    x1 = np.minimum(x0 + 1, w - 1)
    y1 = np.minimum(y0 + 1, h - 1)
    fx = (x - x0)[:, None].astype(np.float32)
    fy = (y - y0)[:, None].astype(np.float32)
    img = image.astype(np.float32)
    top = img[y0, x0] * (1 - fx) + img[y0, x1] * fx
    bot = img[y1, x0] * (1 - fx) + img[y1, x1] * fx
    return top * (1 - fy) + bot * fy


def estimate_gains(stitcher: Stitcher, images: Dict[str, Optional[np.ndarray]],
                   step: int = 4, regularize: float = 0.1,
                   limits: Tuple[float, float] = (0.5, 2.0)) -> Dict[str, float]:
    """
    Per-camera brightness gains that make overlapping regions match.

    Each camera sets its own exposure, so seams show as brightness steps. For
    every pair of cameras that overlap, compare their mean brightness over the
    shared pixels (sampled every `step` pixels), then solve for log-gains in the
    least-squares sense, pulled gently toward 1.0 by `regularize`. The median
    camera is anchored at gain 1.0, so the majority keeps its exposure.
    """
    names = [m.name for m in stitcher._maps if images.get(m.name) is not None]
    if len(names) < 2:
        return {n: 1.0 for n in names}
    H, W = stitcher.height, stitcher.width
    lum = {}
    for m in stitcher._maps:
        if m.name not in names:
            continue
        row, col = np.divmod(m.pix, W)
        keep = (row % step == 0) & (col % step == 0)
        img = images[m.name]
        gray = (img.mean(axis=2) if img.ndim == 3 else img).astype(np.float32)[..., None]
        grid = np.full((H, W), np.nan, np.float32)  # NaN = this camera doesn't see it
        grid[row[keep], col[keep]] = bilinear(gray, m.px[keep], m.py[keep])[:, 0]
        lum[m.name] = grid

    idx = {n: i for i, n in enumerate(names)}
    rows, rhs = [], []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            both = ~np.isnan(lum[a]) & ~np.isnan(lum[b])
            both &= np.nan_to_num(lum[a]) > 2
            both &= np.nan_to_num(lum[b]) > 2
            if both.sum() < 50:
                continue
            ma, mb = lum[a][both].mean(), lum[b][both].mean()
            row = np.zeros(len(names))
            row[idx[a]], row[idx[b]] = 1.0, -1.0       # log ga - log gb = log mb - log ma
            rows.append(row)
            rhs.append(math.log(mb) - math.log(ma))
    for n in names:                                    # keep gains near 1
        row = np.zeros(len(names))
        row[idx[n]] = regularize
        rows.append(row)
        rhs.append(0.0)
    sol, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
    # Gains only fix brightness *between* cameras; anchor the median camera at
    # 1.0 so one odd camera gets corrected instead of all the others shifting.
    sol -= np.median(sol)
    lo, hi = limits
    return {n: float(np.clip(math.exp(sol[idx[n]]), lo, hi)) for n in names}
