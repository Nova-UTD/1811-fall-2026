"""
panorama_node -- stitches the four ZED X cameras into one 360-degree image.

Runs on the Jetson, next to the cameras (jetson_bringup zed_cameras.launch.py),
so only the finished panorama crosses the cable to the Karbon.

Subscribes (per camera in `cameras`):
  /zed_<camera>/zed_node/rgb/color/rect/image        sensor_msgs/Image
  /zed_<camera>/zed_node/rgb/color/rect/camera_info  sensor_msgs/CameraInfo
Uses TF for each image's frame relative to `base_frame` -- the URDF on the
Karbon places zed_<camera>_camera_link, the ZED driver adds the lens frames.
Publishes:
  /panorama/image  sensor_msgs/Image (bgr8), best effort, at `rate` Hz

The warp is built once, when every camera has sent its camera_info and its
TF is known. After that each frame is a lookup and blend (panorama_core).
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from camera_panorama import panorama_core as pc

CHANNELS = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3, 'mono8': 1}


def image_to_bgr(msg: Image) -> np.ndarray:
    """Return a ROS image as an H x W x 3 BGR uint8 array (no cv_bridge needed)."""
    ch = CHANNELS.get(msg.encoding)
    if ch is None:
        raise ValueError(f'unsupported encoding {msg.encoding!r}')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    if msg.encoding == 'mono8':
        return np.repeat(img, 3, axis=2)
    if msg.encoding.startswith('rgb'):
        img = img[..., 2::-1]
    return np.ascontiguousarray(img[..., :3])


class PanoramaNode(Node):

    def __init__(self):
        super().__init__('panorama_node')
        self.declare_parameter('cameras', ['front', 'left', 'rear', 'right'])
        self.declare_parameter('image_topic', '/zed_{camera}/zed_node/rgb/color/rect/image')
        self.declare_parameter('info_topic', '/zed_{camera}/zed_node/rgb/color/rect/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_topic', '/panorama/image')
        self.declare_parameter('width', 2048)
        self.declare_parameter('elev_min_deg', -35.0)
        self.declare_parameter('elev_max_deg', 15.0)
        self.declare_parameter('stitch_distance', 8.0)
        self.declare_parameter('feather', 0.15)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('max_age', 0.5)
        self.declare_parameter('startup_timeout', 15.0)
        self.declare_parameter('use_opencv', True)
        self.declare_parameter('exposure_compensation', True)
        self.declare_parameter('gain_interval', 1.0)
        self.declare_parameter('gain_smoothing', 0.3)

        p = self._param
        self._names = list(p('cameras'))
        for name in self._names:
            # Small mounting fixes on top of the URDF, in the camera's body frame:
            # [roll, pitch (+ = nose down), yaw (+ = pan left)], degrees.
            self.declare_parameter(f'{name}_correction_rpy_deg', [0.0, 0.0, 0.0])
        self._base = p('base_frame')
        self._spec = pc.PanoramaSpec(
            width=int(p('width')), elev_min_deg=float(p('elev_min_deg')),
            elev_max_deg=float(p('elev_max_deg')), stitch_distance=float(p('stitch_distance')),
            feather=float(p('feather')))
        self._max_age = float(p('max_age'))
        self._startup_timeout = float(p('startup_timeout'))
        self._use_cv2 = bool(p('use_opencv'))
        self._exposure = bool(p('exposure_compensation'))
        self._gain_interval = float(p('gain_interval'))
        self._gain_alpha = float(p('gain_smoothing'))

        self._info = {}       # name -> CameraInfo
        self._latest = {}     # name -> (Image msg, receive time)
        self._stitcher = None
        self._built_sizes = {}  # name -> (width, height) the warp was built for
        self._gains = {n: 1.0 for n in self._names}
        self._last_gain_update = 0.0
        self._started = time.monotonic()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        for name in self._names:
            self.create_subscription(
                Image, p('image_topic').format(camera=name),
                lambda m, n=name: self._latest.__setitem__(n, (m, time.monotonic())),
                qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, p('info_topic').format(camera=name),
                lambda m, n=name: self._info.__setitem__(n, m),
                qos_profile_sensor_data)
        self._pub = self.create_publisher(Image, p('output_topic'), qos_profile_sensor_data)
        self.create_timer(1.0 / float(p('rate')), self._tick)
        self.get_logger().info(
            f'Waiting for {", ".join(self._names)}: images, camera_info and TF to '
            f'{self._base}. Output: {p("output_topic")}')

    # --- setup ------------------------------------------------------------------

    def _param(self, name):
        return self.get_parameter(name).value

    def _camera(self, name):
        """Build a pc.Camera from camera_info + TF, or return a reason it can't yet."""
        info, latest = self._info.get(name), self._latest.get(name)
        if info is None or latest is None:
            return None, 'no camera_info/image yet'
        frame = latest[0].header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(self._base, frame, Time())
        except tf2_ros.TransformException:
            return None, f'no TF {self._base} -> {frame} (is bringup running on the Karbon?)'
        q, tr = tf.transform.rotation, tf.transform.translation
        R = pc.quat_to_matrix(q.x, q.y, q.z, q.w)
        roll, pitch, yaw = self.get_parameter(f'{name}_correction_rpy_deg').value
        R = pc.correct_rotation(R, roll, pitch, yaw)
        K = np.array(info.k, dtype=float).reshape(3, 3)
        return pc.Camera(name, K, int(info.width), int(info.height), R,
                         np.array([tr.x, tr.y, tr.z])), None

    def _try_build(self):
        ready, waiting = [], {}
        for name in self._names:
            cam, reason = self._camera(name)
            if cam is None:
                waiting[name] = reason
            else:
                ready.append(cam)
        timed_out = time.monotonic() - self._started > self._startup_timeout
        if waiting and not (timed_out and ready):
            self.get_logger().info(
                'Panorama waiting: ' + '; '.join(f'{n}: {r}' for n, r in waiting.items()),
                throttle_duration_sec=5.0)
            return
        if waiting:
            self.get_logger().warning(
                f'Starting without {", ".join(waiting)} after {self._startup_timeout:.0f} s '
                '-- their part of the panorama stays black.')
        self._stitcher = pc.Stitcher(ready, self._spec, use_cv2=self._use_cv2)
        self._built_sizes = {c.name: (c.width, c.height) for c in ready}
        self.get_logger().info(
            f'Panorama ready: {self._stitcher.width}x{self._stitcher.height} from '
            f'{", ".join(c.name for c in ready)}, coverage '
            f'{100 * self._stitcher.coverage_fraction():.0f}%, '
            f'{"OpenCV" if self._stitcher.use_cv2 else "numpy"} warping')

    # --- per frame ----------------------------------------------------------------

    def _tick(self):
        if self._stitcher is None:
            self._try_build()
            return
        now = time.monotonic()
        images, newest = {}, None
        for name, size in self._built_sizes.items():
            entry = self._latest.get(name)
            if entry is None or now - entry[1] > self._max_age:
                continue
            msg = entry[0]
            if (msg.width, msg.height) != size:
                self.get_logger().warning(
                    f'{name} image size changed to {msg.width}x{msg.height}; rebuilding.')
                self._stitcher = None
                return
            try:
                images[name] = image_to_bgr(msg)
            except ValueError as exc:
                self.get_logger().error(f'{name}: {exc}', throttle_duration_sec=5.0)
                continue
            if newest is None or Time.from_msg(msg.header.stamp) > Time.from_msg(newest):
                newest = msg.header.stamp
        if not images:
            self.get_logger().warning('No recent camera images.', throttle_duration_sec=5.0)
            return

        if self._exposure and now - self._last_gain_update >= self._gain_interval:
            self._last_gain_update = now
            target = pc.estimate_gains(self._stitcher, images)
            a = self._gain_alpha
            for n, g in target.items():   # smooth in log space so exposure changes ease in
                self._gains[n] = math.exp((1 - a) * math.log(self._gains.get(n, 1.0))
                                          + a * math.log(g))

        pano = self._stitcher.stitch(images, self._gains if self._exposure else None)
        out = Image()
        out.header.stamp = newest
        out.header.frame_id = self._base
        out.height, out.width = pano.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = pano.tobytes()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = PanoramaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
