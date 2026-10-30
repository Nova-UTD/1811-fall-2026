#!/usr/bin/env python3
"""Display the colored cloud produced by the Jetson; never detect edges locally."""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

EDGE_CLOUD_TOPIC = "/lidar_perception/edge_cloud"


def unpack_colored_cloud(msg):
    """Read the Jetson's XYZRGB PointCloud2 without interpreting raw lidar."""
    fields = {field.name: field for field in msg.fields}
    expected = {"x": 0, "y": 4, "z": 8, "rgb": 12}
    if (msg.height != 1 or msg.width == 0 or msg.is_bigendian or msg.point_step != 16
            or msg.row_step != msg.width * 16 or len(msg.data) != msg.row_step
            or any(name not in fields or fields[name].offset != offset
                   or fields[name].datatype != PointField.FLOAT32 or fields[name].count != 1
                   for name, offset in expected.items())):
        raise ValueError("unexpected Jetson XYZRGB cloud layout")

    # An idle viewer does not load NumPy or Open3D before the first remote cloud.
    import numpy as np
    packed = np.frombuffer(msg.data, dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4"),
    ])
    xyz = np.column_stack((packed["x"], packed["y"], packed["z"]))
    rgb = packed["rgb"]
    colors = np.column_stack(((rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255)) / 255.0
    return xyz, colors


class EdgeVisualizer:
    def __init__(self):
        import numpy as np
        import open3d as o3d

        self.o3d = o3d
        self.vis = o3d.visualization.Visualizer()
        if not self.vis.create_window(window_name="Lidar edge points", width=1280, height=720):
            raise RuntimeError("Could not open the Open3D window; check DISPLAY and X11 access")
        options = self.vis.get_render_option()
        options.background_color = np.array([0.05, 0.05, 0.05])
        options.point_size = 2.0
        options.light_on = False
        self.pcd = o3d.geometry.PointCloud()
        self.added = False

    def show_cloud(self, points, colors):
        self.pcd.points = self.o3d.utility.Vector3dVector(points)
        self.pcd.colors = self.o3d.utility.Vector3dVector(colors)
        if not self.added:
            self.vis.add_geometry(self.pcd, reset_bounding_box=True)
            self.added = True
        else:
            self.vis.update_geometry(self.pcd)
        self.vis.update_renderer()

    def poll_events(self):
        return self.vis.poll_events()

    def close(self):
        self.vis.destroy_window()


class CornerViewer(Node):
    def __init__(self):
        super().__init__("corner_viewer")
        self.visualizer = None
        self.pending_cloud = None
        self.last_frame = None
        self.retry_at = 0.0
        latest_cloud = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            PointCloud2, EDGE_CLOUD_TOPIC, self.cloud_callback, latest_cloud
        )
        self.get_logger().info("Waiting for a processed edge cloud from the Jetson")

    def cloud_callback(self, msg):
        self.pending_cloud = msg

    def update(self):
        now = time.monotonic()
        if (self.visualizer is not None and now - self.last_frame > 30.0
                and self.count_publishers(EDGE_CLOUD_TOPIC) == 0):
            self.get_logger().warning("Jetson edge publisher is gone; closing the viewer")
            return False

        if self.pending_cloud is not None and now >= self.retry_at:
            msg = self.pending_cloud
            self.pending_cloud = None
            try:
                points, colors = unpack_colored_cloud(msg)
                if self.visualizer is None:
                    self.visualizer = EdgeVisualizer()
                self.visualizer.show_cloud(points, colors)
                self.last_frame = time.monotonic()
            except Exception as exc:
                if self.visualizer is not None:
                    self.visualizer.close()
                    self.visualizer = None
                self.get_logger().error(f"Could not display Jetson edge cloud: {exc}")
                self.retry_at = now + 10.0
        return self.visualizer is None or self.visualizer.poll_events()

    def close(self):
        if self.visualizer is not None:
            self.visualizer.close()
            self.visualizer = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CornerViewer()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.update():
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
