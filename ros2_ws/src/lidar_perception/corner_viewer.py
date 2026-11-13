#!/usr/bin/env python3

# Display the filtered cloud produced by the Jetson.
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

FILTERED_CLOUD_TOPIC = "/lidar_perception/filtered_points"


def unpack_filtered_cloud(msg):
    # Read the Jetson's XYZ PointCloud2 without interpreting raw lidar.
    fields = {field.name: field for field in msg.fields}
    expected = {"x": 0, "y": 4, "z": 8}
    if (msg.height != 1 or msg.is_bigendian or msg.point_step != 12
            or msg.row_step != msg.width * 12 or len(msg.data) != msg.row_step
            or any(name not in fields or fields[name].offset != offset
                   or fields[name].datatype != PointField.FLOAT32 or fields[name].count != 1
                   for name, offset in expected.items())):
        raise ValueError("unexpected Jetson XYZ cloud layout")

    # An idle viewer does not load NumPy or Open3D before the first remote cloud.
    import numpy as np
    return np.frombuffer(msg.data, dtype="<f4").reshape(-1, 3)


class CloudVisualizer:
    def __init__(self):
        import numpy as np
        import open3d as o3d

        self.o3d = o3d
        self.vis = o3d.visualization.Visualizer()
        if not self.vis.create_window(window_name="Lidar without planar patches", width=1280, height=720):
            raise RuntimeError("Could not open the Open3D window; check DISPLAY and X11 access")
        options = self.vis.get_render_option()
        options.background_color = np.array([0.05, 0.05, 0.05])
        options.point_size = 2.0
        options.light_on = False
        self.pcd = o3d.geometry.PointCloud()
        self.added = False

    def show_cloud(self, points):
        self.pcd.points = self.o3d.utility.Vector3dVector(points)
        if len(points) == 0:
            if self.added:
                self.vis.remove_geometry(self.pcd, reset_bounding_box=False)
                self.added = False
            self.vis.update_renderer()
            return
        self.pcd.paint_uniform_color([0.9, 0.9, 0.9])
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
            PointCloud2, FILTERED_CLOUD_TOPIC, self.cloud_callback, latest_cloud
        )
        self.get_logger().info("Corner viewer started — waiting for a filtered point cloud from the Jetson...")

    def cloud_callback(self, msg):
        self.pending_cloud = msg

    def update(self):
        now = time.monotonic()
        if (self.visualizer is not None and now - self.last_frame > 30.0
                and self.count_publishers(FILTERED_CLOUD_TOPIC) == 0):
            self.get_logger().warning("Jetson filtered cloud publisher went offline; closing the viewer.")
            return False

        if self.pending_cloud is not None and now >= self.retry_at:
            msg = self.pending_cloud
            self.pending_cloud = None
            try:
                points = unpack_filtered_cloud(msg)
                if self.visualizer is None:
                    self.visualizer = CloudVisualizer()
                self.visualizer.show_cloud(points)
                self.last_frame = time.monotonic()
            except Exception as exc:
                if self.visualizer is not None:
                    self.visualizer.close()
                    self.visualizer = None
                self.get_logger().error(f"Could not display Jetson filtered cloud: {exc}")
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
