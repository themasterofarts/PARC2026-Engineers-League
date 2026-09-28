#!/usr/bin/env python3
"""Thin the top depth camera's point cloud into something the costmaps can use.

The LiDAR scans ~4 cm above the floor, so it misses anything that doesn't
reach the floor at that height: cafe tabletops overhang their bases, and the
world's standing visitor model floats 16 cm above the floor. The top D435
sees both, but its raw cloud (640x480 at 15 Hz) is too heavy for the costmap
and full of inf values (no return), which break costmap raytracing.

This node keeps every STEP-th pixel in each direction of the organized cloud,
drops non-finite points, and republishes in the same frame on
/top_camera_depth/points_thinned. The costmaps (3D VoxelLayer) use it to mark
and clear, within a height band clear of the floor and its depth noise.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

STEP = 6


class DepthObstacles(Node):
    def __init__(self):
        super().__init__("depth_obstacles")
        self.pub = self.create_publisher(PointCloud2, "/top_camera_depth/points_thinned", 5)
        self.create_subscription(PointCloud2, "/top_camera_depth/points", self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=False)
        if msg.height > 1:  # organized: subsample rows and columns
            pts = pts.reshape(msg.height, msg.width, 3)[::STEP, ::STEP].reshape(-1, 3)
        else:
            pts = pts[::STEP * STEP]
        pts = pts[np.isfinite(pts).all(axis=1)]
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, pts.astype(np.float32)))


def main():
    rclpy.init()
    node = DepthObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
