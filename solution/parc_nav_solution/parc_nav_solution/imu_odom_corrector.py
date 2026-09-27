#!/usr/bin/env python3
"""Publish an IMU-heading-corrected odometry frame, odom_imu.

The robot's wheel odometry overestimates every turn by ~30% (measured against
Gazebo ground truth: a spin reported as 134.6 deg was really 102.5 deg), while
straight-line distance is accurate to ~1%. The IMU orientation matches ground
truth heading. This node dead-reckons the robot with the IMU's heading and the
wheels' forward displacement, and publishes the correction as the transform
odom_imu -> odom, so the existing odom -> base_footprint transform from the
simulator is left untouched (the same pattern AMCL/SLAM use for map -> odom).

At startup odom_imu coincides with odom, so poses expressed in odom before the
robot moves are equally valid in odom_imu.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ImuOdomCorrector(Node):
    def __init__(self):
        super().__init__("imu_odom_corrector")
        self.frame = self.declare_parameter("corrected_frame", "odom_imu").value
        self.imu_yaw = None
        self.imu_yaw0 = None
        self.last_odom = None  # (x, y, yaw) of the previous /odom message
        self.pose = None  # corrected (x, y, yaw) of base_footprint in odom_imu
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)

    def on_imu(self, msg):
        self.imu_yaw = _yaw(msg.orientation)

    def on_odom(self, msg):
        if self.imu_yaw is None:
            return
        p = msg.pose.pose.position
        odom = (p.x, p.y, _yaw(msg.pose.pose.orientation))

        if self.pose is None:
            self.imu_yaw0 = self.imu_yaw
            self.start_yaw = odom[2]
            self.pose = odom
        else:
            dx, dy = odom[0] - self.last_odom[0], odom[1] - self.last_odom[1]
            # Forward displacement along odom's own heading: its magnitude is
            # reliable even though that heading isn't.
            forward = dx * math.cos(self.last_odom[2]) + dy * math.sin(self.last_odom[2])
            yaw_prev = self.pose[2]
            yaw = _wrap(self.start_yaw + self.imu_yaw - self.imu_yaw0)
            mid = yaw_prev + _wrap(yaw - yaw_prev) / 2.0
            self.pose = (
                self.pose[0] + forward * math.cos(mid),
                self.pose[1] + forward * math.sin(mid),
                yaw,
            )
        self.last_odom = odom
        self.publish(msg.header.stamp, odom)

    def publish(self, stamp, odom):
        # odom_imu -> odom = T(odom_imu -> base) * inverse(T(odom -> base))
        cx, cy, cyaw = self.pose
        ox, oy, oyaw = odom
        yaw = _wrap(cyaw - oyaw)
        c, s = math.cos(yaw), math.sin(yaw)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame
        t.child_frame_id = "odom"
        t.transform.translation.x = cx - (c * ox - s * oy)
        t.transform.translation.y = cy - (s * ox + c * oy)
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = ImuOdomCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
