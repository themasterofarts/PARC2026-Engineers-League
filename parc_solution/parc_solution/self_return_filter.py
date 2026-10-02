#!/usr/bin/env python3
"""Remove only LiDAR returns produced by the robot's own wheels and caster.

The filter preserves nearby measurements in every other direction.  It is not
a global minimum-range filter: a close chair leg, wall, or person must remain
visible to Nav2.
"""

from copy import deepcopy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


class SelfReturnFilter(Node):
    """Publish `/scan_filtered`, with known robot-body returns removed."""

    # Coordinates are in lidar_link. The LiDAR is 0.021 m behind the
    # base_footprint origin. Wheel centres are at x=0.095 m, y=+/-0.19649 m;
    # the caster centre is at x=-0.133 m, y=0. Values include a 1.5 cm margin
    # for LiDAR noise and the small sensor pitch in the supplied robot model.
    _SELF_CIRCLES = (
        (0.116, 0.19649, 0.101),   # left drive wheel
        (0.1155, -0.19649, 0.101), # right drive wheel
        (-0.112, 0.0, 0.066),      # rear caster
    )

    # Gazebo's bridged sensor data can be consumed with BEST_EFFORT, but the
    # Jazzy SLAM Toolbox subscription requests RELIABLE. Publish the derived
    # scan reliably so both SLAM Toolbox and Nav2 can subscribe to it.
    _OUTPUT_QOS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    def __init__(self) -> None:
        super().__init__("self_return_filter")
        self._publisher = self.create_publisher(
            LaserScan, "scan_filtered", self._OUTPUT_QOS
        )
        self._subscription = self.create_subscription(
            LaserScan, "scan", self._filter_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Filtering known wheel/caster self-returns: /scan -> /scan_filtered"
        )

    @classmethod
    def _is_robot_return(cls, distance: float, angle: float) -> bool:
        """Return true when a scan endpoint falls in a known robot component."""
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return any(
            (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2
            for center_x, center_y, radius in cls._SELF_CIRCLES
        )

    def _filter_scan(self, scan: LaserScan) -> None:
        filtered = deepcopy(scan)
        angle = scan.angle_min
        ranges = []

        for distance in scan.ranges:
            if math.isfinite(distance) and self._is_robot_return(distance, angle):
                # Infinity is the LaserScan convention for "no obstacle on
                # this ray". Nav2 and SLAM Toolbox will ignore this reading.
                ranges.append(math.inf)
            else:
                ranges.append(distance)
            angle += scan.angle_increment

        filtered.ranges = ranges
        self._publisher.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SelfReturnFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
