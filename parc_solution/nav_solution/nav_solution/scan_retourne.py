#! /usr/bin/env python3

##### LiDAR du vrai robot : remet les mesures dans le bon sens.
#    Mesure sur le rosbag du 05/11 : avec la TF du robot (base_link -> laser, lacet 180 deg),
#    les scans projetes donnent une carte floue ; avec les angles inverses (LiDAR monte a
#    l'envers), l'arene est un rectangle net (24 531 -> 11 666 cellules occupees de 5 cm).
#    On garde le repere 'laser' du robot et on inverse l'ordre des mesures : la mesure i passe
#    a l'angle -angle_i. Tout ce qui lit le scan (filtre, SLAM, costmaps, mission) reste juste.
#    /scan -> /scan_corrige

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRetourne(Node):

    def __init__(self):
        super().__init__('scan_retourne')
        entree = self.declare_parameter('entree', '/scan').value
        sortie = self.declare_parameter('sortie', '/scan_corrige').value
        self.pub = self.create_publisher(LaserScan, sortie, 10)
        self.create_subscription(LaserScan, entree, self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'{entree} -> {sortie} : mesures dans l\'ordre inverse (LiDAR monte a l\'envers)')

    def on_scan(self, s):
        n = len(s.ranges)
        angle_max = s.angle_min + (n - 1) * s.angle_increment
        s.angle_min, s.angle_max = -angle_max, -s.angle_min
        s.ranges = list(reversed(s.ranges))
        if len(s.intensities) == n:
            s.intensities = list(reversed(s.intensities))
        self.pub.publish(s)


def main():
    rclpy.init()
    node = ScanRetourne()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
