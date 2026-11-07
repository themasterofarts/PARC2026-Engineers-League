#!/usr/bin/env python3
"""Envoyeur du depart pour course.sh, lance AVANT la simulation.

Avec ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, un processus lance tard (ros2 topic pub au moment du
depart) ne trouvait plus le noeud de mission quand beaucoup de noeuds tournaient (masque + RViz,
7 nov) : le depart n'arrivait jamais. Ce noeud est cree en premier ; il attend le fichier FICHIER,
puis publie "go" sur /ma64/start toutes les 2 s jusqu'a ce qu'on le tue.

    python3 depart_ma64.py FICHIER
"""
import os
import sys
import time

import rclpy
from std_msgs.msg import String


def main():
    fichier = sys.argv[1]
    rclpy.init()
    n = rclpy.create_node('depart_ma64')
    pub = n.create_publisher(String, '/ma64/start', 10)
    try:
        while not os.path.exists(fichier):
            rclpy.spin_once(n, timeout_sec=0.5)
        while rclpy.ok():
            pub.publish(String(data='go'))
            n.get_logger().info(f'go publie ({pub.get_subscription_count()} abonne(s))')
            t0 = time.time()
            while time.time() - t0 < 2.0:
                rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
