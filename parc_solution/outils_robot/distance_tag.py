#!/usr/bin/env python3
"""Etalonnage : affiche en continu la distance camera base_link -> tag (repere du sol).

    python3 distance_tag.py 4        # tag 4, cadre TF tag_4 publie par apriltag_node

Comparer a la distance mesuree au metre ruban (centre du robot -> face du tag), puis :
    nouvelle tag_size = tag_size x distance ruban / distance affichee
"""
import math
import sys

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else '4'
    rclpy.init()
    node = rclpy.create_node('distance_tag')
    tf = Buffer()
    TransformListener(tf, node)
    mesures = []

    def affiche():
        try:
            tr = tf.lookup_transform('base_link', f'tag_{tag}', Time())
        except TransformException:
            print(f'tag {tag} non vu', flush=True)
            return
        age = (node.get_clock().now() - Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9
        if age > 1.0:
            print(f'tag {tag} non vu depuis {age:.0f} s', flush=True)
            return
        t = tr.transform.translation
        d = math.hypot(t.x, t.y)
        mesures.append(d)
        del mesures[:-20]
        print(f'tag {tag} : distance {d:.3f} m   moyenne des 20 dernieres {sum(mesures) / len(mesures):.3f} m',
              flush=True)

    node.create_timer(0.5, affiche)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
