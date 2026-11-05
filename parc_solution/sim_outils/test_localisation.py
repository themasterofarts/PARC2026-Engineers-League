#!/usr/bin/env python3
"""Banc de test de la localisation (simulateur officiel, tests seulement).

Fait faire au robot un parcours fixe sur /cmd_vel (deux tours sur place, 1 m tout droit, un
tour dans l'autre sens), puis compare la pose estimee (TF map -> base_footprint, SLAM) a la
vraie pose Gazebo (/verite/robot_pose), ramenee dans le repere map (depart en (0, 0), cap 0).
Ecrit un JSON avec l'erreur de position et de cap a chaque etape.

    python3 test_localisation.py --rapport /tmp/loc.json
"""
import argparse
import json
import math

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rapport', required=True)
    ap.add_argument('--depart', default='0,-7,1.5708', help='pose de depart dans le monde : x,y,cap')
    ap.add_argument('--tf', default='/tf', help='/tf_ma64 avec robot.launch.py gyro:=true')
    a = ap.parse_args()
    x0, y0, c0 = (float(v) for v in a.depart.split(','))

    rclpy.init()
    n = rclpy.create_node('test_localisation', cli_args=['--ros-args', '-r', f'/tf:={a.tf}'])
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, n)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    verite = {}
    n.create_subscription(Pose, '/verite/robot_pose', lambda m: verite.update(p=m), 10)

    def t():
        return n.get_clock().now().nanoseconds * 1e-9

    def attendre(duree, vx=0.0, wz=0.0):
        t0 = t()
        while t() - t0 < duree:
            m = Twist()
            m.linear.x, m.angular.z = vx, wz
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.05)

    def mesure(etape):
        for _ in range(40):
            rclpy.spin_once(n, timeout_sec=0.05)
        try:
            tr = tf.lookup_transform('map', 'base_footprint', Time())
        except TransformException as e:
            return {'etape': etape, 'erreur': str(e)}
        v = verite.get('p')
        if v is None:
            return {'etape': etape, 'erreur': 'pas de verite'}
        # vraie pose dans le repere map (depart = origine, cap 0)
        dx, dy = v.position.x - x0, v.position.y - y0
        c, s = math.cos(-c0), math.sin(-c0)
        vx, vy, vc = c * dx - s * dy, s * dx + c * dy, wrap(yaw_of(v.orientation) - c0)
        ex, ey = tr.transform.translation.x, tr.transform.translation.y
        ec = yaw_of(tr.transform.rotation)
        r = {'etape': etape, 'verite': [round(vx, 3), round(vy, 3), round(math.degrees(vc), 1)],
             'slam': [round(ex, 3), round(ey, 3), round(math.degrees(ec), 1)],
             'erreur_position_m': round(math.hypot(ex - vx, ey - vy), 3),
             'erreur_cap_deg': round(math.degrees(wrap(ec - vc)), 1)}
        n.get_logger().info(str(r))
        return r

    while t() < 1.0 or 'p' not in verite:
        rclpy.spin_once(n, timeout_sec=0.1)
    attendre(5.0)
    res = [mesure('depart')]
    # 0.4 rad/s commande : en simulation la rotation reelle depend de wheel_separation
    attendre(2 * 2 * math.pi / 0.4, wz=0.4)
    attendre(2.0)
    res.append(mesure('deux tours'))
    attendre(1.0 / 0.15, vx=0.15)
    attendre(2.0)
    res.append(mesure('1 m tout droit'))
    attendre(2 * math.pi / 0.4, wz=-0.4)
    attendre(2.0)
    res.append(mesure('un tour inverse'))
    attendre(1.0 / 0.15, vx=0.15)
    attendre(2.0)
    res.append(mesure('1 m tout droit'))
    json.dump(res, open(a.rapport, 'w'), indent=1)
    print(json.dumps(res))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
