#! /usr/bin/env python3

##### Odometrie corrigee par le gyroscope, sur un arbre TF a nous (/tf_ma64).
#    Les roues donnent une bonne avance mais une mauvaise rotation quand l'ecartement des roues
#    du controleur est faux (simulateur officiel : 0.3409 m au lieu de ~0.39 m, rotations x1.31 ;
#    le SLAM se perd des le premier tour). Ce noeud garde la vitesse d'avance des roues et prend la
#    vitesse de rotation sur le gyroscope de l'IMU (biais retire, estime a l'arret).
#    Le robot publie deja la TF odom -> base sur /tf : on ne peut pas la remplacer sans conflit.
#    On recopie donc /tf vers /tf_ma64 en y remplacant cette seule TF par la notre ; nos noeuds
#    (SLAM, Nav2, AprilTag, mission) lisent /tf_ma64 (robot.launch.py gyro:=true).

import math
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from geometry_msgs.msg import TransformStamped


def rotation(q, v):
    """Applique la rotation du quaternion q (x, y, z, w) au vecteur v."""
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz)


class OdomGyro(Node):

    def __init__(self):
        super().__init__('odom_gyro')
        p = self.declare_parameter
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.vitesse_arret = p('vitesse_arret', 0.01).value   # roues immobiles en dessous (m/s, rad/s)
        odom_topic = p('odom_topic', '/botwheel_explorer/odom').value
        imu_topic = p('imu_topic', '/camera2/camera2/imu').value

        self.lock = threading.Lock()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)   # repere de l'IMU dans le robot (TF statique)
        self.rot_imu = None      # quaternion base <- imu
        self.biais = 0.0
        self.biais_n = 0
        self.cap = 0.0
        self.x = self.y = 0.0
        self.t_gyro = None
        self.t_odom = None
        self.a_l_arret = True
        self.v = 0.0

        self.pub_tf = self.create_publisher(TFMessage, '/tf_ma64', 100)
        self.pub_odom = self.create_publisher(Odometry, '/odom_ma64', 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.get_logger().info(f'odometrie roues {odom_topic} + gyroscope {imu_topic} -> /tf_ma64 '
                               f'({self.odom_frame} -> {self.base_frame})')

    def on_tf(self, msg):
        """Recopie /tf sans la TF odom -> base des roues (remplacee par la notre dans on_odom)."""
        garde = [t for t in msg.transforms
                 if not (t.header.frame_id.lstrip('/') == self.odom_frame
                         and t.child_frame_id.lstrip('/') == self.base_frame)]
        if garde:
            self.pub_tf.publish(TFMessage(transforms=garde))

    def on_imu(self, msg):
        if self.rot_imu is None:
            try:
                tr = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except TransformException:
                return
            r = tr.transform.rotation
            self.rot_imu = (r.x, r.y, r.z, r.w)
            self.get_logger().info(f'IMU {msg.header.frame_id} trouvee dans {self.base_frame}')
        w = msg.angular_velocity
        wz = rotation(self.rot_imu, (w.x, w.y, w.z))[2]     # vitesse de lacet dans le repere du robot
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        with self.lock:
            if self.a_l_arret:
                # robot immobile : la mesure du gyroscope est son biais (moyenne glissante)
                self.biais_n = min(self.biais_n + 1, 400)
                self.biais += (wz - self.biais) / self.biais_n
            if self.t_gyro is not None and 0.0 < t - self.t_gyro < 0.2 and not self.a_l_arret:
                self.cap = math.atan2(math.sin(self.cap + (wz - self.biais) * (t - self.t_gyro)),
                                      math.cos(self.cap + (wz - self.biais) * (t - self.t_gyro)))
            self.t_gyro = t

    def on_odom(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        with self.lock:
            self.a_l_arret = abs(v) < self.vitesse_arret and abs(w) < self.vitesse_arret
            if self.t_odom is not None and 0.0 < t - self.t_odom < 0.5:
                dt = t - self.t_odom
                self.x += v * math.cos(self.cap) * dt
                self.y += v * math.sin(self.cap) * dt
            self.t_odom = t
            x, y, cap = self.x, self.y, self.cap
        tr = TransformStamped()
        tr.header.stamp = msg.header.stamp
        tr.header.frame_id = self.odom_frame
        tr.child_frame_id = self.base_frame
        tr.transform.translation.x, tr.transform.translation.y = x, y
        tr.transform.rotation.z, tr.transform.rotation.w = math.sin(cap / 2), math.cos(cap / 2)
        self.pub_tf.publish(TFMessage(transforms=[tr]))
        o = Odometry()
        o.header = tr.header
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation = tr.transform.rotation
        o.twist = msg.twist
        self.pub_odom.publish(o)


def main():
    rclpy.init()
    node = OdomGyro()
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
