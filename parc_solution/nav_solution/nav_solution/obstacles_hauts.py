#! /usr/bin/env python3

##### Obstacles en hauteur vus par la camera de profondeur, publies comme un scan au sol.
#    Le LiDAR est au ras du sol : il ne voit que les pieds des tables et des chaises. Le robot
#    passait sous les plateaux et s'y coincait, ou montait sur un socle (simulateur officiel, sans
#    masque). Ce noeud lit l'image de profondeur de la camera horizontale, garde les points entre
#    hauteur_min et hauteur_max (le sol est ignore), et publie pour chaque direction le plus proche
#    sur /ma64/obstacles_hauts (LaserScan dans le repere du robot). Nav2 le prend comme une source
#    d'obstacles de plus : un plateau devient un obstacle pose au sol. Champ limite a celui de la
#    camera ; la costmap globale garde ce qui a ete vu.
#    Profondeur 16UC1 (millimetres, robot et simulateur officiel) ou 32FC1 (metres).

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def matrice(q):
    """Matrice de rotation du quaternion q (x, y, z, w)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ObstaclesHauts(Node):

    def __init__(self):
        super().__init__('obstacles_hauts')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_link').value
        self.h_min = p('hauteur_min', 0.08).value      # au-dessus du sol (le sol est ignore)
        self.h_max = p('hauteur_max', 1.35).value      # haut du robot
        self.portee = p('portee', 2.5).value
        # le robot penche (roulette : ~2.5 deg dans le simulateur) : vu de la camera, le sol "monte"
        # avec la distance ; le seuil de hauteur monte donc aussi (sol a 4.5 m : 18 cm sans ceci)
        self.pente = math.tan(math.radians(p('pente_sol_deg', 4.0).value))
        self.pas = p('pas_pixels', 4).value
        self.periode = 1.0 / p('frequence', 5.0).value
        resolution = math.radians(p('resolution_deg', 1.0).value)
        depth_topic = p('depth_topic', '/camera2/camera2/depth/image_rect_raw').value
        info_topic = p('info_topic', '/camera2/camera2/depth/camera_info').value

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.info = None
        self.cam = None          # (R, t) camera -> robot (statique)
        self.dernier = 0.0
        self.resolution = resolution
        self.pub = self.create_publisher(LaserScan, '/ma64/obstacles_hauts', 10)
        self.create_subscription(CameraInfo, info_topic, lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data)
        self.get_logger().info(f'{depth_topic} -> /ma64/obstacles_hauts (hauteur {self.h_min} m + pente, {self.h_max} m, '
                               f'portee {self.portee} m, repere {self.base_frame})')

    def on_depth(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.info is None or t - self.dernier < self.periode:
            return
        if self.cam is None:
            try:
                tr = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except TransformException:
                return
            q, v = tr.transform.rotation, tr.transform.translation
            self.cam = (matrice((q.x, q.y, q.z, q.w)), np.array([v.x, v.y, v.z]))
            self.get_logger().info(f'camera {msg.header.frame_id} a {v.z:.2f} m dans {self.base_frame}')
        self.dernier = t
        if msg.encoding == '16UC1':
            d = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, -1)[:, :msg.width] * 0.001
        elif msg.encoding == '32FC1':
            d = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, -1)[:, :msg.width]
        else:
            return
        k = self.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        s = self.pas
        v, u = np.mgrid[0:msg.height:s, 0:msg.width:s]
        z = d[::s, ::s].astype(np.float32)
        ok = np.isfinite(z) & (z > 0.1) & (z < self.portee + 0.5)
        z, u, v = z[ok], u[ok], v[ok]
        pts = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z])      # repere optique de la camera
        R, tr = self.cam
        b = R @ pts + tr[:, None]                                       # repere du robot
        h = b[2]
        r = np.hypot(b[0], b[1])
        n = int(round(2 * math.pi / self.resolution))
        # directions reellement vues (sol compris) : seules elles recoivent une mesure ; les autres
        # restent NaN (ignorees par Nav2), sinon elles effaceraient les obstacles hors du champ
        vu = np.zeros(n, bool)
        vu[np.clip(((np.arctan2(b[1], b[0]) + math.pi) / self.resolution).astype(int), 0, n - 1)] = True
        garde = (h > self.h_min + self.pente * r) & (h < self.h_max) & (r > 0.15) & (r < self.portee)
        a = np.arctan2(b[1][garde], b[0][garde])
        r = r[garde]
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min, scan.angle_max = -math.pi, math.pi - self.resolution
        scan.angle_increment = self.resolution
        scan.range_min, scan.range_max = 0.15, self.portee
        ranges = np.full(n, np.nan, np.float32)
        ranges[vu] = np.inf                                             # vu et libre jusqu'a la portee
        if r.size:
            idx = np.clip(((a + math.pi) / self.resolution).astype(int), 0, n - 1)
            np.minimum.at(ranges, idx, r.astype(np.float32))
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = ObstaclesHauts()
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
