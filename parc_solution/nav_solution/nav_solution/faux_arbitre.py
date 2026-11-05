#! /usr/bin/env python3

##### Faux arbitre PARC, pour tester la mission sans les organisateurs.
#    Reproduit les 5 topics /parc/ du document PARC2026_Topics_Arbitre_FR.pdf :
#    arme la course (ordre des tags), demarre le chrono a la reception de
#    /parc/start, recoit les visites, termine au dernier tag, sur /parc/finish
#    ou au bout de 10 minutes, et signale un faux depart si le robot bouge
#    pendant ARMED.
#    En simulation, il note aussi chaque visite avec la vraie position du
#    robot (Gazebo) : distance au tag, angle face au tag, immobilite 3 s.
#    Le rapport est ecrit en JSON a la fin de la course.

import json
import math
import os
import random

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from parc_msgs.msg import Finish, RunStatus, StartSignal, TagOrder, TagVisit

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
FIN = (RunStatus.FINISHED, RunStatus.TIMED_OUT, RunStatus.TERMINATED)
NOMS = {0: 'IDLE', 1: 'ARMED', 2: 'RUNNING', 3: 'FINISHED', 4: 'TIMED_OUT', 5: 'TERMINATED'}


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class FauxArbitre(Node):

    def __init__(self):
        super().__init__('faux_arbitre')
        ordre = list(self.declare_parameter('tag_order', [0]).value)
        if ordre == [0]:
            ordre = [1, 4, 7, 9]
            random.shuffle(ordre)
        self.ordre = ordre
        self.arm_delay = self.declare_parameter('arm_delay', 5.0).value
        self.time_limit = self.declare_parameter('time_limit', 600.0).value
        # > 0 : envoie lui-meme la commande de depart (tests automatiques)
        self.auto_depart = self.declare_parameter('auto_depart', -1.0).value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.rapport_path = self.declare_parameter('rapport', '').value
        # Verite terrain (simulation) : position et direction de chaque tag
        self.verite = self.declare_parameter('verite_pose_topic', '').value
        ids = self.declare_parameter('verite_tag_ids', [0]).value
        xs = self.declare_parameter('verite_tag_x', [0.0]).value
        ys = self.declare_parameter('verite_tag_y', [0.0]).value
        yaws = self.declare_parameter('verite_tag_yaw', [0.0]).value
        self.tags = {i: (x, y, a) for i, x, y, a in zip(ids, xs, ys, yaws) if i}
        self.decalage_face = self.declare_parameter('verite_decalage_face', 0.011).value

        self.state = RunStatus.IDLE
        self.t_start = None
        self.elapsed = 0.0
        self.reported = []
        self.start_method = 0
        self.false_start = False
        self.odom_ref = None
        self.pose = None
        self.visites = []
        self.evenements = []

        self.order_pub = self.create_publisher(TagOrder, '/parc/tag_order', LATCHED)
        self.status_pub = self.create_publisher(RunStatus, '/parc/run_status', LATCHED)
        self.start_cmd_pub = self.create_publisher(String, '/ma64/start', 10)
        self.create_subscription(StartSignal, '/parc/start', self.on_start, EVENTS)
        self.create_subscription(TagVisit, '/parc/tag_visit', self.on_visit, EVENTS)
        self.create_subscription(Finish, '/parc/finish', self.on_finish, EVENTS)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        if self.verite:
            self.create_subscription(Pose, self.verite, self.on_pose, 10)

        self.t0 = self.now()
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'Ordre de la course : {self.ordre}, armement dans {self.arm_delay:.0f} s')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, texte):
        self.get_logger().info(texte)
        self.evenements.append((round(self.elapsed, 1), texte))

    # --- entrees ---
    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.state == RunStatus.ARMED:
            if self.odom_ref is None:
                self.odom_ref = (p.x, p.y, _yaw(msg.pose.pose.orientation))
            elif (math.hypot(p.x - self.odom_ref[0], p.y - self.odom_ref[1]) > 0.03
                  or abs(_yaw(msg.pose.pose.orientation) - self.odom_ref[2]) > 0.05) and not self.false_start:
                self.false_start = True
                self.log('FAUX DEPART : le robot bouge avant le depart')
                self.publie()

    def on_pose(self, msg):
        self.pose = (msg.position.x, msg.position.y, _yaw(msg.orientation))
        q = msg.orientation
        inclinaison = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x ** 2 + q.y ** 2)))))
        if (msg.position.z > 0.08 or inclinaison > 15.0) and not getattr(self, 'renverse', False):
            self.renverse = True
            self.log(f'ROBOT SOULEVE OU RENVERSE : z {msg.position.z:.2f} m, inclinaison {inclinaison:.0f} deg '
                     f'en ({msg.position.x:.2f}, {msg.position.y:.2f})')
        for v in self.visites:
            if 'immobile_3s' not in v and self.now() - v['t'] >= 3.0:
                dep = math.hypot(self.pose[0] - v['pose'][0], self.pose[1] - v['pose'][1])
                v['deplacement_3s_m'] = round(dep, 3)
                v['immobile_3s'] = dep < 0.02

    def on_start(self, msg):
        if self.state != RunStatus.ARMED:
            self.log(f'StartSignal ignore : etat {NOMS[self.state]}')
            return
        self.state = RunStatus.RUNNING
        self.t_start = self.now()
        self.start_method = msg.method
        self.log(f'DEPART ({"voix : " + msg.heard if msg.method == StartSignal.METHOD_VOICE else "commande"})')
        self.publie()

    def on_visit(self, msg):
        if self.state != RunStatus.RUNNING:
            self.log(f'TagVisit {msg.tag_id} ignore : etat {NOMS[self.state]}')
            return
        if msg.tag_id in self.reported:
            self.log(f'TagVisit {msg.tag_id} en double')
            return
        self.reported.append(msg.tag_id)
        v = {'tag': msg.tag_id, 't': self.now(), 'temps_course_s': round(self.elapsed, 1),
             'image_format': msg.image.format, 'image_octets': len(msg.image.data),
             'frame_id': msg.header.frame_id}
        if self.rapport_path:
            nom = os.path.splitext(self.rapport_path)[0] + f'_tag{msg.tag_id}.jpg'
            with open(nom, 'wb') as f:
                f.write(bytes(msg.image.data))
            v['image'] = nom
        if self.pose and msg.tag_id in self.tags:
            tx, ty, tyaw = self.tags[msg.tag_id]
            # position du modele = centre du poteau ; la face imprimee est 11 mm devant
            tx, ty = tx + self.decalage_face * math.cos(tyaw), ty + self.decalage_face * math.sin(tyaw)
            rx, ry, ryaw = self.pose
            d = math.hypot(tx - rx, ty - ry)
            face = math.degrees(math.atan2(math.sin(math.atan2(ty - ry, tx - rx) - ryaw),
                                           math.cos(math.atan2(ty - ry, tx - rx) - ryaw)))
            devant = (rx - tx) * math.cos(tyaw) + (ry - ty) * math.sin(tyaw)
            lateral = -(rx - tx) * math.sin(tyaw) + (ry - ty) * math.cos(tyaw)
            v.update({'pose': self.pose, 'distance_m': round(d, 3), 'angle_face_deg': round(face, 1),
                      'devant_m': round(devant, 3), 'lateral_m': round(lateral, 3),
                      'valide_hypothese': d <= 0.5 and abs(face) <= 12.0})
        self.visites.append(v)
        rang = self.ordre.index(msg.tag_id) if msg.tag_id in self.ordre else -1
        self.log(f'VISITE tag {msg.tag_id} (rang {rang + 1}) a {self.elapsed:.0f} s : '
                 + ', '.join(f'{k}={v[k]}' for k in ('distance_m', 'angle_face_deg', 'lateral_m',
                                                     'valide_hypothese') if k in v))
        if msg.tag_id == self.ordre[-1]:
            self.termine(RunStatus.FINISHED, 'dernier tag de l\'ordre signale')
        self.publie()

    def on_finish(self, msg):
        if self.state == RunStatus.RUNNING:
            self.termine(RunStatus.FINISHED, f'Finish recu (raison {msg.reason})')
            self.publie()

    # --- boucle ---
    def tick(self):
        t = self.now()
        if self.state == RunStatus.IDLE and t - self.t0 >= self.arm_delay:
            self.state = RunStatus.ARMED
            self.t_armed = t
            o = TagOrder()
            o.stamp = self.get_clock().now().to_msg()
            o.tag_ids = self.ordre
            self.order_pub.publish(o)
            self.log(f'ARME, ordre {self.ordre}')
        if self.state == RunStatus.ARMED and 0 < self.auto_depart <= t - self.t_armed:
            self.start_cmd_pub.publish(String(data='command'))
        if self.state == RunStatus.RUNNING:
            self.elapsed = t - self.t_start
            if self.elapsed >= self.time_limit:
                self.termine(RunStatus.TIMED_OUT, '10 minutes atteintes')
        self.publie()

    def termine(self, etat, raison):
        self.state = etat
        self.log(f'FIN : {NOMS[etat]} ({raison}), tags signales {self.reported}')
        self.create_timer(4.0, self.ecrit_rapport)

    def ecrit_rapport(self):
        if not self.rapport_path or getattr(self, 'rapport_ecrit', False):
            return
        self.rapport_ecrit = True
        # points dans l'ordre : un tag compte s'il arrive apres le dernier tag compte
        dernier, dans_ordre = -1, []
        for v in self.visites:
            rang = self.ordre.index(v['tag']) if v['tag'] in self.ordre else -1
            if rang > dernier:
                dans_ordre.append(v['tag'])
                dernier = rang
        rapport = {'ordre': self.ordre, 'etat_final': NOMS[self.state], 'duree_s': round(self.elapsed, 1),
                   'faux_depart': self.false_start, 'tags_signales': self.reported,
                   'tags_dans_l_ordre': dans_ordre, 'visites': self.visites, 'evenements': self.evenements}
        with open(self.rapport_path, 'w') as f:
            json.dump(rapport, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f'Rapport ecrit : {self.rapport_path}')

    def publie(self):
        s = RunStatus()
        s.stamp = self.get_clock().now().to_msg()
        s.state = self.state
        s.elapsed_s = float(self.elapsed)
        s.time_limit_s = float(self.time_limit)
        s.tag_order = self.ordre if self.state != RunStatus.IDLE else []
        s.tags_reported = self.reported
        s.start_method = self.start_method
        s.false_start = self.false_start
        self.status_pub.publish(s)


def main():
    rclpy.init()
    node = FauxArbitre()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ecrit_rapport()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
