#! /usr/bin/env python3

##### Noeud de mission PARC phase reelle (equipe MA64).
#    Deroule une course complete en parlant a l'arbitre (topics /parc/) :
#      1. attend l'ordre des tags (/parc/tag_order) ; le robot ne bouge pas ;
#      2. attend le depart : mot reconnu sur /ma64/start_vocal (demarrage_vocal, METHOD_VOICE)
#         ou n'importe quel message sur /ma64/start, envoye a la main (METHOD_COMMAND),
#         puis publie un seul StartSignal ;
#      3. pour chaque tag dans l'ordre :
#           - si sa position est inconnue, explore : tour sur place, puis
#             points de vue choisis sur la carte SLAM, jusqu'a le voir ;
#           - Nav2 amene le robot a ~1 m devant le tag ;
#           - approche finale guidee par la camera jusqu'a 0.5 m, face au tag ;
#           - publie un TagVisit (image JPEG du tag) et reste immobile 3.5 s ;
#         un tag qui prend trop de temps est saute (les suivants comptent encore) ;
#      4. s'arrete quand /parc/run_status passe a FINISHED, TIMED_OUT ou TERMINATED.
#    La position des tags vient de apriltag_tf_node (/apriltag/tag_<id>/pose, repere map),
#    leur position en direct de la TF camera -> tag_<id> publiee par apriltag_node.

import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
import yaml
from action_msgs.msg import GoalStatus
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from parc_msgs.msg import Finish, RunStatus, StartSignal, TagOrder, TagVisit

try:
    import cv2
except ImportError:  # encodage JPEG de secours plus bas
    cv2 = None

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
FIN = (RunStatus.FINISHED, RunStatus.TIMED_OUT, RunStatus.TERMINATED)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def tourne(x, y, a):
    """Vecteur (x, y) tourne de a (rad)."""
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def normale_tag(q):
    """Axe z du tag (sort de la face imprimee, vers la camera), projete au sol."""
    nx = 2 * (q.x * q.z + q.w * q.y)
    ny = 2 * (q.y * q.z - q.w * q.x)
    n = math.hypot(nx, ny)
    return (nx / n, ny / n) if n > 1e-6 else None


def clamp(v, lim):
    return max(-lim, min(lim, v))


class CourseFinie(Exception):
    """Levee des que l'arbitre annonce la fin de la course."""


class Mission(Node):

    def __init__(self):
        super().__init__('mission')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_link').value
        self.map_frame = p('map_frame', 'map').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.tag_prefix = p('tag_frame_prefix', 'tag_').value
        self.tag_ids = list(p('tag_ids', [1, 4, 7, 9]).value)
        self.visit_dist = p('visit_distance', 0.60).value
        self.view_dist = p('view_distance', 1.2).value
        self.time_limit = p('time_limit', 600.0).value
        self.tag_budget_min = p('tag_budget_min', 60.0).value
        self.hold_time = p('hold_time', 3.5).value
        self.backup_after_visit = p('backup_after_visit', 0.3).value
        self.max_lin = p('max_linear', 0.15).value
        self.max_ang = p('max_angular', 0.6).value
        self.spin_speed = p('spin_speed', 0.45).value
        self.look_pause = p('look_pause', 1.2).value
        self.front_stop = p('front_stop_distance', 0.25).value
        # ouverture : au depart, avance tout droit (m) puis tourne (deg, + = gauche), avant
        # d'explorer. Pour une marque de depart dans un coin. 0 = pas d'ouverture.
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.ouverture_avance = float(self.declare_parameter('ouverture_avance', 0.0, dyn).value)
        self.ouverture_virage = float(self.declare_parameter('ouverture_virage', 0.0, dyn).value)
        self.front_sector = math.radians(p('front_sector_deg', 20.0).value)
        self.pos_tol = p('approach_position_tolerance', 0.04).value
        self.dist_tol = p('visit_distance_tolerance', 0.05).value
        self.face_tol = math.radians(p('visit_angle_tolerance_deg', 5.0).value)
        self.approach_timeout = p('approach_timeout', 45.0).value
        self.stuck_time = p('stuck_time', 3.0).value   # duree d'observation du patinage (s)
        self.nav_timeout = p('nav_timeout', 60.0).value
        self.min_margin = p('min_decision_margin', 30.0).value
        self.explore_clearance = p('explore_clearance', 0.40).value
        self.explore_spacing = p('explore_spacing', 0.8).value
        self.explore_max_cost = p('explore_max_cost', 70).value  # < cout circonscrit (~78) : le robot entier tient
        self.wait_running = p('wait_running_status', True).value
        self.test_approche = p('test_approche_tag', 0).value
        self.journal_dir = os.path.expanduser(p('journal_dir', '').value)
        # positions des tags notees pendant la cartographie (sauver_tags) : simples indices
        fichier = os.path.expanduser(p('indices_tags', '').value)
        self.indices = {}
        if fichier:
            try:
                with open(fichier) as f:
                    self.indices = {int(k): v for k, v in (yaml.safe_load(f) or {}).get('tags', {}).items()}
            except (OSError, yaml.YAMLError, AttributeError, ValueError) as e:
                # pas de positions notees : le robot explore pour trouver chaque tag
                self.get_logger().warn(f'indices ignores ({fichier}) : {e}')
        cmd_topic = p('cmd_vel_topic', '/cmd_vel').value
        scan_topic = p('scan_topic', '/scan').value
        scan_filtre_topic = p('scan_filtered_topic', '/scan_ma64').value
        self.rotation_clearance = p('rotation_clearance', 0.38).value
        # demi-empreinte du robot (meme valeurs que la footprint Nav2)
        self.half_length = p('footprint_half_length', 0.25).value
        self.half_width = p('footprint_half_width', 0.25).value
        image_topic = p('image_topic', '/camera1/camera1/color/image_raw').value
        detections_topic = p('detections_topic', '/detections').value

        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        # vrai robot : /botwheel_explorer/cmd_vel en TwistStamped (consigne des organisateurs) ;
        # simulation : Twist
        self.cmd_stamped = p('cmd_vel_stamped', False).value
        self.cmd_pub = self.create_publisher(TwistStamped if self.cmd_stamped else Twist, cmd_topic, 10)
        self.start_pub = self.create_publisher(StartSignal, '/parc/start', EVENTS)
        self.visit_pub = self.create_publisher(TagVisit, '/parc/tag_visit', EVENTS)
        self.finish_pub = self.create_publisher(Finish, '/parc/finish', EVENTS)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)
        self.nav_manager = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes',
                                              callback_group=self.cb)
        self.derniere_relance = -1e9

        self.order = None
        self.status = None
        self.start_word = None
        self.start_vocal = False   # le depart vient de la reconnaissance vocale
        self.tags = {}         # id -> (x, y, z, nx, ny) dans map
        self.last_seen = {}    # id -> temps de la derniere detection fiable
        self.image = None
        self.image_c = None
        self.capture = False      # images gardees seulement pendant une visite
        self.subs_images = []
        self.scan = None
        self.scan_filtre = None
        self.laser_yaw = None
        self.grid = None
        self.costmap = None
        self.costmap_locale = None
        self.viewpoints = []   # points de vue deja explores (map)
        self.journal = []
        self.started = False

        self.create_subscription(TagOrder, '/parc/tag_order', self.on_order, LATCHED, callback_group=self.cb)
        self.create_subscription(RunStatus, '/parc/run_status', self.on_status, LATCHED, callback_group=self.cb)
        self.create_subscription(String, '/ma64/start', self.on_start_word, 10, callback_group=self.cb)
        self.create_subscription(String, '/ma64/start_vocal', lambda m: self.on_start_word(m, vocal=True), 10,
                                 callback_group=self.cb)
        for i in self.tag_ids:
            self.create_subscription(PoseStamped, f'/apriltag/tag_{i}/pose',
                                     lambda m, i=i: self.on_tag_pose(i, m), LATCHED, callback_group=self.cb)
        self.create_subscription(AprilTagDetectionArray, detections_topic, self.on_detections, 10,
                                 callback_group=self.cb)
        # images : abonnement seulement au moment de la visite (decoder 30 images/s en Python coute cher)
        self.image_topic = image_topic
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(LaserScan, scan_filtre_topic, lambda m: setattr(self, 'scan_filtre', m),
                                 qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED, callback_group=self.cb)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, LATCHED,
                                 callback_group=self.cb)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: setattr(self, 'costmap_locale', m), LATCHED, callback_group=self.cb)

    # ---------- reception ----------
    def on_order(self, msg):
        if not msg.tag_ids:
            return   # l'arbitre publie un ordre vide tant que la course n'est pas armee
        self.order = list(msg.tag_ids)
        self.log(f'ordre recu : {self.order}')

    def on_status(self, msg):
        ancien = self.status.state if self.status else None
        self.status = msg
        if msg.state != ancien:
            self.log(f'etat arbitre : {msg.state} (temps {msg.elapsed_s:.0f} s, signales {list(msg.tags_reported)})')
        if msg.state in FIN:
            self.commande(0.0, 0.0)

    def on_start_word(self, msg, vocal=False):
        # un mot entendu avant l'armement ne doit pas faire partir le robot a l'armement
        if self.status is None:
            armee = self.order is not None     # essai sans arbitre : seulement apres l'ordre
        else:
            armee = self.status.state == RunStatus.ARMED
        if not armee:
            self.log(f'mot de depart ignore : course pas armee ("{msg.data.strip()}")')
            return
        if self.start_word is None:
            self.start_vocal = vocal
            self.start_word = msg.data.strip().lower()

    def on_tag_pose(self, i, msg):
        n = normale_tag(msg.pose.orientation)
        if n is None:
            return
        p = msg.pose.position
        if i not in self.tags:
            self.log(f'tag {i} localise sur la carte : ({p.x:.2f}, {p.y:.2f})')
        self.tags[i] = (p.x, p.y, p.z, n[0], n[1])

    def on_detections(self, msg):
        t = self.t()
        for d in msg.detections:
            if d.decision_margin >= self.min_margin:
                self.last_seen[d.id] = t

    def on_image(self, msg):
        if self.capture:
            self.image = msg

    def on_image_c(self, msg):
        if self.capture:
            self.image_c = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_map(self, msg):
        self.grid = msg

    def on_costmap(self, msg):
        self.costmap = msg

    # ---------- outils ----------
    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, duree):
        t0 = self.t()
        while self.t() - t0 < duree:
            time.sleep(0.02)

    def log(self, texte):
        self.get_logger().info(texte)
        temps = self.status.elapsed_s if self.status else 0.0
        self.journal.append((round(float(temps), 1), texte))

    def fini(self):
        return self.status is not None and self.status.state in FIN

    def verifie(self):
        if self.fini():
            raise CourseFinie()

    def temps_restant(self):
        if self.status is not None and self.status.state == RunStatus.RUNNING:
            return self.status.time_limit_s - self.status.elapsed_s
        return self.time_limit - (self.t() - self.t_depart)

    def commande(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        if self.cmd_stamped:
            m = TwistStamped(twist=m)
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.base_frame
        self.cmd_pub.publish(m)

    def stop(self, duree=0.3):
        t0, mur = self.t(), time.time()
        while True:
            self.commande(0.0, 0.0)
            # borne en temps reel aussi : en simulation, /clock s'arrete avec Gazebo a la fermeture
            ecoule, ecoule_mur = self.t() - t0, time.time() - mur
            if ecoule >= duree or ecoule_mur > 8 * duree + 1.0 or (ecoule_mur > duree + 0.5 and ecoule < 1e-3):
                break
            time.sleep(0.05)

    def pose(self, frame):
        try:
            tr = self.tf.lookup_transform(frame, self.base_frame, Time())
        except TransformException:
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    def vu_recemment(self, i, age=0.5):
        return i in self.last_seen and self.t() - self.last_seen[i] <= age

    def tag_direct(self, i, frame):
        """Position (x, y, nx, ny) du tag vu en ce moment par la camera, dans frame."""
        if not self.vu_recemment(i, 0.6):
            return None
        try:
            tr = self.tf.lookup_transform(frame, f'{self.tag_prefix}{i}', Time())
        except TransformException:
            return None
        if self.t() - Time.from_msg(tr.header.stamp).nanoseconds * 1e-9 > 0.8:
            return None
        n = normale_tag(tr.transform.rotation)
        if n is None:
            return None
        t = tr.transform.translation
        return t.x, t.y, n[0], n[1]

    def degagement(self):
        """Distance au plus proche obstacle tout autour (scan filtre, sans le chassis)."""
        s = self.scan_filtre
        if s is None:
            return None
        r = np.asarray(s.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r > s.range_min)
        return float(r[ok].min()) if ok.any() else 10.0

    def secteur(self, s, centre, demi_angle):
        """Distance la plus courte du scan s dans le secteur centre +- demi_angle (repere du robot)."""
        if s is None:
            return None
        if self.laser_yaw is None:
            try:
                tr = self.tf.lookup_transform(self.base_frame, s.header.frame_id, Time())
                self.laser_yaw = yaw_of(tr.transform.rotation)
            except TransformException:
                return None
        r = np.asarray(s.ranges, dtype=np.float32)
        a = s.angle_min + np.arange(r.size) * s.angle_increment + self.laser_yaw - centre
        a = np.arctan2(np.sin(a), np.cos(a))
        ok = np.isfinite(r) & (r > max(s.range_min, 0.12)) & (np.abs(a) < demi_angle)
        return float(r[ok].min()) if ok.any() else 10.0

    def devant(self):
        """Distance libre devant le robot (scan brut, secteur +-front_sector)."""
        return self.secteur(self.scan, 0.0, self.front_sector)

    def obstacle_proche(self):
        """(distance, direction dans le repere du robot) de l'obstacle le plus proche, ou None."""
        self.devant()  # calcule l'orientation du LiDAR dans le repere du robot (laser_yaw)
        s = self.scan_filtre
        libre = self.degagement()
        if s is None or libre is None or self.laser_yaw is None:
            return None
        r = np.asarray(s.ranges, dtype=np.float32)
        r = np.where(np.isfinite(r) & (r > s.range_min), r, np.inf)
        return libre, wrap(s.angle_min + int(np.argmin(r)) * s.angle_increment + self.laser_yaw)

    def degager(self):
        """Colle a un obstacle, Nav2 se croit en collision et ne repart pas : on s'ecarte
        d'abord en ligne droite, a l'oppose de l'obstacle le plus proche ; si ce sens est
        bloque dans la costmap, dans l'autre sens (sinon le robot reste coince : seed 7, 06/11)."""
        o = self.obstacle_proche()
        if o is None or o[0] >= self.rotation_clearance:
            return
        libre, a = o
        self.log(f'obstacle a {libre:.2f} m ({math.degrees(a):.0f} deg) : on se degage')
        essais = [(self.reculer, 0.25), (self.avancer, 0.2)]
        if abs(a) >= math.pi / 2:
            essais.reverse()
        for bouger, distance in essais:
            if bouger(distance):
                return

    def trajet_libre(self, distance):
        """Distance (m, signee : < 0 en marche arriere) que le robot peut parcourir en ligne
        droite sans que son empreinte touche un nouvel obstacle de la costmap locale. Les
        obstacles deja sous l'empreinte au depart (robot colle) ne bloquent pas : c'est d'eux
        qu'on s'ecarte. Sans costmap, renvoie distance (seul le scan protege alors)."""
        c = self.costmap_locale
        if c is None:
            return distance
        try:
            tr = self.tf.lookup_transform(c.header.frame_id, self.base_frame, Time())
        except TransformException:
            return distance
        x0, y0 = tr.transform.translation.x, tr.transform.translation.y
        yaw = yaw_of(tr.transform.rotation)
        res, ox, oy = c.info.resolution, c.info.origin.position.x, c.info.origin.position.y
        data = np.asarray(c.data, dtype=np.int16).reshape(c.info.height, c.info.width)
        lx, ly = np.meshgrid(np.arange(-self.half_length, self.half_length + 1e-6, res),
                             np.arange(-self.half_width, self.half_width + 1e-6, res))
        lx, ly = lx.ravel(), ly.ravel()

        c_yaw, s_yaw = math.cos(yaw), math.sin(yaw)

        def touche(d):
            """True si l'empreinte avancee de d recouvre un obstacle qu'on ne quitte pas."""
            px = x0 + (lx + d) * c_yaw - ly * s_yaw
            py = y0 + (lx + d) * s_yaw + ly * c_yaw
            i = np.floor((px - ox) / res).astype(int)
            j = np.floor((py - oy) / res).astype(int)
            ok = (i >= 0) & (i < c.info.width) & (j >= 0) & (j < c.info.height)
            i, j = i[ok], j[ok]
            occ = data[j, i] >= 100
            if not occ.any():
                return False
            # centre des cellules occupees, dans le repere du robot au depart
            cx = ox + (i[occ] + 0.5) * res - x0
            cy = oy + (j[occ] + 0.5) * res - y0
            rx, ry = cx * c_yaw + cy * s_yaw, -cx * s_yaw + cy * c_yaw
            sous_depart = (np.abs(rx) <= self.half_length + res) & (np.abs(ry) <= self.half_width + res)
            # un obstacle deja sous l'empreinte ne bloque que si on va vers lui
            quitte = sous_depart & (np.sign(rx) == -np.sign(d))
            return bool((~quitte).any())

        pas = math.copysign(0.03, distance)
        d = 0.0
        while abs(d + pas) <= abs(distance) + 1e-6:
            if touche(d + pas):
                break
            d += pas
        return d

    def avancer(self, distance):
        """Avance en ligne droite (odometrie), si le scan brut ne voit rien devant et que
        la costmap locale est libre sous l'empreinte."""
        distance = self.trajet_libre(distance)
        if distance < 0.05:
            self.log('avance annulee : pas de place dans la costmap')
            return False
        p0 = self.pose(self.odom_frame)
        if p0 is None:
            return False
        t0, mur = self.t(), time.time()
        duree = distance / 0.08 + 3.0
        while self.t() - t0 < duree:
            self.verifie()
            if time.time() - mur > 8 * duree:
                # borne en temps reel : horloge simulee ou TF figees (course s740)
                self.log('deplacement en ligne droite arrete : temps depasse (horloge ou TF figees ?)')
                break
            p = self.pose(self.odom_frame)
            if p is None or math.hypot(p[0] - p0[0], p[1] - p0[1]) >= distance:
                break
            if (self.devant() or 10.0) < self.front_stop + 0.05:
                break
            self.commande(0.08, 0.0)
            time.sleep(0.05)
        self.stop()
        return True

    def reculer(self, distance):
        """Recule en ligne droite (odometrie), si le scan filtre ne voit rien derriere et que
        la costmap locale est libre sous l'empreinte."""
        distance = -self.trajet_libre(-distance)
        if distance < 0.05:
            self.log('recul annule : pas de place dans la costmap')
            return False
        p0 = self.pose(self.odom_frame)
        if p0 is None:
            return False
        t0, mur = self.t(), time.time()
        duree = distance / 0.08 + 3.0
        while self.t() - t0 < duree:
            self.verifie()
            if time.time() - mur > 8 * duree:
                # borne en temps reel : horloge simulee ou TF figees (course s740)
                self.log('deplacement en ligne droite arrete : temps depasse (horloge ou TF figees ?)')
                break
            p = self.pose(self.odom_frame)
            if p is None or math.hypot(p[0] - p0[0], p[1] - p0[1]) >= distance:
                break
            arriere = self.secteur(self.scan_filtre, math.pi, math.radians(35))
            if arriere is not None and arriere < 0.35:
                self.log(f'recul arrete : obstacle a {arriere:.2f} m derriere')
                break
            self.commande(-0.08, 0.0)
            time.sleep(0.05)
        self.stop()
        return True

    # ---------- Nav2 ----------
    def relancer_nav2(self):
        """Demande au gestionnaire de Nav2 de redemarrer la navigation (au plus toutes les 15 s)."""
        if self.t() - self.derniere_relance < 15.0:
            return
        self.derniere_relance = self.t()
        if not self.nav_manager.wait_for_service(timeout_sec=2.0):
            self.log('Nav2 refuse les buts et son gestionnaire ne repond pas')
            return
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP
        fut = self.nav_manager.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 30.0:
            time.sleep(0.1)
        ok = fut.done() and fut.result() is not None and fut.result().success
        self.log(f'Nav2 etait desactive : relance {"reussie" if ok else "ratee"}')

    def naviguer(self, x, y, yaw, arret=None, timeout=None):
        """Va a (x, y, yaw) dans map. arret() -> True interrompt le trajet. Renvoie 'ok', 'echec' ou 'arret'."""
        self.degager()
        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.log('Nav2 indisponible')
            return 'echec'
        but = NavigateToPose.Goal()
        but.pose.header.frame_id = self.map_frame
        but.pose.header.stamp = self.get_clock().now().to_msg()
        but.pose.pose.position.x, but.pose.pose.position.y = float(x), float(y)
        but.pose.pose.orientation.z, but.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        fut = self.nav.send_goal_async(but)
        t0 = self.t()
        while not fut.done():
            time.sleep(0.02)
            if self.t() - t0 > 5.0:
                return 'echec'
        handle = fut.result()
        if not handle.accepted:
            # Nav2 refuse tout but quand son gestionnaire l'a desactive (noeud fige) :
            # on le relance, et on ne reessaie pas tout de suite (pas de boucle a vide)
            self.relancer_nav2()
            self.sleep(1.0)
            return 'echec'
        res = handle.get_result_async()
        timeout = timeout or self.nav_timeout
        t0 = self.t()
        while not res.done():
            time.sleep(0.05)
            raison = None
            if self.fini():
                raison = 'fin de course'
            elif arret is not None and arret():
                raison = 'arret'
            elif self.t() - t0 > timeout:
                raison = 'delai'
            if raison:
                c = handle.cancel_goal_async()
                t1 = time.time()
                while not c.done() and time.time() - t1 < 2.0:
                    time.sleep(0.02)
                self.stop()
                self.verifie()
                return 'arret' if raison == 'arret' else 'echec'
        self.stop(0.1)
        return 'ok' if res.result().status == GoalStatus.STATUS_SUCCEEDED else 'echec'

    # ---------- mouvements simples ----------
    def tourner(self, angle, vitesse, arret=None):
        """Tourne sur place de angle (rad) mesure par la TF odom. Renvoie True si arret() a interrompu."""
        p0 = self.pose(self.odom_frame)
        if p0 is None:
            return False
        fait, prec = 0.0, p0[2]
        t0 = self.t()
        while abs(fait) < abs(angle) and self.t() - t0 < abs(angle) / vitesse * 2.5 + 3:
            self.verifie()
            if arret is not None and arret():
                self.stop()
                return True
            # en tournant, les coins du robot balaient un cercle : rien ne doit y entrer
            libre = self.degagement()
            if libre is not None and libre < self.rotation_clearance:
                self.log(f'rotation sur place annulee : obstacle a {libre:.2f} m')
                break
            self.commande(0.0, math.copysign(vitesse, angle))
            time.sleep(0.05)
            p = self.pose(self.odom_frame)
            if p:
                fait += wrap(p[2] - prec)
                prec = p[2]
        self.stop()
        return False

    def regarder_autour(self, arret):
        """Tour complet par pas de 45 deg, avec une pause a chaque pas : image nette pour
        la detection (flou de bouge en rotation continue), et assez de mesures pour
        localiser un tag. Renvoie True si arret() a interrompu."""
        ecarte = False
        for _ in range(8):
            o = self.obstacle_proche()
            if o is not None and o[0] < self.rotation_clearance and not ecarte:
                # obstacle sur le cote (mur, pied de table) : avancer ou reculer n'aide pas,
                # Nav2 nous eloigne de 0.4 m a l'oppose, en manoeuvrant
                ecarte = True
                r = self.pose(self.map_frame)
                if r:
                    d = r[2] + o[1] + math.pi
                    self.log(f'tour d\'observation : obstacle a {o[0]:.2f} m, on s\'en ecarte')
                    if self.naviguer(r[0] + 0.4 * math.cos(d), r[1] + 0.4 * math.sin(d), r[2], arret=arret,
                                     timeout=20.0) == 'arret':
                        return True
                    o = self.obstacle_proche()
            if o is not None and o[0] < self.rotation_clearance:
                self.log(f'tour d\'observation annule : obstacle a {o[0]:.2f} m')
                return False
            if self.tourner(math.pi / 4, self.spin_speed, arret):
                return True
            t0 = self.t()
            while self.t() - t0 < self.look_pause:
                self.verifie()
                if arret():
                    return True
                time.sleep(0.05)
        return False

    # ---------- exploration ----------
    def choisir_point_de_vue(self):
        """Point libre de la carte, loin des points deja vus, proche du robot et pres de l'inconnu."""
        g = self.grid
        robot = self.pose(self.map_frame)
        if g is None or robot is None:
            return None
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = np.asarray(g.data, dtype=np.int16).reshape(h, w)
        occ = np.argwhere(data >= 65)
        unk = np.argwhere(data < 0)
        pas = max(1, int(0.25 / res))
        cand = np.argwhere(np.zeros((h, w), bool)[::pas, ::pas] == False) * pas  # noqa: E712
        cand = cand[(data[cand[:, 0], cand[:, 1]] >= 0) & (data[cand[:, 0], cand[:, 1]] < 25)]
        if cand.size == 0:
            return None
        xy = np.column_stack([ox + (cand[:, 1] + 0.5) * res, oy + (cand[:, 0] + 0.5) * res])
        c = self.costmap
        if c is not None:
            # la costmap de Nav2 contient aussi les obstacles vus par les cameras (plateaux
            # de table) et la marge de securite : un point de vue doit y etre bien libre
            ci = np.floor((xy[:, 0] - c.info.origin.position.x) / c.info.resolution).astype(int)
            cj = np.floor((xy[:, 1] - c.info.origin.position.y) / c.info.resolution).astype(int)
            dedans = (ci >= 0) & (ci < c.info.width) & (cj >= 0) & (cj < c.info.height)
            cout = np.full(len(xy), 100)
            cdata = np.asarray(c.data, dtype=np.int16).reshape(c.info.height, c.info.width)
            cout[dedans] = cdata[cj[dedans], ci[dedans]]
            garde = (cout >= 0) & (cout <= self.explore_max_cost)
            atteignable = self.zone_atteignable(c, cdata, robot)
            if atteignable is not None:
                # des rayons du LiDAR passent par-dessus les murets bas : la carte a des zones
                # libres hors de l'arene, ou aucun chemin ne mene ("no valid path")
                garde &= dedans
                garde[dedans] &= atteignable[cj[dedans], ci[dedans]]
            xy = xy[garde]
            if len(xy) == 0:
                return None
        if occ.size:
            occ_xy = np.column_stack([ox + (occ[:, 1] + 0.5) * res, oy + (occ[:, 0] + 0.5) * res])
            dmin = np.full(len(xy), 10.0)
            for k in range(0, len(occ_xy), 2000):
                d = np.linalg.norm(xy[:, None, :] - occ_xy[None, k:k + 2000, :], axis=2).min(axis=1)
                dmin = np.minimum(dmin, d)
            garde = dmin >= self.explore_clearance
            xy = xy[garde]
        if len(xy) == 0:
            return None
        deja = np.array([v[:2] for v in self.viewpoints] + [robot[:2]])
        loin = np.linalg.norm(xy[:, None, :] - deja[None, :, :], axis=2).min(axis=1)
        garde = loin >= self.explore_spacing
        xy, loin = xy[garde], loin[garde]
        if len(xy) == 0:
            return None
        gain = np.zeros(len(xy))
        if unk.size:
            unk_xy = np.column_stack([ox + (unk[::4, 1] + 0.5) * res, oy + (unk[::4, 0] + 0.5) * res])
            for k in range(0, len(unk_xy), 2000):
                gain += (np.linalg.norm(xy[:, None, :] - unk_xy[None, k:k + 2000, :], axis=2) < 1.0).sum(axis=1)
        trajet = np.linalg.norm(xy - np.array(robot[:2]), axis=1)
        score = np.minimum(loin, 2.0) + 0.02 * np.minimum(gain, 100) - 0.5 * trajet
        k = int(np.argmax(score))
        return float(xy[k, 0]), float(xy[k, 1])

    def zone_atteignable(self, c, cdata, robot):
        """Cellules de la costmap globale reliees au robot sans traverser d'obstacle (remplissage)."""
        if cv2 is None:
            return None
        libre = ((cdata >= 0) & (cdata < 99)).astype(np.uint8)   # 99 : zone inscrite, 100 : obstacle
        i = int((robot[0] - c.info.origin.position.x) / c.info.resolution)
        j = int((robot[1] - c.info.origin.position.y) / c.info.resolution)
        h, w = libre.shape
        # le robot peut etre dans la marge d'un obstacle : on part de la cellule libre la plus proche
        ys, xs = np.nonzero(libre)
        if xs.size == 0:
            return None
        k = int(np.argmin((xs - i) ** 2 + (ys - j) ** 2))
        if (xs[k] - i) ** 2 + (ys[k] - j) ** 2 > (0.6 / c.info.resolution) ** 2:
            return None
        masque = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(libre, masque, (int(xs[k]), int(ys[k])), 2)
        return libre == 2

    def explorer(self, cible, limite):
        """Cherche le tag cible jusqu'au temps limite. Renvoie True si sa position est connue."""
        connu = lambda: cible in self.tags  # noqa: E731
        self.log(f'exploration pour le tag {cible}')
        if not self.viewpoints:
            p = self.pose(self.map_frame)
            if p:
                self.viewpoints.append(p[:2])
            self.regarder_autour(connu)
        while not connu() and self.t() < limite:
            self.verifie()
            pv = self.choisir_point_de_vue()
            if pv is None:
                if len(self.viewpoints) > 1:
                    self.log('carte couverte : on recommence la tournee des points de vue')
                    self.viewpoints = self.viewpoints[-1:]
                    continue
                self.regarder_autour(connu)
                continue
            self.viewpoints.append(pv)
            r = self.pose(self.map_frame)
            cap = math.atan2(pv[1] - r[1], pv[0] - r[0]) if r else 0.0
            self.log(f'point de vue ({pv[0]:.2f}, {pv[1]:.2f})')
            res = self.naviguer(pv[0], pv[1], cap, arret=connu, timeout=min(self.nav_timeout, limite - self.t()))
            if res == 'arret' or connu():
                break
            if res == 'ok':
                self.regarder_autour(connu)
                continue
            # point non atteint : si le robot a quand meme change d'endroit, on regarde autour
            p = self.pose(self.map_frame)
            anciens = self.viewpoints[:-1]
            if p and all(math.hypot(p[0] - v[0], p[1] - v[1]) > 0.5 * self.explore_spacing for v in anciens):
                self.viewpoints.append(p[:2])
                self.regarder_autour(connu)
        return connu()

    # ---------- approche finale ----------
    def patine(self, v):
        """Detecte un robot bloque dont les roues tournent : l'odometrie avance, mais les
        distances LiDAR devant (ou derriere) ne changent pas. L'odometrie ne s'en apercoit
        pas, et la localisation suivrait cette fausse avance (course 15)."""
        if abs(v) < 0.02:
            self.suivi_patinage = []
            return False
        p = self.pose(self.odom_frame)
        s = self.scan if v > 0 else self.scan_filtre
        d = self.secteur(s, 0.0 if v > 0 else math.pi, math.radians(30))
        if p is None or d is None or d >= 9.0:
            return False
        self.suivi_patinage.append((self.t(), p[0], p[1], d))
        t0, x0, y0, d0 = self.suivi_patinage[0]
        if self.t() - t0 < self.stuck_time:
            return False
        self.suivi_patinage.pop(0)
        avance_odom = math.hypot(p[0] - x0, p[1] - y0)
        avance_lidar = abs(d - d0)
        return avance_odom > 0.08 and avance_lidar < 0.3 * avance_odom

    def approche(self, i, limite, beta=0.0):
        """Amene le robot a visit_distance du tag i, face a lui, guide par la camera.
        beta : le point vise est sur la normale du tag tournee de beta (visite en biais)."""
        self.suivi_patinage = []
        fin = min(limite, self.t() + self.approach_timeout)
        but = None          # (gx, gy, tx, ty) dans odom
        mesures = []
        phase = 'aller'
        bloque_depuis = None
        while self.t() < fin:
            self.verifie()
            d = self.tag_direct(i, self.odom_frame)
            if d is not None:
                mesures.append(d)
                mesures = mesures[-6:]
            if mesures:
                m = np.median(np.array(mesures), axis=0)
                n = math.hypot(m[2], m[3])
                ux, uy = tourne(m[2] / n, m[3] / n, beta)
                but = (m[0] + self.visit_dist * ux, m[1] + self.visit_dist * uy, m[0], m[1])
            elif but is None:
                # pas encore vu en direct : position memorisee dans la carte
                tg = self.tags.get(i)
                try:
                    tr = self.tf.lookup_transform(self.odom_frame, self.map_frame, Time())
                except TransformException:
                    tr = None
                if tg is None or tr is None:
                    return False
                c, s = math.cos(yaw_of(tr.transform.rotation)), math.sin(yaw_of(tr.transform.rotation))
                ox, oy = tr.transform.translation.x, tr.transform.translation.y
                tx, ty = ox + c * tg[0] - s * tg[1], oy + s * tg[0] + c * tg[1]
                nx, ny = tourne(c * tg[3] - s * tg[4], s * tg[3] + c * tg[4], beta)
                but =(tx + self.visit_dist * nx, ty + self.visit_dist * ny, tx, ty)
            r = self.pose(self.odom_frame)
            if r is None:
                time.sleep(0.05)
                continue
            gx, gy, tx, ty = but
            rho = math.hypot(gx - r[0], gy - r[1])
            vise = wrap(math.atan2(ty - r[1], tx - r[0]) - r[2])

            if phase == 'aller' and rho < self.pos_tol:
                phase = 'face'
            if phase == 'face' and rho > 2.5 * self.pos_tol:
                phase = 'aller'

            if phase == 'aller':
                alpha = wrap(math.atan2(gy - r[1], gx - r[0]) - r[2])
                if abs(alpha) > math.pi / 2 and rho < 0.35:
                    # but derriere et proche (robot trop pres du tag) : on recule
                    alpha_r = wrap(alpha - math.pi)
                    v = -min(0.08, 0.6 * rho) if abs(alpha_r) < 0.4 else 0.0
                    w = clamp(1.5 * alpha_r, self.max_ang)
                elif abs(alpha) > 0.35:
                    v, w = 0.0, clamp(1.5 * alpha, self.max_ang)
                else:
                    v = min(self.max_lin, max(0.03, 0.6 * rho)) * math.cos(alpha)
                    w = clamp(1.5 * alpha, self.max_ang)
                libre = self.devant()
                raison = None
                if v > 0 and libre is not None and libre < self.front_stop:
                    raison = f'obstacle a {libre:.2f} m devant'
                elif abs(v) > 0.0 and abs(self.trajet_libre(math.copysign(0.06, v))) < 0.05:
                    # l'empreinte toucherait un obstacle de la costmap (cote du robot compris)
                    raison = 'empreinte bloquee dans la costmap'
                if raison:
                    v = 0.0
                    bloque_depuis = bloque_depuis or self.t()
                    if self.t() - bloque_depuis > 4.0:
                        self.log(f'approche tag {i} : {raison}, abandon')
                        self.stop()
                        return False
                else:
                    bloque_depuis = None
                if self.patine(v):
                    self.log(f'approche tag {i} : le robot ne bouge plus malgre la commande, abandon')
                    self.stop()
                    return False
                self.commande(v, w)
            else:
                if abs(vise) > math.radians(2.0):
                    w = clamp(1.5 * vise, 0.4)
                    w = math.copysign(max(abs(w), 0.08), w)
                    self.commande(0.0, w)
                else:
                    self.stop(0.4)
                    if self.visite_valide(i):
                        return True
                    # la mesure en direct contredit le but : on repart avec le but mis a jour
                    phase = 'aller'
            time.sleep(0.05)
        self.stop()
        self.log(f'approche tag {i} : delai depasse')
        return False

    def visite_valide(self, i):
        """Controle final sur la mesure camera en direct : distance et angle."""
        t0 = self.t()
        while self.t() - t0 < 1.0:
            d = self.tag_direct(i, self.base_frame)
            if d is not None:
                dist = math.hypot(d[0], d[1])
                angle = math.atan2(d[1], d[0])
                ok = abs(dist - self.visit_dist) <= self.dist_tol and abs(angle) <= self.face_tol
                self.log(f'controle tag {i} : distance {dist:.3f} m, angle {math.degrees(angle):.1f} deg -> '
                         f'{"OK" if ok else "a corriger"}')
                return ok
            time.sleep(0.05)
        self.log(f'controle tag {i} : tag non vu par la camera')
        return False

    # ---------- visite ----------
    def jpeg(self):
        """Derniere image camera en JPEG (CompressedImage), ou None."""
        m = self.image
        if m is not None and cv2 is not None:
            enc = m.encoding.lower()
            buf = np.frombuffer(bytes(m.data), dtype=np.uint8)
            if enc in ('rgb8', 'bgr8'):
                img = buf.reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
                if enc == 'rgb8':
                    img = img[:, :, ::-1]
            elif enc == 'mono8':
                img = buf.reshape(m.height, m.step)[:, :m.width]
            else:
                img = None
            if img is not None:
                ok, data = cv2.imencode('.jpg', np.ascontiguousarray(img), [cv2.IMWRITE_JPEG_QUALITY, 90])
                if ok:
                    out = CompressedImage()
                    out.header = m.header
                    out.format = 'jpeg'
                    out.data = data.tobytes()
                    return out
        if self.image_c is not None and 'jpeg' in self.image_c.format.lower():
            return self.image_c
        return None

    def signaler(self, i):
        t_arret = self.t()
        self.image = self.image_c = None
        if not self.subs_images:
            # creees a la premiere visite puis gardees : detruire un abonnement pendant que
            # l'executeur tourne peut le bloquer (course s740 : horloge figee, recul sans fin)
            self.subs_images = [
                self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data,
                                         callback_group=self.cb),
                self.create_subscription(CompressedImage, self.image_topic + '/compressed', self.on_image_c,
                                         qos_profile_sensor_data, callback_group=self.cb)]
        self.capture = True
        self.stop(0.5)
        img = None
        t0 = self.t()
        while self.t() - t0 < 3.0:
            img = self.jpeg()
            stamp = Time.from_msg(img.header.stamp).nanoseconds * 1e-9 if img else 0
            if img is not None and stamp >= t_arret and self.vu_recemment(i, 1.0):
                break
            self.commande(0.0, 0.0)
            time.sleep(0.05)
        self.capture = False
        v = TagVisit()
        v.header.stamp = self.get_clock().now().to_msg()
        v.tag_id = int(i)
        if img is not None:
            v.header.frame_id = img.header.frame_id
            v.image = img
        else:
            self.log('ATTENTION : aucune image JPEG disponible pour la visite')
        self.visit_pub.publish(v)
        self.log(f'VISITE signalee : tag {i}')
        self.stop(self.hold_time)
        # a 0.5 m du tag, l'avant du robot touche la zone inscrite du poteau : Nav2 se croirait
        # en collision et refuserait de repartir. On s'ecarte d'abord en ligne droite.
        self.reculer(self.backup_after_visit)

    # ---------- course ----------
    def attendre_depart(self):
        self.log('attente de l\'ordre des tags')
        while self.order is None:
            time.sleep(0.05)
        self.log('attente du depart (mot sur /ma64/start_vocal ou message sur /ma64/start)')
        while True:
            if self.start_word is not None:
                armee = self.status is None or self.status.state == RunStatus.ARMED
                if armee:
                    break
                self.log('depart ignore : course pas armee')
                self.start_word = None
            time.sleep(0.02)
        s = StartSignal()
        s.stamp = self.get_clock().now().to_msg()
        if self.start_vocal:
            s.method = StartSignal.METHOD_VOICE
            s.heard = self.start_word
        else:
            s.method = StartSignal.METHOD_COMMAND
        self.start_pub.publish(s)
        self.t_depart = self.t()
        self.started = True
        self.log(f'DEPART publie (vocal : "{s.heard}")' if self.start_vocal else 'DEPART publie (commande)')
        if self.wait_running and self.status is not None:
            t0 = self.t()
            while self.status.state != RunStatus.RUNNING and self.t() - t0 < 3.0:
                time.sleep(0.02)

    def budget(self, restants):
        reste = self.temps_restant() - 5.0
        return min(reste, max(self.tag_budget_min, reste / max(restants, 1) * 1.5))

    def aller_a_l_indice(self, i, limite):
        """Va voir la ou le tag i etait pendant la cartographie. Renvoie True s'il y est vu."""
        x, y, nx, ny = self.indices[i]
        connu = lambda: i in self.tags  # noqa: E731
        self.log(f'tag {i} : indice de cartographie ({x:.2f}, {y:.2f})')
        self.naviguer(x + self.view_dist * nx, y + self.view_dist * ny, math.atan2(-ny, -nx), arret=connu,
                      timeout=min(self.nav_timeout, limite - self.t()))
        if not connu():
            # l'indice peut etre decale de quelques degres : on balaie autour
            if not (self.tourner(0.8, 0.35, connu) or self.tourner(-1.6, 0.35, connu)):
                self.tourner(0.8, 0.35, connu)
        return connu()

    def visiter(self, i, limite):
        if i not in self.tags and i in self.indices:
            self.aller_a_l_indice(i, limite)
        absences = 0
        while self.t() < limite:
            self.verifie()
            if i not in self.tags and not self.explorer(i, limite):
                return False
            tx, ty, _, nx, ny = self.tags[i]
            # face au tag n'impose pas d'etre dans son axe : si une table bouche l'axe, on se
            # place en biais (le robot regarde quand meme le tag, cap a +-12 deg)
            beta = self.angle_de_visite(tx, ty, nx, ny)
            if beta:
                self.log(f'tag {i} : axe encombre, visite en biais de {math.degrees(beta):.0f} deg')
            nx, ny = tourne(nx, ny, beta)
            for dist in (self.view_dist, 0.8 * self.view_dist):
                vx, vy = tx + dist * nx, ty + dist * ny
                self.log(f'tag {i} : vers le point de vue ({vx:.2f}, {vy:.2f})')
                nav = self.naviguer(vx, vy, math.atan2(-ny, -nx), timeout=min(self.nav_timeout, limite - self.t()))
                if nav == 'ok':
                    break
            vu = lambda: self.vu_recemment(i, 0.3)  # noqa: E731
            r = self.pose(self.map_frame)
            if not self.vu_recemment(i, 1.0) and r:
                # Nav2 peut finir sans le bon cap (tolerance de lacet) : on se tourne vers le tag
                ecart = wrap(math.atan2(ty - r[1], tx - r[0]) - r[2])
                if abs(ecart) > 0.3:
                    self.tourner(ecart, 0.5, vu)
            if not self.vu_recemment(i, 1.0):
                # petit balayage pour retrouver le tag dans l'image
                if not (self.tourner(0.6, 0.35, vu) or self.tourner(-1.2, 0.35, vu)):
                    self.tourner(0.6, 0.35, vu)
            r = self.pose(self.map_frame)
            if not self.vu_recemment(i, 2.0) and r and math.hypot(r[0] - tx, r[1] - ty) < 1.6 * self.view_dist:
                # au point de vue, le tag n'est pas la : sa position memorisee est fausse (detection
                # mal placee sur la carte). On l'oublie et on repart explorer. Si Nav2 n'a pas atteint
                # le point de vue, le robot ne fait peut-etre pas face au tag : on reessaie d'abord.
                absences += 1
                etat = (f'robot ({r[0]:.2f}, {r[1]:.2f}, {math.degrees(r[2]):.0f} deg), navigation {nav}')
                if nav != 'ok' and absences < 2:
                    self.log(f'tag {i} pas vu depuis le point de vue, {etat} : nouvel essai')
                    continue
                self.log(f'tag {i} absent de sa position memorisee ({tx:.2f}, {ty:.2f}), {etat} : on l\'oublie')
                self.tags.pop(i, None)
                self.indices.pop(i, None)
                absences = 0
                continue
            if r and math.hypot(r[0] - tx, r[1] - ty) > 1.6 * self.view_dist:
                # Nav2 n'a pas atteint le point de vue : l'approche finale ne planifie pas de
                # chemin, on ne la lance pas de loin
                self.log(f'tag {i} : trop loin ({math.hypot(r[0] - tx, r[1] - ty):.1f} m) pour l\'approche')
                self.sleep(1.0)
                continue
            if self.approche(i, limite, beta):
                self.signaler(i)
                return True
            self.log(f'tag {i} : approche ratee, nouvel essai')
        return False

    def cout_global(self, x, y):
        """Cout de la costmap globale au point (x, y) de map (-1 : inconnu ou hors carte)."""
        c = self.costmap
        if c is None:
            return -1
        i = int((x - c.info.origin.position.x) / c.info.resolution)
        j = int((y - c.info.origin.position.y) / c.info.resolution)
        if not (0 <= i < c.info.width and 0 <= j < c.info.height):
            return -1
        return c.data[j * c.info.width + i]

    def angle_de_visite(self, tx, ty, nx, ny):
        """Angle (rad) dont tourner la normale du tag pour que le point de visite et le point de vue
        soient libres dans la costmap globale (99 : robot en collision, 100 : obstacle ou masque).
        0 si l'axe est libre ou si aucun angle ne l'est."""
        for deg in (0, 25, -25, 40, -40, 50, -50):
            ux, uy = tourne(nx, ny, math.radians(deg))
            if all(self.cout_global(tx + d * ux, ty + d * uy) < 99 for d in (self.visit_dist, self.view_dist)):
                return math.radians(deg)
        return 0.0

    def degagement_derriere(self):
        """Distance mediane derriere le robot (+-15 deg, scan brut) : grandit quand il avance."""
        s = self.scan
        if s is None or self.laser_yaw is None:
            self.devant()
            return None
        r = np.asarray(s.ranges, dtype=np.float32)
        a = s.angle_min + np.arange(r.size) * s.angle_increment + self.laser_yaw - math.pi
        a = np.arctan2(np.sin(a), np.cos(a))
        ok = np.isfinite(r) & (r > max(s.range_min, 0.12)) & (np.abs(a) < math.radians(15))
        return float(np.median(r[ok])) if ok.any() else None

    def ouverture(self):
        """Avance tout droit de ouverture_avance (seul le scan devant arrete), puis tourne."""
        d = self.ouverture_avance
        self.log(f'ouverture : avance de {d:.2f} m puis virage de {self.ouverture_virage:.0f} deg')
        p0 = self.pose(self.odom_frame)
        if p0 is None:
            self.log('ouverture annulee : pas de pose odom')
            return
        t0, mur = self.t(), time.time()
        duree = d / 0.10 + 5.0
        scan0 = self.degagement_derriere()
        while self.t() - t0 < duree and time.time() - mur < 3 * duree:
            self.verifie()
            p = self.pose(self.odom_frame)
            if p is None or math.hypot(p[0] - p0[0], p[1] - p0[1]) >= d:
                break
            # 07/11 : odometrie +4 m, LiDAR fige (moteurs coupes ?). Derriere doit s'eloigner.
            fait = math.hypot(p[0] - p0[0], p[1] - p0[1])
            scan1 = self.degagement_derriere()
            if fait > 0.5 and scan0 and scan1 and abs(scan1 - scan0) < 0.10:
                self.get_logger().error(f'ROBOT IMMOBILE : odometrie +{fait:.2f} m mais le LiDAR ne voit aucun '
                                        f'deplacement ({scan0:.2f} -> {scan1:.2f} m). Moteurs ? arret d\'urgence ?')
                break
            libre = self.devant() or 10.0
            if libre < self.front_stop + 0.05:
                self.log(f'ouverture : obstacle devant a {libre:.2f} m, on s\'arrete')
                break
            self.commande(0.10, 0.0)
            time.sleep(0.05)
        self.stop()
        p = self.pose(self.odom_frame)
        if p:
            self.log(f'ouverture : avance faite {math.hypot(p[0] - p0[0], p[1] - p0[1]):.2f} m')
        if abs(self.ouverture_virage) > 1:
            self.tourner(math.radians(self.ouverture_virage), self.spin_speed)
            self.stop()

    def courir(self):
        self.attendre_depart()
        if self.ouverture_avance > 0:
            self.ouverture()
        ordre = list(self.order)
        signales = []
        for k, i in enumerate(ordre):
            self.verifie()
            limite = self.t() + self.budget(len(ordre) - k)
            self.log(f'=== tag {i} ({k + 1}/{len(ordre)}), budget {limite - self.t():.0f} s')
            if self.visiter(i, limite):
                signales.append(i)
            else:
                self.log(f'tag {i} SAUTE')
        if not self.fini() and (not signales or signales[-1] != ordre[-1]):
            f = Finish()
            f.stamp = self.get_clock().now().to_msg()
            f.reason = Finish.REASON_GAVE_UP
            self.finish_pub.publish(f)
            self.log('Finish publie (dernier tag non signale)')
        self.log(f'fin de mission, tags signales : {signales}')

    def run(self):
        try:
            if self.indices:
                self.log(f'indices de cartographie charges pour les tags {sorted(self.indices)}')
            if self.test_approche:
                self.t_depart = self.t()
                self.sleep(2.0)
                self.log(f'test d\'approche du tag {self.test_approche}')
                ok = self.approche(self.test_approche, self.t() + self.approach_timeout)
                self.log(f'test d\'approche : {"reussi" if ok else "rate"}')
            else:
                self.courir()
        except CourseFinie:
            self.log('course terminee par l\'arbitre')
        finally:
            try:
                self.stop(0.5)
            except Exception:  # noqa: BLE001  contexte ROS deja ferme (Ctrl+C)
                pass
            self.ecrire_journal()

    def ecrire_journal(self):
        if not self.journal_dir:
            return
        os.makedirs(self.journal_dir, exist_ok=True)
        nom = os.path.join(self.journal_dir, time.strftime('mission_%Y%m%d_%H%M%S.json'))
        with open(nom, 'w') as f:
            json.dump({'ordre': self.order, 'tags_carte': self.tags, 'journal': self.journal}, f,
                      indent=1, ensure_ascii=False)
        self.get_logger().info(f'journal ecrit : {nom}')


def main():
    # Sans le gestionnaire de rclpy, Ctrl+C leve KeyboardInterrupt dans la boucle de mission :
    # le robot est arrete et le journal ecrit. Avec lui, le contexte etait ferme sans
    # interrompre la boucle, qui tournait jusqu'au SIGKILL de ros2 launch.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Mission()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)

    def tourner_executeur():
        try:
            ex.spin()
        except Exception:  # noqa: BLE001  contexte ferme pendant l'attente
            pass

    th = threading.Thread(target=tourner_executeur, daemon=True)
    th.start()
    try:
        node.run()
        # reste actif (sans rien publier) pour que les journaux restent lisibles
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop(0.2)
        except Exception:  # noqa: BLE001  contexte deja ferme par Ctrl+C
            pass
        # ros2 launch tue le noeud 5 s apres Ctrl+C : on ne doit pas attendre plus
        ex.shutdown(timeout_sec=1.0)
        th.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
