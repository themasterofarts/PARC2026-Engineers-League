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

import collections
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
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

from nav_solution.couverture import Couverture

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
        # tournee : cercle de points autour du centre de l'arene, camera tournee vers les murs
        self.tournee_active = p('explore_tournee', True).value
        self.tournee_n = p('tournee_points', 8).value
        self.tournee_marge = p('tournee_marge_mur', 1.0).value   # distance cercle -> bord de l'arene (m)
        self.tournee_pts = None   # [(x, y, cap vers l'exterieur)] dans map, calcule une fois
        self.tournee_k = 0        # prochain point de la tournee
        # couverture : surfaces deja regardees par la camera (exploration apres la tournee)
        self.couv = Couverture(portee=p('couverture_portee', 4.0).value,          # = max_distance des tags
                               angle_max_deg=p('couverture_angle_max_deg', 55.0).value,
                               hfov_deg=p('camera_hfov_deg', 55.0).value)       # camera2 du robot : 55 deg
        self.couv_lock = threading.Lock()
        self.couv_carte = None    # (message de carte, temps) de la derniere mise a jour des surfaces
        self.couv_cap = None
        self.trace = collections.deque(maxlen=80)   # positions passees (odom), tous les 8 cm
        self.couv_rates = []      # points de couverture que Nav2 n'a pas atteints : on n'y retourne pas
        # masque automatique des tables (groupes de petits obstacles du LiDAR), sauf masque dessine
        self.masque_auto = p('masque_auto', True).value
        self.masque_carte = None
        self.masque_tables = -1
        self.tables = []          # tables memorisees : listes de points (map) ; jamais oubliees
        self.arene = None         # bords de l'arene (x0, x1, y0, y1) mesures par la tournee
        self.ilots_cache = (None, [])                # (carte, centres des petits obstacles isoles)
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
        # rotation sur place : le robot pivote autour de l'axe des roues (URDF : x = +0.095 dans
        # base_footprint) et l'arriere du chassis balaie un cercle de 0.44 m autour de cet axe
        self.pivot_x = p('pivot_x', 0.095).value
        self.rayon_balayage = p('rayon_balayage', 0.46).value
        self.degagement_min = p('degagement_min', 0.30).value   # plus pres : on s'ecarte avant de partir
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
        self.costmap_np = None     # (message, tableau numpy) de la costmap globale
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
        self.create_timer(0.5, self.maj_couverture, callback_group=self.cb)
        if self.masque_auto:
            self.pub_info = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info', LATCHED)
            self.pub_masque = self.create_publisher(OccupancyGrid, '/keepout_filter_mask', LATCHED)
            self.create_timer(2.0, self.maj_masque_auto, callback_group=self.cb)

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
        # message et tableau ensemble : la carte peut changer de taille entre deux lectures
        self.costmap_np = (msg, np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width))
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
        """Distance au plus proche obstacle tout autour : scan filtre (sans le chassis) et costmap
        globale (masque compris : plateaux et assises que le LiDAR ne voit pas)."""
        o = self.obstacle_proche()
        return None if o is None else o[0]

    def points_obstacles(self, rayon=1.0):
        """Obstacles a moins de rayon, dans le repere du robot (N x 2) : scan filtre et cases obstacle
        ou masque (cout 100) de la costmap globale."""
        pts = []
        self.devant()   # calcule laser_yaw
        s = self.scan_filtre
        if s is not None and self.laser_yaw is not None:
            r = np.asarray(s.ranges, dtype=np.float32)
            a = s.angle_min + np.arange(r.size) * s.angle_increment + self.laser_yaw
            ok = np.isfinite(r) & (r > s.range_min) & (r < rayon)
            pts.append(np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]))
        p = self.pose(self.map_frame)
        if self.costmap_np is not None and p is not None:
            c, cdata = self.costmap_np
            res, ox, oy = c.info.resolution, c.info.origin.position.x, c.info.origin.position.y
            n = int(rayon / res) + 1
            i, j = int((p[0] - ox) / res), int((p[1] - oy) / res)
            i0, i1, j0, j1 = max(0, i - n), min(c.info.width, i + n + 1), max(0, j - n), min(c.info.height, j + n + 1)
            if i0 < i1 and j0 < j1:
                jj, ii = np.nonzero(cdata[j0:j1, i0:i1] >= 100)
                dx = ox + (ii + i0 + 0.5) * res - p[0]
                dy = oy + (jj + j0 + 0.5) * res - p[1]
                cy, sy = math.cos(-p[2]), math.sin(-p[2])
                pts.append(np.column_stack([cy * dx - sy * dy, sy * dx + cy * dy]))
        return np.concatenate(pts) if pts else np.zeros((0, 2))

    def place_rotation(self):
        """Distance (m) entre l'axe des roues et l'obstacle le plus proche : il faut plus que
        rayon_balayage pour tourner sur place sans toucher. 10.0 si rien autour."""
        pts = self.points_obstacles(self.rayon_balayage + 0.3)
        if len(pts) == 0:
            return 10.0
        return float(np.hypot(pts[:, 0] - self.pivot_x, pts[:, 1]).min())

    def obstacle_costmap(self, rayon=1.0):
        """(distance, direction dans le repere du robot) de la case obstacle ou masque (cout 100) la
        plus proche dans la costmap globale, a moins de rayon, ou None. Le LiDAR ne voit que les pieds
        des tables : en tournant pres d'elles, le chassis haut touchait les plateaux (course tournee21)."""
        r = self.pose(self.map_frame)
        if self.costmap_np is None or r is None:
            return None
        c, cdata = self.costmap_np
        res, ox, oy = c.info.resolution, c.info.origin.position.x, c.info.origin.position.y
        n = int(rayon / res) + 1
        i, j = int((r[0] - ox) / res), int((r[1] - oy) / res)
        i0, i1 = max(0, i - n), min(c.info.width, i + n + 1)
        j0, j1 = max(0, j - n), min(c.info.height, j + n + 1)
        if i0 >= i1 or j0 >= j1:
            return None
        jj, ii = np.nonzero(cdata[j0:j1, i0:i1] >= 100)
        if ii.size == 0:
            return None
        x = ox + (ii + i0 + 0.5) * res - r[0]
        y = oy + (jj + j0 + 0.5) * res - r[1]
        d = np.hypot(x, y)
        k = int(np.argmin(d))
        if d[k] > rayon:
            return None
        return max(0.0, float(d[k]) - res / 2), wrap(math.atan2(y[k], x[k]) - r[2])

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
        """(distance, direction dans le repere du robot) de l'obstacle le plus proche, scan filtre
        et costmap globale (masque compris), ou None."""
        self.devant()  # calcule l'orientation du LiDAR dans le repere du robot (laser_yaw)
        s = self.scan_filtre
        if s is None or self.laser_yaw is None:
            return None
        r = np.asarray(s.ranges, dtype=np.float32)
        r = np.where(np.isfinite(r) & (r > s.range_min), r, np.inf)
        k = int(np.argmin(r))
        lidar = (float(r[k]) if np.isfinite(r[k]) else 10.0,
                 wrap(s.angle_min + k * s.angle_increment + self.laser_yaw))
        cm = self.obstacle_costmap()
        return cm if cm is not None and cm[0] < lidar[0] else lidar

    def degager(self):
        """Colle a un obstacle, Nav2 se croit en collision et ne repart pas : on s'ecarte
        d'abord en ligne droite, a l'oppose de l'obstacle le plus proche."""
        o = self.obstacle_proche()
        if o is None or o[0] >= self.degagement_min:
            return
        libre, a = o
        self.log(f'obstacle a {libre:.2f} m ({math.degrees(a):.0f} deg) : on se degage')
        bouge = self.reculer(0.25) if abs(a) < math.pi / 2 else self.avancer(0.2)
        if bouge is False:
            # enferme dans la costmap (plateau de table vu trop tard, autour du robot) : on refait
            # a reculons le chemin par lequel on est arrive, qui etait libre
            self.retracer(0.6)

    def retracer(self, distance):
        """Recule le long de la trace des positions passees (repere odom), sur distance metres,
        en ne surveillant que le LiDAR arriere : la costmap croit le robot enferme."""
        trace = list(self.trace)
        p = self.pose(self.odom_frame)
        if p is None or len(trace) < 2:
            return False
        self.log(f'sortie de secours : on refait le chemin a reculons ({distance:.1f} m)')
        cibles, fait, prec = [], 0.0, (p[0], p[1])
        for x, y in reversed(trace):
            d = math.hypot(x - prec[0], y - prec[1])
            if d < 0.05:
                continue
            fait += d
            cibles.append((x, y))
            prec = (x, y)
            if fait >= distance:
                break
        t0, mur = self.t(), time.time()
        for cx, cy in cibles:
            while self.t() - t0 < 40.0 and time.time() - mur < 120.0:
                self.verifie()
                p = self.pose(self.odom_frame)
                if p is None:
                    break
                if math.hypot(cx - p[0], cy - p[1]) < 0.06:
                    break
                arriere = self.secteur(self.scan_filtre, math.pi, math.radians(35))
                if arriere is not None and arriere < 0.30:
                    self.log(f'sortie de secours arretee : obstacle a {arriere:.2f} m derriere')
                    self.stop()
                    return False
                # l'arriere du robot (cap + pi) vise le point : poursuite en marche arriere
                alpha = wrap(math.atan2(cy - p[1], cx - p[0]) - (p[2] + math.pi))
                if abs(alpha) > 0.6:
                    self.commande(0.0, clamp(1.2 * alpha, 0.4))
                else:
                    self.commande(-0.07, clamp(1.2 * alpha, 0.4))
                time.sleep(0.05)
        self.stop()
        return True

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
            return
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
            return
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
            # en tournant, l'arriere du robot balaie un cercle autour de l'axe des roues
            place = self.place_rotation()
            if place < self.rayon_balayage:
                self.log(f'rotation sur place annulee : obstacle a {place:.2f} m de l\'axe des roues')
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
            if o is not None and self.place_rotation() < self.rayon_balayage and not ecarte:
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
            if self.place_rotation() < self.rayon_balayage:
                self.log(f'tour d\'observation annule : obstacle a {self.place_rotation():.2f} m de l\'axe des roues')
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

    def zone_atteignable(self, c, cdata, robot, inconnu=False):
        """Cellules de la costmap globale reliees au robot sans traverser d'obstacle (remplissage).
        inconnu=True : les cases encore inconnues sont traversables (un mur connu les separe quand
        meme de l'exterieur de l'arene)."""
        if cv2 is None:
            return None
        libre = (((cdata >= 0) | inconnu) & (cdata < 99)).astype(np.uint8)   # 99 : zone inscrite, 100 : obstacle
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

    def ilots(self):
        """Centres (map) des petits obstacles isoles de la carte SLAM : colonne de table, pieds de
        chaise, poteau. Le LiDAR ne voit que ca d'une table : on garde les points de passage a
        distance (course seed 135 : un point de tournee entre une table et une poubelle)."""
        g = self.grid
        if g is None or cv2 is None:
            return []
        if self.ilots_cache[0] is g:
            return self.ilots_cache[1]
        occ = (np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width) >= 65).astype(np.uint8)
        n, etiquettes, stats, centres = cv2.connectedComponentsWithStats(occ, connectivity=8)
        res, ox, oy = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y
        petit = max(2, int(0.25 / res))     # boite englobante de moins de 25 cm
        petits = [k for k in range(1, n)
                  if stats[k, cv2.CC_STAT_WIDTH] <= petit and stats[k, cv2.CC_STAT_HEIGHT] <= petit]
        # un ilot a moins de 30 cm d'un grand obstacle continu (mur, meuble) est un morceau de ce mur,
        # vu en pointilles de loin, pas un pied de table (seed 135 : faux groupes dans les coins)
        grand = (occ > 0) & ~np.isin(etiquettes, petits)
        loin = cv2.distanceTransform((~grand).astype(np.uint8), cv2.DIST_L2, 3) * res
        pts = [(ox + (centres[k][0] + 0.5) * res, oy + (centres[k][1] + 0.5) * res) for k in petits
               if loin[int(centres[k][1]), int(centres[k][0])] > 0.30]
        self.ilots_cache = (g, pts)
        return pts

    def maj_masque_auto(self):
        """Masque keepout construit pendant la course a partir du LiDAR : une table avec ses chaises
        apparait sur la carte comme un groupe d'au moins 3 petits ilots (colonne, pieds) a moins de
        0.9 m les uns des autres. On interdit l'enveloppe du groupe, plus un disque de 0.45 m autour
        de chaque ilot (plateau, assises), plus 0.15 m. Un poteau de tag reste un ilot seul : il
        n'est pas masque. Nav2 contourne alors les tables sur tous ses trajets (seed 135 : chemin
        entre une table et une poubelle)."""
        g = self.grid
        if g is None or cv2 is None or self.masque_carte is g:
            return
        self.masque_carte = g
        tags = [(v[0], v[1]) for v in list(self.tags.values())]
        pts = [pt for pt in self.ilots() if all(math.hypot(pt[0] - tx, pt[1] - ty) > 0.4 for tx, ty in tags)]
        groupe = list(range(len(pts)))

        def racine(k):
            while groupe[k] != k:
                groupe[k] = groupe[groupe[k]]
                k = groupe[k]
            return k
        for a in range(len(pts)):
            for b in range(a + 1, len(pts)):
                if math.hypot(pts[a][0] - pts[b][0], pts[a][1] - pts[b][1]) < 0.9:
                    groupe[racine(a)] = racine(b)
        groupes = {}
        for k in range(len(pts)):
            groupes.setdefault(racine(k), []).append(pts[k])
        # memoire : les pieds font une ou deux cases et le SLAM les efface parfois ; une table vue
        # une fois reste masquee toute la course (seed 135 : la table centrale clignotait)
        for v in groupes.values():
            if len(v) < 3 or not self.forme_de_table(v):
                continue
            cx, cy = np.mean([x for x, _ in v]), np.mean([y for _, y in v])
            for t in self.tables:
                if math.hypot(cx - np.mean([x for x, _ in t]), cy - np.mean([y for _, y in t])) < 0.9:
                    t.extend(pt for pt in v if all(math.hypot(pt[0] - q[0], pt[1] - q[1]) > 0.05 for q in t))
                    break
            else:
                self.tables.append(list(v))
        # pas de table contre un mur : un mur vu de loin en pointilles imite des pieds, et un faux
        # masque pres d'un tag empecherait sa visite (seed 135 : faux groupes pres des tags 1 et 9)
        tables = self.tables
        if self.arene is not None:
            x0, x1, y0, y1 = self.arene
            tables = [t for t in tables
                      if x0 + 0.4 < np.mean([x for x, _ in t]) < x1 - 0.4
                      and y0 + 0.4 < np.mean([y for _, y in t]) < y1 - 0.4]
        res, ox, oy = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y
        m = np.zeros((g.info.height, g.info.width), np.uint8)
        for t in tables:
            px = np.array([[(x - ox) / res, (y - oy) / res] for x, y in t], np.int32)
            cv2.fillConvexPoly(m, cv2.convexHull(px), 1)
            for cx, cy in px:
                cv2.circle(m, (int(cx), int(cy)), int(0.45 / res), 1, -1)
        if tables:
            r = max(1, int(0.15 / res))
            m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
        grille = OccupancyGrid()
        grille.header.stamp = self.get_clock().now().to_msg()
        grille.header.frame_id = g.header.frame_id or self.map_frame
        grille.info = g.info
        grille.data = (m.astype(np.int8) * 100).ravel().tolist()
        info = CostmapFilterInfo()
        info.header = grille.header
        info.type = 0
        info.filter_mask_topic = '/keepout_filter_mask'
        info.base, info.multiplier = 0.0, 1.0
        self.pub_info.publish(info)
        self.pub_masque.publish(grille)
        if len(tables) != self.masque_tables:
            self.masque_tables = len(tables)
            self.log(f'masque auto : {len(tables)} table(s) ' +
                     ' '.join(f'({np.mean([x for x, _ in t]):.1f}, {np.mean([y for _, y in t]):.1f})' for t in tables))

    @staticmethod
    def forme_de_table(pts):
        """Une table et ses chaises forment un ensemble en 2D de moins de 1.6 m ; un mur vu de loin
        et en biais apparait en pointilles : des ilots alignes (seed 135, faux groupes le long des murs)."""
        a = np.array(pts)
        etendue = np.ptp(a, axis=0)
        if max(etendue) > 1.6:
            return False
        if len(a) < 3:
            return False
        valeurs = np.linalg.eigvalsh(np.cov((a - a.mean(axis=0)).T))
        largeur, longueur = math.sqrt(max(valeurs[0], 0.0)), math.sqrt(max(valeurs[1], 0.0))
        return not (largeur < 0.08 and longueur > 0.3)   # ecart-type de 0.3 m : ilots alignes sur ~1 m

    def loin_des_ilots(self, x, y, marge=1.0):
        # coin du plateau a 0.65 m de la colonne + arriere du robot a 0.31 m de son centre
        return all(math.hypot(x - ix, y - iy) >= marge for ix, iy in self.ilots())

    def points_tournee(self):
        """Cercle de tournee_n points autour du centre de l'arene (zone atteignable de la costmap
        globale, sans les rayons passes par-dessus les murets), a tournee_marge des bords, dans
        l'ordre du cercle en partant du point le plus proche du robot. Un point encombre est
        rapproche du centre, ou abandonne."""
        c, robot = self.costmap, self.pose(self.map_frame)
        if c is None or robot is None:
            return []
        cdata = np.asarray(c.data, dtype=np.int16).reshape(c.info.height, c.info.width)
        # cases inconnues traversables : juste apres le tour de depart, la carte n'est pas finie et la
        # zone connue seule donnait une arene de 1.9 x 2.5 m au lieu de 6 x 7 ; un mur connu la borne
        atteignable = self.zone_atteignable(c, cdata, robot, inconnu=True)
        if atteignable is None:
            return []
        ys, xs = np.nonzero(atteignable)
        if xs.size < 100:
            return []
        res, ox, oy = c.info.resolution, c.info.origin.position.x, c.info.origin.position.y
        x0, x1 = np.percentile(ox + (xs + 0.5) * res, [2, 98])
        y0, y1 = np.percentile(oy + (ys + 0.5) * res, [2, 98])
        self.arene = (x0, x1, y0, y1)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        rayon = min(2.5, max(0.7, min(x1 - x0, y1 - y0) / 2 - self.tournee_marge))
        depart = math.atan2(robot[1] - cy, robot[0] - cx)
        pts = []
        for k in range(self.tournee_n):
            a = depart + 2 * math.pi * k / self.tournee_n
            for r in (rayon, 0.75 * rayon, 0.5 * rayon):
                x, y = cx + r * math.cos(a), cy + r * math.sin(a)
                i, j = int((x - ox) / res), int((y - oy) / res)
                if 0 <= i < c.info.width and 0 <= j < c.info.height and atteignable[j, i] \
                        and cdata[j, i] <= self.explore_max_cost and self.loin_des_ilots(x, y):
                    pts.append((x, y, a))
                    break
        self.log(f'tournee : centre ({cx:.2f}, {cy:.2f}), arene {x1 - x0:.1f} x {y1 - y0:.1f} m, '
                 f'rayon {rayon:.2f} m, {len(pts)} points')
        return pts

    def tournee(self, connu, limite):
        """Fait le tour de l'arene (points_tournee) jusqu'a connaitre le tag. A chaque point, la
        camera regarde vers l'exterieur (les murs, ou sont les tags) et balaie +-45 deg. Tous les
        tags vus en route sont memorises. Renvoie True si le tag est connu."""
        if self.tournee_pts is None:
            self.tournee_pts = self.points_tournee()
        while self.tournee_k < len(self.tournee_pts) and not connu() and self.t() < limite:
            self.verifie()
            x, y, cap = self.tournee_pts[self.tournee_k]
            self.tournee_k += 1
            self.viewpoints.append((x, y))
            self.log(f'tournee {self.tournee_k}/{len(self.tournee_pts)} : ({x:.2f}, {y:.2f})')
            if self.naviguer(x, y, cap, arret=connu, timeout=min(self.nav_timeout, limite - self.t())) == 'arret':
                break
            r = self.pose(self.map_frame)
            if r and abs(wrap(cap - r[2])) > 0.3 and self.tourner(wrap(cap - r[2]), self.spin_speed, connu):
                break
            # vers le mur (+-45 deg), puis vers le centre : les tags sur poteaux y sont souvent
            for angle in (math.pi / 4, -math.pi / 2, math.pi / 4 + math.pi):
                if connu() or self.tourner(angle, self.spin_speed, connu) or self.attendre_vue(connu):
                    break
        return connu()

    def attendre_vue(self, arret):
        """Pause immobile (image nette pour la detection). Renvoie True si arret() a interrompu."""
        t0 = self.t()
        while self.t() - t0 < self.look_pause:
            self.verifie()
            if arret():
                return True
            time.sleep(0.05)
        return False

    # ---------- couverture camera ----------
    def zone_accessible(self, inconnu=False):
        """Fonction (x, y) -> bool des points atteignables par le robot (costmap globale), ou None."""
        r = self.pose(self.map_frame)
        if self.costmap_np is None or r is None:
            return None
        c, cdata = self.costmap_np
        atteignable = self.zone_atteignable(c, cdata, r, inconnu)
        if atteignable is None:
            return None
        res, ox, oy = c.info.resolution, c.info.origin.position.x, c.info.origin.position.y
        h, w = atteignable.shape

        def accessible(x, y):
            i = np.floor((np.asarray(x) - ox) / res).astype(int)
            j = np.floor((np.asarray(y) - oy) / res).astype(int)
            dedans = (i >= 0) & (i < w) & (j >= 0) & (j < h)
            out = np.zeros(i.shape, bool)
            out[dedans] = atteignable[j[dedans], i[dedans]]
            return out
        return accessible

    def maj_couverture(self):
        """Timer : trace des positions, surfaces de la carte (toutes les 3 s au plus) et ce que la
        camera voit maintenant. Rien n'est marque pendant une rotation rapide (image floue)."""
        if not self.started:
            return
        po = self.pose(self.odom_frame)
        if po and (not self.trace or math.hypot(po[0] - self.trace[-1][0], po[1] - self.trace[-1][1]) > 0.08):
            self.trace.append(po[:2])
        if self.grid is None:
            return
        r = self.pose(self.map_frame)
        if r is None:
            return
        try:
            with self.couv_lock:
                g = self.grid
                if self.couv_carte is None or (self.couv_carte[0] is not g and self.t() - self.couv_carte[1] > 3.0):
                    acc = self.zone_accessible()
                    if acc is not None:
                        data = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
                        self.couv.maj_carte(data, g.info.resolution, g.info.origin.position.x,
                                            g.info.origin.position.y, acc)
                        self.couv_carte = (g, self.t())
                if self.couv_cap is not None and abs(wrap(r[2] - self.couv_cap)) < 0.25:
                    self.couv.observer(r[0], r[1], r[2])
                self.couv_cap = r[2]
        except Exception as e:  # noqa: BLE001  la couverture ne doit jamais arreter la mission
            self.get_logger().warn(f'couverture : {e}')

    def chercher_par_couverture(self, connu, limite):
        """Va vers le point qui voit le plus de surfaces encore jamais regardees, s'y tourne vers elles
        et balaie +-30 deg. Quand tout a ete vu sans trouver le tag, une detection a ete ratee : on
        remet la couverture a zero. Renvoie True (tag connu), False (temps ecoule) ou None
        (couverture indisponible : l'appelant reprend l'exploration classique)."""
        remises = 0
        while not connu() and self.t() < limite:
            self.verifie()
            r = self.pose(self.map_frame)
            if self.costmap_np is None or r is None:
                return None
            c, cdata = self.costmap_np
            atteignable = self.zone_atteignable(c, cdata, r)
            if atteignable is None:
                return None
            res, ox, oy = c.info.resolution, c.info.origin.position.x, c.info.origin.position.y
            pas = max(1, int(0.35 / res))
            jj, ii = np.nonzero(atteignable[::pas, ::pas])
            jj, ii = jj * pas, ii * pas
            garde = (cdata[jj, ii] >= 0) & (cdata[jj, ii] <= self.explore_max_cost)
            cand = np.column_stack([ox + (ii[garde] + 0.5) * res, oy + (jj[garde] + 0.5) * res])
            cand = np.array([pt for pt in cand if self.loin_des_ilots(pt[0], pt[1])
                             and all(math.hypot(pt[0] - rx, pt[1] - ry) > 0.5 for rx, ry in self.couv_rates)
                             ]).reshape(-1, 2)
            with self.couv_lock:
                if self.couv.cases is None:
                    return None
                choix = self.couv.meilleur_point(cand, r[:2])
                reste = int(self.couv.non_vues().sum())
            if choix is None or choix[3] < 3:
                if remises >= 1:
                    return None
                remises += 1
                self.log(f'couverture : toutes les surfaces vues ({reste} restantes) sans le tag : remise a zero')
                with self.couv_lock:
                    self.couv.remise_a_zero()
                self.couv_rates.clear()
                continue
            x, y, cap, gain = choix
            self.viewpoints.append((x, y))
            self.log(f'couverture : vers ({x:.2f}, {y:.2f}), {gain} cases a voir, {reste} non vues en tout')
            nav = self.naviguer(x, y, cap, arret=connu, timeout=min(self.nav_timeout, limite - self.t()))
            if nav == 'arret':
                break
            p = self.pose(self.map_frame)
            if nav != 'ok' and (p is None or math.hypot(x - p[0], y - p[1]) > 0.5):
                # point pas atteint : sinon il reste le meilleur et on y retourne sans fin (seed 135)
                self.couv_rates.append((x, y))
                self.log(f'couverture : ({x:.2f}, {y:.2f}) pas atteint, mis de cote')
            if p and abs(wrap(cap - p[2])) > 0.3 and self.tourner(wrap(cap - p[2]), self.spin_speed, connu):
                break
            for angle in (0.0, 0.5, -1.0, 0.5):
                if connu() or (angle and self.tourner(angle, self.spin_speed, connu)) or self.attendre_vue(connu):
                    break
        return connu()

    def explorer(self, cible, limite):
        """Cherche le tag cible jusqu'au temps limite. Renvoie True si sa position est connue."""
        connu = lambda: cible in self.tags  # noqa: E731
        self.log(f'exploration pour le tag {cible}')
        if not self.viewpoints:
            p = self.pose(self.map_frame)
            if p:
                self.viewpoints.append(p[:2])
            self.regarder_autour(connu)
        if self.tournee_active and not connu() and self.tournee(connu, limite):
            return True
        if not connu() and self.t() < limite:
            trouve = self.chercher_par_couverture(connu, limite)
            if trouve is not None:
                return trouve
            self.log('couverture indisponible : exploration classique')
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
        echecs = 0   # points de vue inaccessibles ou jamais atteints, d'affilee
        while self.t() < limite:
            self.verifie()
            if echecs >= 2:
                # position ou orientation memorisee probablement fausse (tag vu de loin et de biais :
                # point de vue derriere un mur, course juges seed 135) : on le recherche pour la remesurer
                self.log(f'tag {i} : {echecs} points de vue inaccessibles, on le remesure')
                self.tags.pop(i, None)
                self.indices.pop(i, None)
                echecs = 0
            if i not in self.tags and not self.explorer(i, limite):
                return False
            tx, ty, _, nx, ny = self.tags[i]
            # face au tag n'impose pas d'etre dans son axe : si une table bouche l'axe, on se
            # place en biais (le robot regarde quand meme le tag, cap a +-12 deg)
            beta = self.angle_de_visite(tx, ty, nx, ny)
            if beta is None:
                self.log(f'tag {i} : aucun point de vue accessible autour de ({tx:.2f}, {ty:.2f})')
                echecs += 1
                self.sleep(1.0)
                continue
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
                echecs += 1
                self.sleep(1.0)
                continue
            echecs = 0
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
        soient libres (cout < 99 : 99 robot en collision, 100 obstacle ou masque) et atteignables depuis
        le robot sans traverser d'obstacle connu (case inconnue acceptee : au depart la carte n'est pas
        finie ; un point derriere un mur reste refuse, course juges seed 135). None si aucun angle ne
        convient. 0 sans costmap."""
        accessible = self.zone_accessible(inconnu=True)
        if accessible is None:
            return 0.0
        for deg in (0, 25, -25, 40, -40, 50, -50):
            ux, uy = tourne(nx, ny, math.radians(deg))
            pts = [(tx + d * ux, ty + d * uy) for d in (self.visit_dist, self.view_dist)]
            if all(self.cout_global(x, y) < 99 and accessible(x, y) for x, y in pts):
                return math.radians(deg)
        return None

    def courir(self):
        self.attendre_depart()
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
