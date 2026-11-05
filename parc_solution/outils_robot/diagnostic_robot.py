#!/usr/bin/env python3
"""Diagnostic du robot PARC (equipe MA64).

Enregistre dans un dossier tout ce qu'il faut pour regler notre code sur le vrai robot :
environnement, graphe ROS, frequences et contenu des capteurs, TF, images, audio,
et en option des tests de mouvement (odometrie en rotation et en ligne droite,
approche lente d'un mur jusqu'a notre garde-fou).

Utilisation sur le robot (apres avoir source ROS et l'espace de travail de base) :
    python3 diagnostic_robot.py                      # tests passifs, le robot ne bouge pas
    python3 diagnostic_robot.py --audio              # + 10 s de micro (dire "go" pendant l'enregistrement)
    python3 diagnostic_robot.py --mouvement          # + tests de mouvement (demande confirmation)
Dans la simulation :
    python3 diagnostic_robot.py --profil sim --mouvement

Ne depend que de rclpy, tf2_ros et numpy. Ne publie que sur le topic de commande,
et seulement avec --mouvement.
"""
import argparse
import importlib.util
import json
import math
import os
import platform
import shutil
import struct
import subprocess
import sys
import threading
import time
import zlib
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener

PROFILS = {
    "robot": {
        "scan": "/scan",
        "scan_filtre": "/scan_filtered",
        "odom": "/botwheel_explorer/odom",
        "imus": ["/camera1/camera1/imu", "/camera2/camera2/imu"],
        "images": ["/camera1/camera1/color/image_raw", "/camera2/camera2/color/image_raw"],
        "infos": ["/camera1/camera1/color/camera_info", "/camera2/camera2/color/camera_info",
                  "/camera1/camera1/depth/camera_info", "/camera2/camera2/depth/camera_info"],
        "profondeurs": ["/camera1/camera1/depth/image_rect_raw", "/camera2/camera2/depth/image_rect_raw"],
        "scans_cam": ["/camera1/scan", "/camera2/scan", "/scan_merged"],
        "audio": ["/speech_audio", "/audio", "/audio/channel0"],
        "divers": ["/is_speeching", "/sound_direction", "/joint_states", "/battery_level"],
        "cmd": "/botwheel_explorer/cmd_vel",   # consigne des organisateurs (4 nov)
        "cmd_stamped": True,
        "base": "base_link",
        "reperes": ["odom", "laser", "camera1_link", "camera2_link"],
    },
    "sim": {
        "scan": "/scan",
        "scan_filtre": "/scan_filtered",
        "odom": "/odom",
        "imus": ["/imu"],
        "images": ["/top_camera_color/image_raw", "/bottom_camera_color/image_raw"],
        "infos": ["/top_camera_color/camera_info", "/bottom_camera_color/camera_info"],
        "profondeurs": [],
        "scans_cam": [],
        "audio": [],
        "divers": ["/joint_states"],
        "cmd": "/robot_base_controller/cmd_vel_unstamped",
        "cmd_stamped": False,
        "base": "base_footprint",
        "reperes": ["odom", "base_link"],
    },
}

PARC_TOPICS = ["/parc/tag_order", "/parc/run_status", "/parc/start", "/parc/tag_visit", "/parc/finish"]
PAQUETS = ["apriltag_ros", "apriltag_msgs", "slam_toolbox", "nav2_bringup", "nav2_simple_commander",
           "robot_localization", "laser_filters", "parc_msgs", "cv_bridge", "image_transport",
           "depthimage_to_laserscan", "depth_image_proc", "teleop_twist_keyboard", "foxglove_bridge",
           "rosbridge_server", "audio_common_msgs"]
MODULES_PY = ["numpy", "cv2", "cv_bridge", "yaml", "scipy", "vosk", "sounddevice", "pyaudio",
              "transforms3d", "PIL"]


# ---------- outils ----------

def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def rpy_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch, yaw_of(q)


def rotate(q, v):
    """Applique la rotation du quaternion q (x, y, z, w) au vecteur v."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx)


def yaw_rot(rot):
    """Lacet d'une rotation (x, y, z, w), 0 si inconnue."""
    if rot is None:
        return 0.0
    x, y, z, w = rot
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def save_png(path, width, height, rgb):
    """PNG RGB 8 bits sans dependance (rgb : bytes de taille width * height * 3)."""
    raw = b"".join(b"\x00" + rgb[r * width * 3:(r + 1) * width * 3] for r in range(height))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def image_to_rgb(msg):
    """Convertit un sensor_msgs/Image en (rgb bytes, info) pour les encodages courants."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if enc == "bgr8":
            img = img[:, :, ::-1]
    elif enc in ("rgba8", "bgra8"):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 4].reshape(msg.height, msg.width, 4)[:, :, :3]
        if enc == "bgra8":
            img = img[:, :, ::-1]
    elif enc == "mono8":
        g = buf.reshape(msg.height, msg.step)[:, :msg.width]
        img = np.stack([g, g, g], axis=2)
    elif enc in ("16uc1", "mono16", "32fc1"):
        dt = np.float32 if enc == "32fc1" else np.uint16
        d = np.frombuffer(bytes(msg.data), dtype=dt).reshape(msg.height, msg.step // np.dtype(dt).itemsize)[:, :msg.width]
        d = d.astype(np.float32)
        valid = d[np.isfinite(d) & (d > 0)]
        top = np.percentile(valid, 98) if valid.size else 1.0
        g = np.clip(d / max(top, 1e-6) * 255, 0, 255).astype(np.uint8)
        img = np.stack([g, g, g], axis=2)
    else:
        return None
    return np.ascontiguousarray(img).tobytes()


def run(cmd, timeout=20):
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return (p.stdout + p.stderr).strip()
    except Exception as e:  # noqa: BLE001
        return f"ECHEC : {e}"


# ---------- noeud d'ecoute ----------

class Ecoute:
    """Abonnement generique : compte, horodate et garde le dernier message de chaque topic."""

    def __init__(self, node):
        self.node = node
        self.stats = {}
        self.callbacks = {}  # topic -> fonction appelee a chaque message (tests de mouvement)
        self.subs = []

    def ajouter(self, topic, type_str, qos):
        try:
            cls = get_message(type_str)
        except Exception as e:  # noqa: BLE001
            self.stats[topic] = {"type": type_str, "erreur": f"type non importable : {e}"}
            return
        st = {"type": type_str, "n": 0, "t": [], "dernier": None, "premier": None, "retards": [], "tailles": []}
        self.stats[topic] = st

        def cb(msg, topic=topic, st=st):
            now = time.time()
            st["n"] += 1
            st["t"].append(now)
            st["dernier"] = msg
            if st["premier"] is None:
                st["premier"] = msg
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            if stamp is not None and stamp.sec > 0:
                st["retards"].append(self.node.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9))
            if type_str.endswith("AudioData"):
                st["tailles"].append(len(msg.data))
            f = self.callbacks.get(topic)
            if f:
                f(msg)

        self.subs.append(self.node.create_subscription(cls, topic, cb, qos))

    def resume(self, topic):
        st = self.stats.get(topic)
        if st is None:
            return {"present": False}
        if "erreur" in st:
            return st
        r = {"type": st["type"], "messages": st["n"]}
        if st["n"] >= 2:
            dt = st["t"][-1] - st["t"][0]
            r["frequence_hz"] = round((st["n"] - 1) / dt, 2) if dt > 0 else None
        if st["retards"]:
            r["retard_stamp_s"] = {"median": round(float(np.median(st["retards"])), 4),
                                   "max": round(float(np.max(st["retards"])), 4)}
        m = st["dernier"]
        if m is not None and hasattr(m, "header"):
            r["frame_id"] = m.header.frame_id
        return r


# ---------- analyses ----------

def analyse_scan(msg, yaw_laser):
    r = np.array(msg.ranges, dtype=np.float32)
    ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
    ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
    base_ang = np.array([wrap(a + yaw_laser) for a in ang])
    proches = ok & (r < 0.40)
    front = ok & (np.abs(base_ang) < math.radians(10))
    res = {
        "angle_min_deg": round(math.degrees(msg.angle_min), 2), "angle_max_deg": round(math.degrees(msg.angle_max), 2),
        "increment_deg": round(math.degrees(msg.angle_increment), 4), "points": int(r.size),
        "range_min": msg.range_min, "range_max": msg.range_max,
        "points_valides": int(ok.sum()), "distance_min": round(float(r[ok].min()), 3) if ok.any() else None,
        "points_a_moins_de_40cm": int(proches.sum()),
        "secteurs_moins_40cm_deg_base": secteurs(base_ang[proches]),
        "distance_devant_m": round(float(np.median(r[front])), 3) if front.any() else None,
        "intensites": len(msg.intensities) > 0,
    }
    return res


def secteurs(angles, pas=math.radians(5)):
    """Regroupe des angles (rad) en secteurs contigus, renvoyes en degres."""
    if len(angles) == 0:
        return []
    a = np.sort(np.array(angles))
    out, debut, prec = [], a[0], a[0]
    for x in a[1:]:
        if x - prec > pas:
            out.append([round(math.degrees(debut), 1), round(math.degrees(prec), 1)])
            debut = x
        prec = x
    out.append([round(math.degrees(debut), 1), round(math.degrees(prec), 1)])
    return out


def compare_scans(brut, filtre, yaw_laser):
    if brut is None or filtre is None:
        return None
    if len(brut.ranges) != len(filtre.ranges):
        return {"remarque": f"tailles differentes : {len(brut.ranges)} contre {len(filtre.ranges)}"}
    rb = np.array(brut.ranges, dtype=np.float32)
    rf = np.array(filtre.ranges, dtype=np.float32)
    okb = np.isfinite(rb) & (rb >= brut.range_min) & (rb <= brut.range_max)
    okf = np.isfinite(rf) & (rf >= filtre.range_min) & (rf <= filtre.range_max)
    retires = okb & ~okf
    ang = brut.angle_min + np.arange(rb.size) * brut.angle_increment
    base_ang = np.array([wrap(a + yaw_laser) for a in ang])
    return {
        "valides_brut": int(okb.sum()), "valides_filtre": int(okf.sum()), "retires": int(retires.sum()),
        "distance_max_des_retires": round(float(rb[retires].max()), 3) if retires.any() else None,
        "secteurs_retires_deg_base": secteurs(base_ang[retires]),
        "remarque": "comparaison du dernier message de chaque topic, pas forcement le meme balayage",
    }


def analyse_imu(msgs, rot_base):
    gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in msgs])
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in msgs])
    m0 = msgs[-1]
    res = {
        "echantillons": len(msgs),
        "orientation_fournie": m0.orientation_covariance[0] != -1.0
        and not (m0.orientation.w == 0 and m0.orientation.x == 0 and m0.orientation.y == 0 and m0.orientation.z == 0),
        "orientation_covariance_0": m0.orientation_covariance[0],
        "gyro_moyen_rad_s": np.round(gyro.mean(axis=0), 5).tolist(),
        "gyro_ecart_type": np.round(gyro.std(axis=0), 5).tolist(),
        "acc_moyenne_m_s2": np.round(acc.mean(axis=0), 3).tolist(),
        "covariances_gyro_acc": [m0.angular_velocity_covariance[0], m0.linear_acceleration_covariance[0]],
    }
    if rot_base is not None:
        g = rotate(rot_base, tuple(acc.mean(axis=0)))
        res["acc_moyenne_dans_base"] = [round(v, 3) for v in g]
        res["remarque"] = "au repos, acc dans base doit etre proche de (0, 0, +9.8) si la TF de l'IMU est juste"
    return res


# ---------- programme principal ----------

class Diagnostic:
    def __init__(self, args):
        self.args = args
        self.p = PROFILS[args.profil]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out = os.path.abspath(args.sortie or f"diag_MA64_{stamp}")
        os.makedirs(self.out, exist_ok=True)
        self.res = {"date": stamp, "profil": args.profil, "arguments": vars(args)}
        self.log_f = open(os.path.join(self.out, "journal.txt"), "w")

        rclpy.init()
        self.node = rclpy.create_node("diag_ma64")
        if args.sim_time:
            self.node.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self.node)
        self.ecoute = Ecoute(self.node)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.spin_thread.start()
        self.cmd_pub = None

    def maintenant(self):
        """Temps du robot en secondes (temps simule avec --sim-time)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def attendre(self, duree):
        t0 = self.maintenant()
        while self.maintenant() - t0 < duree:
            time.sleep(0.02)

    def log(self, *a):
        line = " ".join(str(x) for x in a)
        print(line, flush=True)
        self.log_f.write(line + "\n")
        self.log_f.flush()

    def sauver(self):
        with open(os.path.join(self.out, "resultats.json"), "w") as f:
            json.dump(self.res, f, indent=2, ensure_ascii=False, default=str)

    # --- 1. environnement ---
    def environnement(self):
        self.log("== 1. Environnement")
        env = {k: os.environ.get(k) for k in ("ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
                                               "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE",
                                               "AMENT_PREFIX_PATH", "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE")}
        env.update({
            "machine": platform.machine(), "noyau": platform.release(), "python": sys.version.split()[0],
            "processeurs": os.cpu_count(), "charge": os.getloadavg(),
            "disque_libre_go": round(shutil.disk_usage(self.out).free / 1e9, 1),
            "memoire": run("free -m"), "nvpmodel": run("nvpmodel -q 2>&1 | head -3", 5),
            "utilisateur": os.environ.get("USER"), "dossier": os.getcwd(),
        })
        from ament_index_python.packages import get_package_prefix
        paquets = {}
        for p in PAQUETS:
            try:
                paquets[p] = get_package_prefix(p)
            except Exception:  # noqa: BLE001
                paquets[p] = None
        env["paquets_ros"] = paquets
        env["modules_python"] = {m: importlib.util.find_spec(m) is not None for m in MODULES_PY}
        self.res["environnement"] = env
        manquants = [p for p, v in paquets.items() if v is None]
        self.log("  machine", env["machine"], "| ROS", env["ROS_DISTRO"], "| domaine", env["ROS_DOMAIN_ID"],
                 "| RMW", env["RMW_IMPLEMENTATION"])
        self.log("  paquets ROS absents :", ", ".join(manquants) or "aucun")
        self.log("  modules Python absents :", ", ".join(m for m, v in env["modules_python"].items() if not v) or "aucun")

    # --- 2. graphe ---
    def graphe(self):
        self.log("== 2. Graphe ROS (decouverte 5 s)")
        time.sleep(5.0)
        topics = dict(self.node.get_topic_names_and_types())
        noeuds = sorted(f"{ns.rstrip('/')}/{n}" for n, ns in self.node.get_node_names_and_namespaces())
        g = {"noeuds": noeuds, "topics": {t: ty for t, ty in sorted(topics.items())}}
        detail = {}
        for t in [self.p["cmd"], "/tf", "/tf_static", "/goal"] + PARC_TOPICS:
            if t in topics:
                pubs = self.node.get_publishers_info_by_topic(t)
                subs = self.node.get_subscriptions_info_by_topic(t)
                detail[t] = {"publieurs": [i.node_name for i in pubs], "abonnes": [i.node_name for i in subs]}
        g["detail"] = detail
        g["topics_parc_presents"] = [t for t in PARC_TOPICS if t in topics]
        if self.res["environnement"]["paquets_ros"].get("parc_msgs"):
            g["parc_msgs"] = run("for m in $(ros2 interface package parc_msgs); do echo \"== $m\"; ros2 interface show $m; done", 60)
        self.res["graphe"] = g
        self.topics = topics
        self.log(f"  {len(noeuds)} noeuds, {len(topics)} topics ; topics /parc presents : {g['topics_parc_presents'] or 'aucun'}")
        if self.p["cmd"] in detail:
            self.log(f"  {self.p['cmd']} : publieurs {detail[self.p['cmd']]['publieurs']}, abonnes {detail[self.p['cmd']]['abonnes']}")

    # --- 3. capteurs ---
    def abonner(self):
        p = self.p
        liste = [p["scan"], p["scan_filtre"], p["odom"]] + p["imus"] + p["images"] + p["infos"] \
            + p["profondeurs"] + p["scans_cam"] + p["audio"] + p["divers"] + [p["cmd"]]
        # p["cmd"] ecoute au repos : si quelqu'un y publie deja (obstacle_avoidance ?), nos commandes
        # seraient en conflit avec les siennes
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for t in liste:
            if t in self.topics:
                self.ecoute.ajouter(t, self.topics[t][0], qos_profile_sensor_data)
            else:
                self.ecoute.stats[t] = {"erreur": "topic absent du graphe"}
        for t in ["/parc/tag_order", "/parc/run_status"]:
            if t in self.topics:
                self.ecoute.ajouter(t, self.topics[t][0], latched)

    def lookup(self, parent, enfant):
        try:
            tr = self.tf.lookup_transform(parent, enfant, Time())
        except Exception as e:  # noqa: BLE001
            return None, str(e).split("\n")[0]
        t, q = tr.transform.translation, tr.transform.rotation
        r, pi, y = rpy_of(q)
        return (q.x, q.y, q.z, q.w), {"xyz": [round(t.x, 4), round(t.y, 4), round(t.z, 4)],
                                       "rpy_deg": [round(math.degrees(r), 2), round(math.degrees(pi), 2), round(math.degrees(y), 2)]}

    def capteurs(self):
        d = self.args.duree
        self.log(f"== 3. Capteurs (ecoute {d:.0f} s, le robot doit rester immobile)")
        imu_hist = {t: [] for t in self.p["imus"]}
        for t in self.p["imus"]:
            self.ecoute.callbacks[t] = lambda m, t=t: imu_hist[t].append(m)
        time.sleep(d)
        for t in self.p["imus"]:
            self.ecoute.callbacks.pop(t, None)

        base = self.p["base"]
        res = {t: self.ecoute.resume(t) for t in self.ecoute.stats}
        st = self.ecoute.stats

        # TF : tous les reperes, puis base -> chaque repere utile
        frames = set(self.p["reperes"])
        for t, s in st.items():
            m = s.get("dernier") if isinstance(s, dict) else None
            if m is not None and hasattr(m, "header") and m.header.frame_id:
                frames.add(m.header.frame_id)
        try:
            self.res["tf_arbre"] = self.tf.all_frames_as_yaml()
        except Exception as e:  # noqa: BLE001
            self.res["tf_arbre"] = str(e)
        tfs, rots = {}, {}
        for f in sorted(frames):
            rot, info = self.lookup(base, f)
            tfs[f"{base} -> {f}"] = info
            rots[f] = rot
        self.res["tf"] = tfs
        _, info_map = self.lookup("map", base)
        self.res["tf_map_existe"] = info_map if isinstance(info_map, dict) else False

        # LiDAR
        scan = st.get(self.p["scan"], {}).get("dernier")
        yaw_laser = 0.0
        if scan is not None:
            yaw_laser = yaw_rot(rots.get(scan.header.frame_id))
            res[self.p["scan"]]["analyse"] = analyse_scan(scan, yaw_laser)
            res[self.p["scan"]]["analyse"]["yaw_laser_dans_base_deg"] = round(math.degrees(yaw_laser), 2)
        filtre = st.get(self.p["scan_filtre"], {}).get("dernier")
        if filtre is not None:
            res[self.p["scan_filtre"]]["analyse"] = analyse_scan(filtre, yaw_laser)
            res[self.p["scan_filtre"]]["comparaison_avec_scan_brut"] = compare_scans(scan, filtre, yaw_laser)
        for t in self.p["scans_cam"]:
            m = st.get(t, {}).get("dernier")
            if m is not None:
                res[t]["analyse"] = analyse_scan(m, yaw_rot(rots.get(m.header.frame_id)))

        # odometrie
        s = st.get(self.p["odom"], {})
        if s.get("dernier") is not None:
            a, b = s["premier"], s["dernier"]
            res[self.p["odom"]]["analyse"] = {
                "frame_id": b.header.frame_id, "child_frame_id": b.child_frame_id,
                "derive_pendant_ecoute_m": round(math.dist((a.pose.pose.position.x, a.pose.pose.position.y),
                                                          (b.pose.pose.position.x, b.pose.pose.position.y)), 4),
                "derive_yaw_deg": round(math.degrees(wrap(yaw_of(b.pose.pose.orientation) - yaw_of(a.pose.pose.orientation))), 3),
                "covariance_pose_diag": [round(b.pose.covariance[i], 6) for i in (0, 7, 14, 21, 28, 35)],
                "covariance_twist_diag": [round(b.twist.covariance[i], 6) for i in (0, 7, 14, 21, 28, 35)],
                "vitesse_au_repos": [round(b.twist.twist.linear.x, 4), round(b.twist.twist.angular.z, 4)],
            }

        # IMU
        for t, msgs in imu_hist.items():
            if msgs:
                res[t]["analyse"] = analyse_imu(msgs, rots.get(msgs[-1].header.frame_id))

        # camera_info
        for t in self.p["infos"]:
            m = st.get(t, {}).get("dernier")
            if m is not None:
                res[t]["analyse"] = {"taille": [m.width, m.height], "K": [round(v, 2) for v in m.k],
                                     "modele": m.distortion_model, "D": [round(v, 5) for v in m.d]}

        # images : une capture de chaque, et distance au centre pour la profondeur
        for t in self.p["images"] + self.p["profondeurs"]:
            m = st.get(t, {}).get("dernier")
            if m is None:
                continue
            info = {"encodage": m.encoding, "taille": [m.width, m.height]}
            rgb = image_to_rgb(m)
            if rgb is not None:
                nom = t.strip("/").replace("/", "_") + ".png"
                save_png(os.path.join(self.out, nom), m.width, m.height, rgb)
                info["capture"] = nom
            if m.encoding.lower() in ("16uc1", "mono16"):
                d = np.frombuffer(bytes(m.data), dtype=np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
                c = d[m.height // 2 - 10:m.height // 2 + 10, m.width // 2 - 10:m.width // 2 + 10]
                c = c[c > 0]
                info["distance_centre_brute"] = float(np.median(c)) if c.size else None
                info["remarque"] = "16UC1 RealSense : en millimetres en general"
            res[t]["analyse"] = info

        # divers : valeur du dernier message
        for t in self.p["divers"] + ["/parc/tag_order", "/parc/run_status"]:
            m = st.get(t, {}).get("dernier")
            if m is not None:
                txt = str(m)
                res[t]["dernier_message"] = txt[:600]

        self.res["capteurs"] = res
        for t, r in res.items():
            if "erreur" in r:
                self.log(f"  {t:45s} {r['erreur']}")
            else:
                self.log(f"  {t:45s} {r.get('messages', 0):5d} msg  {str(r.get('frequence_hz')):>7s} Hz  frame {r.get('frame_id', '-')}")
        self.log("  TF :")
        for k, v in tfs.items():
            self.log(f"    {k:55s} {v}")

    # --- 4. audio ---
    def audio(self):
        topics = [t for t in self.p["audio"] if t in self.topics]
        if not topics:
            self.log("== 4. Audio : aucun topic audio")
            return
        d = self.args.duree_audio
        self.log(f"== 4. Audio : enregistrement {d:.0f} s. DITES 'go', 'allez' puis 'vai' maintenant.")
        bufs = {t: bytearray() for t in topics}
        speech = []
        for t in topics:
            self.ecoute.callbacks[t] = lambda m, t=t: bufs[t].extend(bytes(m.data))
        if "/is_speeching" in self.topics:
            self.ecoute.callbacks["/is_speeching"] = lambda m: speech.append((round(time.time() - t0, 2), bool(m.data)))
        t0 = time.time()
        time.sleep(d)
        for t in topics + ["/is_speeching"]:
            self.ecoute.callbacks.pop(t, None)
        res = {}
        for t in topics:
            nom = t.strip("/").replace("/", "_") + ".raw"
            with open(os.path.join(self.out, nom), "wb") as f:
                f.write(bufs[t])
            octets_s = len(bufs[t]) / d
            tailles = self.ecoute.stats[t]["tailles"]
            res[t] = {"fichier": nom, "octets": len(bufs[t]), "octets_par_s": round(octets_s),
                      "taille_message_mediane": int(np.median(tailles)) if tailles else None,
                      "hypotheses": {"16kHz int16 mono": 32000, "16kHz int16 x4": 128000, "16kHz int16 x6": 192000,
                                     "48kHz int16 mono": 96000}}
            if 30000 < octets_s < 34000:
                wav = nom.replace(".raw", "_16k_s16_mono.wav")
                ecrire_wav(os.path.join(self.out, wav), bufs[t], 16000, 1)
                res[t]["wav_hypothese"] = wav
            self.log(f"  {t}: {len(bufs[t])} octets, {octets_s:.0f} octets/s")
        res["is_speeching"] = speech
        self.res["audio"] = res

    # --- 5. mouvement ---
    def commande(self, vx, wz):
        m = Twist()
        m.linear.x, m.angular.z = float(vx), float(wz)
        if self.p["cmd_stamped"]:
            m = TwistStamped(twist=m)
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.header.frame_id = self.p["base"]
        self.cmd_pub.publish(m)

    def stop(self):
        for _ in range(10):
            self.commande(0.0, 0.0)
            time.sleep(0.05)

    def devant(self):
        """Distance mediane devant le robot (+-10 deg), depuis le dernier scan."""
        m = self.ecoute.stats.get(self.p["scan"], {}).get("dernier")
        if m is None:
            return None
        return analyse_scan(m, self.yaw_laser)["distance_devant_m"]

    def mouvement(self):
        self.log("== 5. Tests de mouvement")
        print(f"\nLe robot va publier sur {self.p['cmd']} a basse vitesse.\n"
              "  Test A : un tour complet sur place a 0,3 rad/s (zone libre de 0,6 m autour du robot).\n"
              "  Test B : robot face a un mur entre 1,2 et 2 m : avance de 0,4 m a 0,1 m/s,\n"
              "           puis approche a 0,06 m/s jusqu'a notre garde-fou\n"
              "           (notre garde-fou arrete a 0,30 m du mur).\n"
              "Ctrl+C arrete le robot a tout moment.")
        if not self.args.oui and input("Zone degagee et personne a proximite ? Tape OUI : ").strip() != "OUI":
            self.log("  tests de mouvement annules")
            return
        self.cmd_pub = self.node.create_publisher(TwistStamped if self.p["cmd_stamped"] else Twist, self.p["cmd"], 10)
        scan = self.ecoute.stats.get(self.p["scan"], {}).get("dernier")
        self.yaw_laser = 0.0
        if scan is not None:
            self.yaw_laser = yaw_rot(self.lookup(self.p["base"], scan.header.frame_id)[0])
        res = {}
        try:
            res["rotation"] = self.test_rotation()
            self.stop()
            if self.args.oui or input("\nPlace le robot face a un mur (1,2 a 2 m), puis tape OUI (autre chose pour sauter) : ").strip() == "OUI":
                res["ligne_et_arret"] = self.test_ligne()
        except KeyboardInterrupt:
            self.log("  interrompu")
        finally:
            self.stop()
        self.res["mouvement"] = res

    def suivi(self):
        """Prepare l'integration de l'odometrie et des gyros pendant un mouvement."""
        etat = {"odom_yaw": 0.0, "odom_last": None, "odom_xy": None, "odom_v": 0.0, "gyro": {}, "gyro_t": {}}

        def on_odom(m):
            y = yaw_of(m.pose.pose.orientation)
            if etat["odom_last"] is not None:
                etat["odom_yaw"] += wrap(y - etat["odom_last"])
            etat["odom_last"] = y
            etat["odom_xy"] = (m.pose.pose.position.x, m.pose.pose.position.y)
            etat["odom_v"] = m.twist.twist.linear.x

        def on_imu(m, t):
            rot, _ = self.lookup(self.p["base"], m.header.frame_id) if t not in etat.setdefault("rot", {}) else (etat["rot"][t], None)
            etat["rot"][t] = rot
            w = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
            wz = rotate(rot, w)[2] if rot else w[2]
            ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if t in etat["gyro_t"] and 0 < ts - etat["gyro_t"][t] < 0.1:
                etat["gyro"][t] = etat["gyro"].get(t, 0.0) + wz * (ts - etat["gyro_t"][t])
            etat["gyro_t"][t] = ts

        self.ecoute.callbacks[self.p["odom"]] = on_odom
        for t in self.p["imus"]:
            if t in self.topics:
                self.ecoute.callbacks[t] = lambda m, t=t: on_imu(m, t)
        return etat

    def fin_suivi(self):
        for t in [self.p["odom"]] + self.p["imus"]:
            self.ecoute.callbacks.pop(t, None)

    def test_rotation(self):
        self.log("  Test A : rotation d'un tour")
        scan0 = self.ecoute.stats[self.p["scan"]]["dernier"]
        etat = self.suivi()
        self.attendre(0.5)
        o0, g0 = etat["odom_yaw"], dict(etat["gyro"])
        # reference d'arret : le premier gyro qui publie, sinon l'odometrie
        imu_ref = next((t for t in self.p["imus"] if t in g0), None)
        self.log(f"  reference d'arret : {imu_ref or 'odometrie'}")
        t0 = self.maintenant()
        bouge = False
        ref = 0.0
        while self.maintenant() - t0 < 60:
            self.commande(0.0, 0.3)
            time.sleep(0.05)
            ref = etat["gyro"][imu_ref] - g0[imu_ref] if imu_ref else etat["odom_yaw"] - o0
            if abs(ref) > 0.05:
                bouge = True
            if not bouge and self.maintenant() - t0 > 4:
                self.log("  le robot ne tourne pas : la commande n'arrive pas aux roues ?")
                self.fin_suivi()
                return {"erreur": "aucune rotation mesuree apres 4 s"}
            if abs(ref) >= 2 * math.pi:
                break
        self.stop()
        self.attendre(1.5)
        scan1 = self.ecoute.stats[self.p["scan"]]["dernier"]
        self.fin_suivi()
        odom = etat["odom_yaw"] - o0
        gyros = {t: v - g0.get(t, 0.0) for t, v in etat["gyro"].items()}
        modulo = residu_scan(scan0, scan1)
        res = {"duree_s": round(self.maintenant() - t0, 1), "tour_complet_selon_reference": abs(ref) >= 2 * math.pi,
               "odom_deg": round(math.degrees(odom), 1),
               "gyro_deg": {t: round(math.degrees(v), 1) for t, v in gyros.items()},
               "lidar_modulo_360_deg": round(math.degrees(modulo), 2) if modulo is not None else None}
        if modulo is not None:
            # le LiDAR donne l'angle a un tour pres : on prend le nombre de tours le plus proche de la reference
            k = round((ref - modulo) / (2 * math.pi))
            vrai = modulo + 2 * math.pi * k
            res["rotation_reelle_estimee_deg"] = round(math.degrees(vrai), 1)
            if abs(vrai) > 0.5:
                res["facteur_odom"] = round(odom / vrai, 3)
                res["facteur_gyro"] = {t: round(v / vrai, 3) for t, v in gyros.items()}
        res["remarque"] = ("rotation reelle = decalage entre le scan avant et le scan apres (correlation circulaire), "
                           "complete du nombre de tours donne par la reference ; facteur 1.000 = mesure juste")
        self.log(f"  {res}")
        return res

    def test_ligne(self):
        self.log("  Test B : ligne droite puis distance d'arret")
        d0 = self.devant()
        if d0 is None or not (1.0 <= d0 <= 3.0):
            self.log(f"  distance devant = {d0} m : il faut un mur entre 1,2 et 2 m devant. Test saute.")
            return {"erreur": f"distance devant {d0}"}
        etat = self.suivi()
        self.attendre(0.5)
        xy0 = etat["odom_xy"]
        t0 = self.maintenant()
        while self.maintenant() - t0 < 10:
            self.commande(0.1, 0.0)
            time.sleep(0.05)
            if etat["odom_xy"] and math.dist(etat["odom_xy"], xy0) >= 0.4:
                break
            if (self.devant() or 9) < 0.5:
                break
        self.stop()
        self.attendre(1.5)
        d1 = self.devant()
        od = math.dist(etat["odom_xy"], xy0)
        res = {"devant_avant_m": d0, "devant_apres_m": d1, "odom_m": round(od, 3),
               "lidar_m": round(d0 - d1, 3) if d1 is not None else None}
        if d1 is not None and d0 - d1 > 0.05:
            res["facteur_odom"] = round(od / (d0 - d1), 3)

        # approche lente : le robot s'arrete-t-il de lui-meme avant notre garde-fou ?
        trace, arret = [], None
        t0 = self.maintenant()
        immobile_depuis = None
        while self.maintenant() - t0 < 30:
            self.commande(0.06, 0.0)
            time.sleep(0.05)
            d = self.devant()
            v = etat["odom_v"]
            trace.append((round(self.maintenant() - t0, 2), d, round(v, 3)))
            if d is not None and d < 0.30:
                arret = {"par": "garde-fou du script (0,30 m)", "distance_m": d}
                break
            if abs(v) < 0.005 and self.maintenant() - t0 > 2:
                immobile_depuis = immobile_depuis or self.maintenant()
                if self.maintenant() - immobile_depuis > 1.5:
                    arret = {"par": "robot immobile malgre la commande (un autre noeud bloque ?)", "distance_m": d}
                    break
            else:
                immobile_depuis = None
        self.stop()
        self.fin_suivi()
        res["arret"] = arret
        res["trace_t_distance_vitesse"] = trace[::4]
        self.log(f"  ligne : {dict((k, v) for k, v in res.items() if k != 'trace_t_distance_vitesse')}")
        return res

    # --- bag ---
    def bag_debut(self):
        if not self.args.bag:
            return None
        p = self.p
        topics = [t for t in [p["scan"], p["scan_filtre"], p["odom"], "/tf", "/tf_static", "/joint_states"]
                  + p["imus"] + p["infos"] + p["scans_cam"] + [p["cmd"]] if t in self.topics]
        dossier = os.path.join(self.out, "bag")
        self.log(f"  rosbag : {' '.join(topics)}")
        return subprocess.Popen(["ros2", "bag", "record", "-o", dossier] + topics,
                                stdout=open(os.path.join(self.out, "bag.log"), "w"), stderr=subprocess.STDOUT)

    def executer(self):
        bag = None
        try:
            self.environnement()
            self.sauver()
            self.graphe()
            self.sauver()
            self.abonner()
            bag = self.bag_debut()
            self.capteurs()
            self.sauver()
            if self.args.audio:
                self.audio()
                self.sauver()
            if self.args.mouvement:
                self.mouvement()
                self.sauver()
        finally:
            if self.cmd_pub is not None:
                self.stop()
            if bag is not None:
                bag.send_signal(2)
                try:
                    bag.wait(10)
                except subprocess.TimeoutExpired:
                    bag.kill()
            self.sauver()
            self.log(f"\nResultats dans {self.out}")
            self.ex.shutdown()
            self.spin_thread.join(timeout=3.0)
            self.node.destroy_node()
            rclpy.try_shutdown()


def residu_scan(a, b):
    """Angle (rad) dont le robot a tourne entre deux scans, modulo un tour, par correlation circulaire."""
    if a is None or b is None or len(a.ranges) != len(b.ranges) or a.angle_increment == 0:
        return None
    ra = np.nan_to_num(np.array(a.ranges, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    rb = np.nan_to_num(np.array(b.ranges, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    # le corps et les roues tournent avec le LiDAR : on les retire, sinon ils tirent le resultat vers 0
    ra[ra < 0.45] = 0.0
    rb[rb < 0.45] = 0.0
    n = ra.size
    meilleur, best = None, None
    for s in range(-(n // 2), n - n // 2):
        rs = np.roll(ra, -s)
        ok = (rs > 0) & (rb > 0)
        if ok.sum() < n * 0.3:
            continue
        e = float(np.mean(np.abs(rs[ok] - rb[ok])))
        if best is None or e < best:
            best, meilleur = e, s
    if meilleur is None:
        return None
    return meilleur * a.angle_increment


def ecrire_wav(path, data, rate, channels):
    with open(path, "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", 36 + len(data)) + b"WAVEfmt "
                + struct.pack("<IHHIIHH", 16, 1, channels, rate, rate * channels * 2, channels * 2, 16)
                + b"data" + struct.pack("<I", len(data)) + bytes(data))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profil", choices=PROFILS, default="robot")
    ap.add_argument("--duree", type=float, default=15.0, help="duree d'ecoute des capteurs (s)")
    ap.add_argument("--audio", action="store_true", help="enregistre le micro")
    ap.add_argument("--duree-audio", type=float, default=10.0)
    ap.add_argument("--mouvement", action="store_true", help="tests de mouvement (rotation, ligne, arret)")
    ap.add_argument("--oui", action="store_true", help="ne demande pas de confirmation (simulation)")
    ap.add_argument("--bag", action=argparse.BooleanOptionalAction, default=True,
                    help="enregistre un rosbag sans images (--no-bag pour desactiver)")
    ap.add_argument("--sim-time", action="store_true")
    ap.add_argument("--sortie", help="dossier de sortie (defaut : ./diag_MA64_<date>)")
    Diagnostic(ap.parse_args()).executer()


if __name__ == "__main__":
    main()
