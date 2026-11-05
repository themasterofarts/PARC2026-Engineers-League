#!/usr/bin/env python3
"""Evaluateur pour le simulateur officiel PARC 2026 (tests seulement : lit les topics /sim/).

Note chaque TagVisit avec la verite du monde genere (/tmp/parc_worlds/parc_random_seed<N>.json)
et la vraie pose du robot (--pose-topic ; /sim/pose ne publie rien : le pont officiel ecoute le
topic Gazebo sitoe_robot/pose, alors que la pose est sur /model/sitoe_robot/pose) : distance centre du robot -> tag, angle face au tag,
ecart a la pose de visite ideale, immobilite pendant 3 s. Compte aussi les contacts du robot
avec autre chose que le sol (/sim/*_collisions), penalises en competition.

    python3 evaluateur_sim.py --seed 7 --rapport /tmp/eval.json
"""
import argparse
import json
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Pose
from parc_msgs.msg import RunStatus, TagVisit
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ros_gz_interfaces.msg import Contacts

ETATS = {0: 'IDLE', 1: 'ARMED', 2: 'RUNNING', 3: 'FINISHED', 4: 'TIMED_OUT', 5: 'TERMINATED'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, help='monde genere (phase2_hard)')
    ap.add_argument('--monde', default='/tmp/parc_worlds')
    ap.add_argument('--verite', help='verite d\'un monde fixe (au lieu de --seed), ex. simu_officielle/arene_fixe.json')
    ap.add_argument('--rapport', required=True)
    ap.add_argument('--pose-topic', default='/verite/robot_pose')
    ap.add_argument('--demi-avant', type=float, default=0.20,
                    help='centre -> avant du robot simule (m), d\'apres son modele de collision')
    a = ap.parse_args()
    verite = json.load(open(a.verite or f'{a.monde}/parc_random_seed{a.seed}.json'))
    tags = {t['id']: t for t in verite['tags']}

    # Ctrl+C, kill -INT ou kill (course.sh) : on sort de la boucle et on ecrit quand meme le rapport.
    # Lance en arriere-plan par un script, le processus herite d'un SIGINT ignore : on le retablit.
    def interrompre(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrompre)
    signal.signal(signal.SIGTERM, interrompre)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = rclpy.create_node('evaluateur_sim')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    etat = {'pose': None, 'visites': [], 'contacts': {}, 'run': None, 'fin': None, 'trajet_m': 0.0, 'prec': None,
            'inclinaison_max_deg': 0.0, 'premiers_contacts': {}, 't_depart': None}

    def on_pose(m):
        p = (m.position.x, m.position.y, yaw_of(m.orientation))
        q = m.orientation   # inclinaison : angle entre l'axe z du robot et la verticale
        incl = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x ** 2 + q.y ** 2)))))
        etat['inclinaison_max_deg'] = max(etat['inclinaison_max_deg'], round(incl, 1))
        if etat['prec'] is not None:
            etat['trajet_m'] += math.hypot(p[0] - etat['prec'][0], p[1] - etat['prec'][1])
        etat['prec'] = etat['pose'] = p
        t = n.get_clock().now().nanoseconds * 1e-9
        for v in etat['visites']:
            if 'deplacement_3s_m' not in v and t - v['t'] >= 3.0:
                v['deplacement_3s_m'] = round(math.hypot(p[0] - v['pose'][0], p[1] - v['pose'][1]), 4)

    def on_visit(m):
        t = n.get_clock().now().nanoseconds * 1e-9
        v = {'tag': int(m.tag_id), 't': t, 'image_octets': len(m.image.data)}
        tg = tags.get(m.tag_id)
        if etat['pose'] and tg:
            rx, ry, ryaw = etat['pose']
            d = math.hypot(tg['x'] - rx, tg['y'] - ry)
            face = math.degrees(wrap(math.atan2(tg['y'] - ry, tg['x'] - rx) - ryaw))
            vx, vy, _ = tg['visit_pose']
            # regle (organisateur, 05/11) : le point du robot le plus proche du tag a ~0.5 m, a l'oeil
            avant = d - a.demi_avant
            v.update({'pose': etat['pose'], 'distance_m': round(d, 3), 'distance_avant_m': round(avant, 3),
                      'angle_face_deg': round(face, 1),
                      'ecart_pose_visite_m': round(math.hypot(vx - rx, vy - ry), 3),
                      'valide_0.5m_12deg': 0.35 <= avant <= 0.65 and abs(face) <= 12.0})
        etat['visites'].append(v)
        n.get_logger().info(f'VISITE {v}')

    def on_status(m):
        etat['run'] = m
        if m.state == 2 and etat['t_depart'] is None:     # RUNNING : debut de la course (temps simule)
            etat['t_depart'] = round(n.get_clock().now().nanoseconds * 1e-9, 1)
        if m.state in (3, 4, 5) and etat['fin'] is None:
            etat['fin'] = time.time()

    def on_contact(nom):
        def cb(m):
            for c in m.contacts:
                autres = [c.collision1.name, c.collision2.name]
                if any('ground' in x or 'floor' in x for x in autres):
                    continue
                cle = ' / '.join(sorted(autres))
                etat['contacts'][cle] = etat['contacts'].get(cle, 0) + 1
                if cle not in etat['premiers_contacts']:
                    # ou et quand : temps simule et vraie pose du robot au premier contact
                    t = round(n.get_clock().now().nanoseconds * 1e-9, 1)
                    etat['premiers_contacts'][cle] = {'t': t, 'pose': etat['pose']}
                    n.get_logger().warn(f'CONTACT {cle} a t={t} s, pose {etat["pose"]}')
        return cb

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(Pose, a.pose_topic, on_pose, 10)
    n.create_subscription(TagVisit, '/parc/tag_visit', on_visit, 10)
    n.create_subscription(RunStatus, '/parc/run_status', on_status, latched)
    for c in ('base', 'left_wheel', 'right_wheel', 'top_chassis'):
        n.create_subscription(Contacts, f'/sim/{c}_collisions', on_contact(c), 10)

    try:
        while rclpy.ok() and not (etat['fin'] and time.time() - etat['fin'] > 6):
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    r = etat['run']
    rapport = {
        'seed': a.seed,
        'etat_final': ETATS.get(r.state) if r else None,
        'temps_course_s': round(r.elapsed_s, 1) if r else None,
        't_depart_simule_s': etat['t_depart'],
        'duree_simulee_s': (round(max(v['t'] for v in etat['visites']) - etat['t_depart'], 1)
                            if etat['visites'] and etat['t_depart'] is not None else None),
        'ordre': list(r.tag_order) if r else None,
        'tags_signales': list(r.tags_reported) if r else None,
        'faux_depart': bool(r.false_start) if r else None,
        'trajet_m': round(etat['trajet_m'], 1),
        'inclinaison_max_deg': etat['inclinaison_max_deg'],
        'visites': etat['visites'],
        'contacts_hors_sol': etat['contacts'],
        'premiers_contacts': etat['premiers_contacts'],
    }
    json.dump(rapport, open(a.rapport, 'w'), indent=1, default=str)
    print(json.dumps({k: v for k, v in rapport.items() if k != 'visites'}, default=str))
    for v in etat['visites']:
        print({k: v.get(k) for k in ('tag', 'distance_m', 'distance_avant_m', 'angle_face_deg', 'ecart_pose_visite_m',
                                     'valide_0.5m_12deg', 'deplacement_3s_m')})
    rclpy.shutdown()


if __name__ == '__main__':
    main()
