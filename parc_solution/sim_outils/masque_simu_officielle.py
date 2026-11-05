#!/usr/bin/env python3
"""Masque keepout pour le simulateur officiel PARC 2026 (tests seulement).

Le LiDAR ne voit que la colonne de 4 cm des cafe_table (plateau 0.91 m a 0.75 m, socle 0.56 m de
4 cm au sol) et les pieds des WoodenChair : sans masque, le robot passe sous le plateau, monte sur
le socle et se renverse. Sur le vrai robot, ces surfaces sont dessinees a la main sur la carte
(generer_masque) ; ici on les lit dans le monde genere (/tmp/parc_worlds/parc_random_seed<N>.sdf) :
tables, chaises et visiteurs assis, empreinte au sol de leur maillage de collision, et feuillage
des plantes (sphere au-dessus du pot, touchee par le haut du chassis sur le seed 3).

Le masque est dans le repere map de slam_toolbox, c.-a-d. la pose de depart du robot.

    python3 masque_simu_officielle.py --seed 7 --sortie /tmp/masque_seed7
    ros2 launch nav_solution robot.launch.py sim:=true ... masque:=/tmp/masque_seed7.yaml
"""
import argparse
import json
import math
import os
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nav_solution'))
from nav_solution.generer_masque import ecrire_masque  # noqa: E402

MODELES = os.path.expanduser('~/parc2026_ws/src/PARC2026-Engineers-League/parc_robot_bringup/models')
DEMI_TABLE = 0.913 / 2
RESOLUTION = 0.05


def empreinte_maillage(modele):
    """Enveloppe convexe XY du maillage de collision (repere du modele)."""
    nom = os.path.join(MODELES, modele, 'meshes', modele + '_Col.obj')
    v = np.array([[float(t) for t in l.split()[1:3]] for l in open(nom) if l.startswith('v ')], np.float32)
    return cv2.convexHull(v).reshape(-1, 2).tolist()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, help='monde genere (phase2_hard)')
    ap.add_argument('--monde', default='/tmp/parc_worlds')
    ap.add_argument('--sdf', help='monde fixe (au lieu de --seed), ex. worlds/nba_sim.sdf')
    ap.add_argument('--verite', help='verite du monde fixe (depart, arene), ex. simu_officielle/arene_fixe.json')
    ap.add_argument('--sortie', required=True, help='prefixe des fichiers ecrits')
    ap.add_argument('--marge', type=float, default=0.05)
    a = ap.parse_args()
    sdf = open(a.sdf or f'{a.monde}/parc_random_seed{a.seed}.sdf').read()
    verite = json.load(open(a.verite or f'{a.monde}/parc_random_seed{a.seed}.json'))
    x0, y0, yaw0 = verite['start']['x'], verite['start']['y'], verite['start']['yaw']
    c0, s0 = math.cos(yaw0), math.sin(yaw0)

    def vers_map(wx, wy):
        dx, dy = wx - x0, wy - y0
        return [round(c0 * dx + s0 * dy, 3), round(-s0 * dx + c0 * dy, 3)]

    formes = {'cafe_table': [[-DEMI_TABLE, -DEMI_TABLE], [DEMI_TABLE, -DEMI_TABLE],
                             [DEMI_TABLE, DEMI_TABLE], [-DEMI_TABLE, DEMI_TABLE]]}
    polygones = []
    # <include> du monde genere (une ligne) ou du monde fixe (balises sur plusieurs lignes, autre ordre)
    for inc in re.findall(r'<include>(.*?)</include>', sdf, re.S):
        uri = re.search(r'<uri>model://([^<]+)</uri>', inc)
        pose = re.search(r'<pose>([^<]+)</pose>', inc)
        if not (uri and pose) or uri.group(1) not in ('cafe_table', 'WoodenChair', 'FemaleVisitorSit',
                                                       'MaleVisitorSit'):
            continue
        uri = uri.group(1)
        if uri not in formes:
            formes[uri] = empreinte_maillage(uri)
        x, y, _, _, _, yaw = (float(t) for t in pose.group(1).split())
        c, s = math.cos(yaw), math.sin(yaw)
        polygones.append([vers_map(x + c * px - s * py, y + s * px + c * py) for px, py in formes[uri]])
    # feuillage des plantes : sphere de 0.33 m au-dessus d'un pot de 0.20 m (le LiDAR ne voit que le pot)
    for modele in re.findall(r"<model name='obs\d+'>.*?</model>", sdf):
        pose = re.search(r'<pose>([^<]+)</pose>', modele)
        sphere = re.search(r'<sphere><radius>([^<]+)</radius>', modele)
        if not (pose and sphere):
            continue
        x, y = (float(t) for t in pose.group(1).split()[:2])
        r = float(sphere.group(1))
        polygones.append([vers_map(x + r * math.cos(k * math.pi / 8), y + r * math.sin(k * math.pi / 8))
                          for k in range(16)])

    # grille couvrant l'arene (murs compris), dans le repere map
    ar = verite['arena']
    coins = [vers_map(wx, wy) for wx in (ar['xmin'], ar['xmax']) for wy in (ar['ymin'], ar['ymax'])]
    mx = [p[0] for p in coins]
    my = [p[1] for p in coins]
    ox, oy = math.floor(min(mx)) - 1.0, math.floor(min(my)) - 1.0
    w = int(math.ceil((max(mx) + 1.0 - ox) / RESOLUTION))
    h = int(math.ceil((max(my) + 1.0 - oy) / RESOLUTION))
    info = {'resolution': RESOLUTION, 'origin': [ox, oy, 0.0]}
    ecrire_masque(polygones, info, (h, w), a.marge, a.sortie)


if __name__ == '__main__':
    main()
