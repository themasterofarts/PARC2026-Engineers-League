#!/usr/bin/env python3
"""Replique de l'arene de la finale (photos du 6 nov ; dimensions des organisateurs).

  arene_finale.py            ecrit arene_finale.sdf et arene_finale.json (verite pour evaluateur_sim.py)
                             depuis arene_finale_config.json
  arene_finale.py --importer relit les poses dans Gazebo ouvert (objets deplaces a la souris),
                             met a jour arene_finale_config.json, puis regenere le monde

Sur (organisateurs) : arene 7.00 x 6.10 m, table 0.70 x 0.50 x 0.76 m, petit plot 0.15 x 0.25 m, grand 0.48 x 0.78 m.
Estime sur les photos : tout le reste (positions, sens des tags, depart).

Repere de la config (local) : x vers l'est (mur du pied Spalding), y vers le nord (mur du fond, conteneur bleu),
origine au coin interieur sud-ouest (mur de l'affiche PARC / gazon). Le simulateur officiel pose toujours le robot
en (0, -7) cap 1.571 : le monde est tourne et decale pour que le depart de la config tombe a cet endroit.
La marque rouge au sol (marque_depart, sans collision) montre le depart et son sens.
"""
import json
import math
import os
import re
import subprocess
import sys

ICI = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(ICI, 'arene_finale_config.json')
MONDE = 'arene_finale'
SPAWN = (0.0, -7.0, 1.571)

BOIS = '<material><ambient>0.76 0.6 0.42 1</ambient><diffuse>0.76 0.6 0.42 1</diffuse></material>'
NOIR = '<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>'
ORANGE = '<material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material>'
ROUGE = '<material><ambient>0.9 0 0 1</ambient><diffuse>0.9 0 0 1</diffuse></material>'


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Repere:
    """local (config) -> monde : le depart local va sur le point d'apparition du robot."""

    def __init__(self, depart):
        dx, dy, dcap = depart
        self.r = SPAWN[2] - dcap
        c, s = math.cos(self.r), math.sin(self.r)
        self.t = (SPAWN[0] - (c * dx - s * dy), SPAWN[1] - (s * dx + c * dy))

    def monde(self, x, y, cap=0.0):
        c, s = math.cos(self.r), math.sin(self.r)
        return c * x - s * y + self.t[0], s * x + c * y + self.t[1], wrap(cap + self.r)

    def local(self, x, y, cap=0.0):
        c, s = math.cos(-self.r), math.sin(-self.r)
        x, y = x - self.t[0], y - self.t[1]
        return c * x - s * y, s * x + c * y, wrap(cap - self.r)


def boite(nom, pose, taille, mat, collision=True):
    g = f'<geometry><box><size>{taille}</size></box></geometry>'
    col = f"<collision name='c_{nom}'><pose>{pose}</pose>{g}</collision>" if collision else ''
    return col + f"<visual name='v_{nom}'><pose>{pose}</pose>{g}{mat}</visual>"


def modele(nom, pose_monde, corps):
    x, y, cap = pose_monde
    return (f"<model name='{nom}'><static>true</static><pose>{x:.4f} {y:.4f} 0 0 0 {cap:.5f}</pose>"
            f"<link name='link'>{corps}</link></model>\n")


def tag(i, pm):
    # meme panneau que les mondes generes (phase2_hard) : poteau, plaque, tag sur la face +x
    tex = f'model://Apriltag36_11_{i:05d}/materials/textures/tag36_11_{i:05d}.png'
    corps = (boite('poteau', '-0.025 0 0.575 0 0 0', '0.03 0.03 1.15', BOIS)
             + boite('plaque', '0 0 1.0 0 0 0', '0.012 0.25 0.25',
                     '<material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material>')
             + "<visual name='tag'><pose>0.0066 0 1.0 3.14159 0 0</pose><geometry><box><size>0.001 0.15 0.15</size>"
               "</box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal>"
               f"<albedo_map>{tex}</albedo_map><roughness>0.5</roughness><metalness>0.0</metalness></metal></pbr>"
               "</material></visual>")
    return modele(f'tag_sign_{i}', pm, corps)


def table(n, pm):
    # plateau 0.70 x 0.50 a 0.76 m ; de chaque cote un cadre noir : deux montants et un patin au sol
    c = boite('plateau', '0 0 0.745 0 0 0', '0.70 0.50 0.03', BOIS)
    for s in (-1, 1):
        for t in (-1, 1):
            c += boite(f'pied{s}{t}', f'{0.31 * s} {0.21 * t} 0.365 0 0 0', '0.03 0.03 0.73', NOIR)
        c += boite(f'patin{s}', f'{0.31 * s} 0 0.015 0 0 0', '0.03 0.50 0.03', NOIR)
    return modele(f'table_{n}', pm, c)


def plot(n, pm, r, h):
    g = f'<geometry><cone><radius>{r}</radius><length>{h}</length></cone></geometry>'
    c = (f"<collision name='c'><pose>0 0 {h / 2} 0 0 0</pose>{g}</collision>"
         f"<visual name='v'><pose>0 0 {h / 2} 0 0 0</pose>{g}{ORANGE}</visual>")
    return modele(f'plot_{n}', pm, c)


def generer(cfg):
    R = Repere(cfg['depart'])
    LX, LY = cfg['dimensions']
    corps = ''
    for nom, x, y, t in (('sud', LX / 2, -0.01, f'{LX + 0.04} 0.02 0.4'), ('nord', LX / 2, LY + 0.01, f'{LX + 0.04} 0.02 0.4'),
                         ('ouest', -0.01, LY / 2, f'0.02 {LY} 0.4'), ('est', LX + 0.01, LY / 2, f'0.02 {LY} 0.4')):
        corps += modele(f'wall_{nom}', R.monde(x, y), boite('mur', '0 0 0.2 0 0 0', t, BOIS))
    # affiche PARC sur le mur avant (repere visuel seulement, sans collision)
    corps += modele('affiche_parc', R.monde(LX / 2, -0.03),
                    boite('affiche', '0 0 0.2 0 0 0', '0.8 0.01 0.25', ROUGE, collision=False))
    # depart : fleche plate au sol (1 mm, sans collision : invisible pour le LiDAR)
    corps += modele('marque_depart', R.monde(*cfg['depart']),
                    boite('corps', '0 0 0.001 0 0 0', '0.4 0.06 0.002', ROUGE, collision=False)
                    + boite('pointe', '0.2 0 0.001 0 0 0.785', '0.12 0.12 0.002', ROUGE, collision=False))
    for i, (x, y, c) in cfg['tags'].items():
        corps += tag(int(i), R.monde(x, y, c))
    for n, (x, y, c) in enumerate(cfg['tables']):
        corps += table(n, R.monde(x, y, c))
    for n, (x, y, r, h) in enumerate(cfg['plots']):
        corps += plot(n, R.monde(x, y), r, h)

    entete = open('/tmp/parc_worlds/parc_random_seed3.sdf').read().split('\n')[:39]   # physique, capteurs, sol, soleil
    entete = '\n'.join(entete).replace("world name='parc_random'", f"world name='{MONDE}'")
    open(os.path.join(ICI, 'arene_finale.sdf'), 'w').write(entete + '\n' + corps + '  </world>\n</sdf>\n')

    coins = [R.monde(x, y)[:2] for x, y in ((0, 0), (LX, 0), (0, LY), (LX, LY))]
    tags = []
    for i, (x, y, c) in cfg['tags'].items():
        wx, wy, wc = R.monde(x, y, c)
        tags.append({'id': int(i), 'x': round(wx, 3), 'y': round(wy, 3), 'facing_yaw': round(wc, 4), 'mount': 'free',
                     'visit_pose': [round(wx + 0.5 * math.cos(wc), 3), round(wy + 0.5 * math.sin(wc), 3),
                                    round(wrap(wc + math.pi), 4)]})
    verite = {
        '_source': 'arene_finale.py (replique estimee de l arene finale), depuis arene_finale_config.json',
        'arena': {'xmin': round(min(p[0] for p in coins), 3), 'xmax': round(max(p[0] for p in coins), 3),
                  'ymin': round(min(p[1] for p in coins), 3), 'ymax': round(max(p[1] for p in coins), 3)},
        'start': {'x': SPAWN[0], 'y': SPAWN[1], 'yaw': SPAWN[2]},
        'tags': tags,
        'obstacles': ([dict(zip(('kind', 'x', 'y', 'yaw'), ('table',) + tuple(round(v, 3) for v in R.monde(*t))))
                       for t in cfg['tables']]
                      + [dict(zip(('kind', 'x', 'y'), ('plot',) + tuple(round(v, 3) for v in R.monde(p[0], p[1])[:2])),
                              r=p[2], h=p[3]) for p in cfg['plots']]),
    }
    json.dump(verite, open(os.path.join(ICI, 'arene_finale.json'), 'w'), indent=1)
    ecrire_indices(cfg)
    print('arene_finale.sdf, arene_finale.json et arene_finale_indices.yaml ecrits')


def ecrire_indices(cfg):
    """Indices pour mission_node (indices:=...) dans le repere du robot au depart (= repere map du
    SLAM) : x devant le robot, y a sa gauche. tags : [x, y, nx, ny] (n : sens de la face imprimee),
    ouverture : points [x, y]. Valable aussi sur le vrai robot s'il part de la meme pose."""
    x0, y0, c0 = cfg['depart']

    def rep(x, y):
        dx, dy = x - x0, y - y0
        return math.cos(c0) * dx + math.sin(c0) * dy, -math.sin(c0) * dx + math.cos(c0) * dy

    lignes = ['# genere par sim_outils/simu_officielle/arene_finale.py depuis arene_finale_config.json',
              '# repere : robot au depart, x devant lui, y a sa gauche (m)', 'tags:']
    for i, (x, y, c) in sorted(cfg['tags'].items(), key=lambda kv: int(kv[0])):
        rx, ry = rep(x, y)
        lignes.append(f'  {int(i)}: [{rx:.3f}, {ry:.3f}, {math.cos(c - c0):.4f}, {math.sin(c - c0):.4f}]')
    lignes.append('ouverture:')
    for x, y in cfg.get('ouverture', []):
        rx, ry = rep(x, y)
        lignes.append(f'  - [{rx:.3f}, {ry:.3f}]')
    open(os.path.join(ICI, 'arene_finale_indices.yaml'), 'w').write('\n'.join(lignes) + '\n')

    # masque keepout des plateaux de table (le LiDAR ne voit que les pieds), meme repere : masque:=...
    sys.path.insert(0, os.path.join(ICI, '..', '..', 'nav_solution'))
    from nav_solution.generer_masque import ecrire_masque
    polygones = []
    for x, y, c in cfg['tables']:
        coins = [(dx * math.cos(c) - dy * math.sin(c), dx * math.sin(c) + dy * math.cos(c))
                 for dx, dy in ((-0.35, -0.25), (0.35, -0.25), (0.35, 0.25), (-0.35, 0.25))]
        polygones.append([list(map(lambda v: round(v, 3), rep(x + px, y + py))) for px, py in coins])
    LX, LY = cfg['dimensions']
    pts = [rep(x, y) for x in (0, LX) for y in (0, LY)]
    ox = math.floor(min(p[0] for p in pts)) - 1.0
    oy = math.floor(min(p[1] for p in pts)) - 1.0
    w = int(math.ceil((max(p[0] for p in pts) + 1.0 - ox) / 0.05))
    h = int(math.ceil((max(p[1] for p in pts) + 1.0 - oy) / 0.05))
    ecrire_masque(polygones, {'resolution': 0.05, 'origin': [ox, oy, 0.0]}, (h, w), 0.05,
                  os.path.join(ICI, 'arene_finale_masque'))


def importer(cfg):
    """Poses actuelles des objets dans Gazebo ouvert -> config locale."""
    sortie = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{MONDE}/pose/info'],
                            capture_output=True, text=True, timeout=20).stdout
    poses = {}
    for bloc in sortie.split('pose {')[1:]:
        nom = re.search(r'name: "([^"]+)"', bloc)
        if not nom:
            continue
        val = lambda grp, k: float((re.search(grp + r'\s*\{[^}]*?' + k + r': (-?[\d.e-]+)', bloc) or [0, 0])[1])
        qz, qw = val('orientation', 'z'), val('orientation', 'w') or 1.0
        poses[nom[1]] = (val('position', 'x'), val('position', 'y'), 2 * math.atan2(qz, qw))
    if not poses:
        sys.exit('aucune pose lue : Gazebo est-il ouvert sur arene_finale.sdf (meme GZ_PARTITION) ?')
    ancien = Repere(cfg['depart'])
    loc = {n: ancien.local(*p) for n, p in poses.items()}
    r = lambda v: round(v, 3)
    if 'marque_depart' in loc:
        cfg['depart'] = [r(v) for v in loc['marque_depart']]
    for i in list(cfg['tags']):
        if f'tag_sign_{i}' in loc:
            cfg['tags'][i] = [r(v) for v in loc[f'tag_sign_{i}']]
    cfg['tables'] = [[r(v) for v in loc[f'table_{n}']] if f'table_{n}' in loc else t for n, t in enumerate(cfg['tables'])]
    cfg['plots'] = [[r(loc[f'plot_{n}'][0]), r(loc[f'plot_{n}'][1]), p[2], p[3]] if f'plot_{n}' in loc else p
                    for n, p in enumerate(cfg['plots'])]
    json.dump(cfg, open(CONFIG, 'w'), indent=1)
    print('arene_finale_config.json mis a jour :\n' + json.dumps(cfg, indent=1))


if __name__ == '__main__':
    cfg = json.load(open(CONFIG))
    if '--importer' in sys.argv:
        importer(cfg)
    generer(cfg)
