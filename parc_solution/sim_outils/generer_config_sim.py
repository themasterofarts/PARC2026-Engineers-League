#!/usr/bin/env python3
"""Genere les fichiers *_robot_sim.yaml (robot.launch.py sim:=true) depuis ceux du vrai robot.

Le simulateur officiel PARC 2026 publie les memes topics que le robot, avec trois differences :
  - TF odom -> base_footprint -> base_link, ou base_link est tourne de 90 deg : le repere qui
    regarde vers l'avant est base_footprint (sur le robot, c'est base_link) ;
  - horloge simulee (use_sim_time) ;
  - commandes Twist sur /cmd_vel (sur le robot : TwistStamped sur /botwheel_explorer/cmd_vel).
Seules ces valeurs changent : la simulation teste la configuration du robot.
A relancer apres toute modification d'un fichier *_robot.yaml.
"""
import os

import yaml

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nav_solution', 'config')
ENTETE = ('# GENERE par sim_outils/generer_config_sim.py depuis {src} : ne pas modifier a la main.\n'
          '# Simulateur officiel : base_footprint au lieu de base_link, use_sim_time, Twist sur /cmd_vel.\n')


def remplace(o):
    if isinstance(o, dict):
        for k, v in list(o.items()):
            if v == 'base_link':
                o[k] = 'base_footprint'
            elif k == 'use_sim_time':
                o[k] = True
            elif k == 'enable_stamped_cmd_vel':
                o[k] = False
            elif v == '/botwheel_explorer/cmd_vel':
                o[k] = '/cmd_vel'
            elif k == 'cmd_vel_stamped':
                o[k] = False
            else:
                remplace(v)
    elif isinstance(o, list):
        for x in o:
            remplace(x)


for nom in ('nav2_params_robot', 'slam_toolbox_robot', 'lasers_scan_filters_robot', 'amcl_robot', 'mission_robot'):
    src = os.path.join(CONFIG, nom + '.yaml')
    d = yaml.safe_load(open(src))
    remplace(d)
    if nom == 'mission_robot':
        d['mission']['ros__parameters']['use_sim_time'] = True
    with open(os.path.join(CONFIG, nom + '_sim.yaml'), 'w') as f:
        f.write(ENTETE.format(src=nom + '.yaml'))
        yaml.safe_dump(d, f, sort_keys=False, default_flow_style=None, width=110)
    print(nom + '_sim.yaml')
