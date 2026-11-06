#!/usr/bin/env python3
"""Genere terminator_ma64.conf : 3 fenetres, volets empiles dans l'ordre de lancement."""
import os

ICI = os.path.dirname(os.path.abspath(__file__))
FENETRES = [
    ('ma64_1_preparation', 'MA64 1/3 - Preparation', ['nettoyer', 'deploy', 'bag', 'diag', 'mouvement', 'web']),
    ('ma64_2_cartographie', 'MA64 2/3 - Cartographie', ['carto', 'etalon', 'teleop', 'sauver']),
    ('ma64_3_course', 'MA64 3/3 - Masque et course', ['masque', 'course', 'etat', 'depart', 'rapatrier']),
]
out = ['# Dispositions Terminator de l equipe MA64 (genere par generer_terminaux.py)',
       '[global_config]', '[keybindings]', '[profiles]', '  [[default]]', '    scrollback_lines = 5000', '[layouts]']
for nom, titre, etapes in FENETRES:
    out += [f'  [[{nom}]]', '    [[[fenetre]]]', '      type = Window', '      parent = ""', f'      title = {titre}',
            '      size = 900, 950']
    parent = 'fenetre'
    for k, e in enumerate(etapes):
        dernier = k == len(etapes) - 1
        if not dernier:
            sep = f'sep{k}'
            out += [f'    [[[{sep}]]]', '      type = VPaned', f'      parent = {parent}',
                    f'      order = {0 if parent == "fenetre" else 1}', f'      ratio = {1.0 / (len(etapes) - k):.3f}']
            t_parent, t_order = sep, 0
        else:
            t_parent, t_order = parent, 1
        out += [f'    [[[terminal_{e}]]]', '      type = Terminal', f'      parent = {t_parent}', f'      order = {t_order}',
                '      profile = default', f'      command = bash "{ICI}/volet.sh" {e}']
        if not dernier:
            parent = sep
open(os.path.join(ICI, 'terminator_ma64.conf'), 'w').write('\n'.join(out) + '\n')
print('terminator_ma64.conf : ' + ', '.join(f'{t} ({len(e)} volets)' for _, t, e in FENETRES))
