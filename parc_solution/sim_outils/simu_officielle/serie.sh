#!/bin/bash
# Serie de courses enchainees, sans fenetres, puis tableau recapitulatif.
#   serie.sh NOM "SEED[:simple] ..." [args robot.launch...]
#   ex. : serie.sh nonreg "3 12 7 21:simple" gyro:=true
# Variables passees a course.sh : MASQUE (defaut 0 : conditions des juges), ORDRE.
# Une course bloquee est arretee au bout de LIMITE_MIN minutes (defaut 40) : l'evaluateur ecrit
# quand meme son rapport. Resultats : ~/ma64_courses_simu/NOM_<seed>/ et ~/ma64_courses_simu/NOM.md
ICI="$(dirname "$(readlink -f "$0")")"
R=$HOME/ma64_courses_simu
NOM=$1; SEEDS=$2; shift 2
export MASQUE=${MASQUE:-0}
for s in $SEEDS; do
  seed=${s%%:*}; simple=0; [ "$s" != "$seed" ] && simple=1
  n=${NOM}_${seed}$([ $simple = 1 ] && echo simple)
  echo "$(date +%T) course $n"
  # demon ROS fige apres un changement de reseau : la course attendait /clock sans fin
  (source "$ICI/env.sh" > /dev/null 2>&1; ros2 daemon stop > /dev/null 2>&1)
  SIMPLE=$simple "$ICI/course.sh" "$seed" "$n" "$@" > "$R/$n.out" 2>&1 &
  pid=$!
  debut=$(date +%s)
  while kill -0 $pid 2>/dev/null; do
    sleep 20
    if [ $(( $(date +%s) - debut )) -gt $(( ${LIMITE_MIN:-40} * 60 )) ]; then
      echo "$(date +%T) $n : limite de temps, arret"
      for p in $(pgrep -f "evaluateur_sim\.py"); do kill -INT $p; done   # rapport, puis fin de course.sh
      sleep 30
    fi
  done
done
python3 - "$R" "$NOM" $SEEDS <<'PY'
import json, os, sys
R, nom, seeds = sys.argv[1], sys.argv[2], sys.argv[3:]
lignes = ['| Course | Tags | Signales | Duree simulee (s) | Temps arbitre (s) | Contacts | Inclinaison max |',
          '|---|---|---|---|---|---|---|']
for s in seeds:
    seed = s.split(':')[0]
    n = f"{nom}_{seed}" + ('simple' if ':' in s else '')
    f = os.path.join(R, n, 'evaluation.json')
    if not os.path.exists(f):
        lignes.append(f'| {n} | pas de rapport | | | | | |')
        continue
    r = json.load(open(f))
    c = {}
    for k, v in r.get('contacts_hors_sol', {}).items():   # l'objet touche, pas le robot
        autre = [x for x in k.split(' / ') if not x.startswith('sitoe_robot')] or [k]
        c[autre[0].split('::')[0]] = v
    lignes.append(f"| {n} | {len(r.get('tags_signales') or [])}/4 | {r.get('tags_signales')} | "
                  f"{r.get('duree_simulee_s')} | {r.get('temps_course_s')} | {c or 'aucun'} | {r.get('inclinaison_max_deg')} |")
open(os.path.join(R, nom + '.md'), 'w').write('\n'.join(lignes) + '\n')
print('\n'.join(lignes))
PY
