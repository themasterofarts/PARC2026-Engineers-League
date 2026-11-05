#!/bin/bash
# Prepare le dossier a envoyer sur le robot, puis le verifie.
#
#   ./preparer_envoi.sh            # cree outils_robot/envoi_robot/MA64 et MA64.tar.gz
#
# Contenu (ce qui ira dans ~/teams/MA64 sur le robot, consigne des organisateurs du 05/11 :
# seulement src, jamais build/ ni install/, compile sur le robot depuis ~/teams/MA64) :
#   src/nav_solution/      notre paquet ROS 2 (code, launch, config)
#   outils/diagnostic_robot.py, outils/distance_tag.py (etalonnage)
#   lancer.sh              les etapes du creneau (appele par les volets Terminator du PC)
#   LISEZMOI.txt           les commandes du creneau
#   VERSION.txt            commit git, date, empreinte de chaque fichier
# Verifications : compilation dans un espace propre, executables, chargement des launch,
# lecture des YAML, analyse des fichiers Python. Le script s'arrete a la premiere erreur.
set -e
ICI="$(dirname "$(readlink -f "$0")")"
SOLUTION="$(dirname "$ICI")"
ENVOI="$ICI/envoi_robot"
D="$ENVOI/MA64"

echo "== Preparation de $D"
rm -rf "$ENVOI"
mkdir -p "$D/src" "$D/outils"
# envoi_robot est dans src/ : sans ceci, colcon y verrait un second paquet nav_solution
touch "$ENVOI/COLCON_IGNORE"
# copie du paquet sans les fichiers generes ni les tests de style
tar -C "$SOLUTION" --exclude='__pycache__' --exclude='*.pyc' --exclude='nav_solution/test' \
    -cf - nav_solution | tar -C "$D/src" -xf -
cp "$ICI/diagnostic_robot.py" "$ICI/distance_tag.py" "$D/outils/"
cp "$ICI/lancer.sh" "$D/" && chmod +x "$D/lancer.sh"

cat > "$D/LISEZMOI.txt" <<'EOF'
Equipe MA64 - dossier ~/teams/MA64 sur le robot (ne rien toucher en dehors). Detail : Ressources/GUIDE_ROBOT_JEUDI.md

Compiler (deploy.sh le fait) :
  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
  cd ~/teams/MA64 && colcon build --packages-select nav_solution
Dans chaque terminal (tmux) :
  source ~/teams/MA64/env.sh
Diagnostic :
  cd ~/teams/MA64/outils && python3 diagnostic_robot.py && python3 diagnostic_robot.py --mouvement --no-bag
Cartographie (teleop) :
  ros2 launch nav_solution robot.launch.py mission:=false nav2:=false camera:=camera2 tag_size:=0.165
  (etalonner : tag_size x distance ruban / distance camera, via ros2 run tf2_ros tf2_echo base_link tag_4)
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/botwheel_explorer/cmd_vel -p stamped:=true -p frame_id:=base_link -p speed:=0.15 -p turn:=0.5
  ros2 run nav2_map_server map_saver_cli -f ~/teams/MA64/cartes/arene
  ros2 run nav_solution sauver_tags ~/teams/MA64/cartes/tags.yaml
Course (robot sur le point de depart marque) :
  ros2 launch nav_solution robot.launch.py camera:=camera2 tag_size:=TAILLE_ETALONNEE indices:=$HOME/teams/MA64/cartes/tags.yaml
  ros2 topic echo /parc/run_status
  ros2 topic pub --once /ma64/start std_msgs/msg/String "{data: go}"     (quand l'etat est ARMED)
Ne jamais lancer faux_arbitre sur le robot. Arret d'urgence : Ctrl+C.
EOF

{
  echo "Prepare le $(date '+%Y-%m-%d %H:%M:%S')"
  ( cd "$SOLUTION" && echo "Branche $(git branch --show-current 2>/dev/null), commit $(git rev-parse --short HEAD 2>/dev/null)" \
    && [ -n "$(git status --porcelain -- nav_solution outils_robot/diagnostic_robot.py 2>/dev/null)" ] \
    && echo "ATTENTION : modifications non commitees incluses" ) || true
  echo
  ( cd "$D" && find src outils lancer.sh -type f | sort | xargs md5sum )
} > "$D/VERSION.txt"

echo "== Verification 1 : compilation dans un espace propre"
TMP=$(mktemp -d /tmp/ma64_verif_XXXX)
mkdir -p "$TMP/ws" && cp -r "$D/src" "$TMP/ws/"
( source /opt/ros/jazzy/setup.bash && cd "$TMP/ws" && colcon build --packages-select nav_solution > build.log 2>&1 ) \
  || { tail -20 "$TMP/ws/build.log"; echo "ECHEC compilation"; exit 1; }
echo "   ok"

echo "== Verification 2 : executables et launch"
(
  set -e
  source /opt/ros/jazzy/setup.bash
  source "$TMP/ws/install/local_setup.bash"
  attendus="apriltag_tf_node faux_arbitre mission_node sauver_tags generer_masque demarrage_vocal"
  presents=$(ros2 pkg executables nav_solution | awk '{print $2}' | tr '\n' ' ')
  for e in $attendus; do
    echo " $presents " | grep -q " $e " || { echo "ECHEC executable manquant : $e"; exit 1; }
  done
  echo "   executables : $presents"
  for l in robot.launch.py localisation.launch.py apriltag.launch.py masque.launch.py navigation_ma64.launch.py; do
    ros2 launch nav_solution "$l" --show-args > /dev/null 2>&1 || { echo "ECHEC chargement $l"; exit 1; }
  done
  echo "   launch du robot : ok"
)

echo "== Verification 3 : fichiers de configuration du robot"
python3 - "$D/src/nav_solution/config" <<'PY'
import sys, yaml, os
d = sys.argv[1]
for f in ('mission_robot.yaml', 'nav2_params_robot.yaml', 'slam_toolbox_robot.yaml', 'amcl_robot.yaml',
          'lasers_scan_filters_robot.yaml', 'apriltag.yaml'):
    yaml.safe_load(open(os.path.join(d, f)))
m = yaml.safe_load(open(os.path.join(d, 'mission_robot.yaml')))['mission']['ros__parameters']
n = yaml.safe_load(open(os.path.join(d, 'nav2_params_robot.yaml')))
cm = n['collision_monitor']['ros__parameters']
erreurs = []
if m.get('cmd_vel_topic') != '/botwheel_explorer/cmd_vel' or not m.get('cmd_vel_stamped'):
    erreurs.append('mission : commandes pas en TwistStamped sur /botwheel_explorer/cmd_vel')
if cm.get('cmd_vel_out_topic') != '/botwheel_explorer/cmd_vel':
    erreurs.append('collision_monitor : sortie pas sur /botwheel_explorer/cmd_vel')
for k in ('controller_server', 'velocity_smoother', 'collision_monitor', 'behavior_server', 'docking_server'):
    if not n[k]['ros__parameters'].get('enable_stamped_cmd_vel'):
        erreurs.append(f'{k} : enable_stamped_cmd_vel absent')
lignes = [l for l in open(os.path.join(d, 'nav2_params_robot.yaml')) if not l.lstrip().startswith('#')]
if any('base_footprint' in l for l in lignes):
    erreurs.append('nav2_params_robot : base_footprint au lieu de base_link')
if erreurs:
    print('ECHEC', *erreurs, sep='\n   ')
    sys.exit(1)
print('   ok : YAML lisibles, TwistStamped sur /botwheel_explorer/cmd_vel, base_link')
PY

echo "== Verification 4 : analyse des fichiers Python"
if python3 -m pyflakes --version > /dev/null 2>&1; then
  python3 -m pyflakes "$D/src/nav_solution/nav_solution/"{mission_node,apriltag_tf_node,sauver_tags,faux_arbitre,generer_masque,demarrage_vocal}.py \
    "$D/outils/diagnostic_robot.py" "$D/outils/distance_tag.py" && bash -n "$D/lancer.sh" && echo "   ok"
else
  python3 -m py_compile "$D/src/nav_solution/nav_solution/"*.py "$D/outils/diagnostic_robot.py" && echo "   ok (syntaxe)"
fi
rm -rf "$TMP"

tar -C "$ENVOI" -czf "$ENVOI/MA64.tar.gz" MA64
echo
echo "== Pret : $D ($(du -sh "$D" | cut -f1)), archive $ENVOI/MA64.tar.gz"
echo "   Envoi : ROBOT=utilisateur@ip ./deploy.sh   (ou a la main : scp MA64.tar.gz, puis sur le robot : cd ~/teams && tar xzf ~/MA64.tar.gz)"
