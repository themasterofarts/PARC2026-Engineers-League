#!/bin/bash
# Copie notre code sur le robot et le compile, dans le dossier de l'equipe ~/teams/MA64
# (consigne des organisateurs du 05/11 : seulement src, jamais build/ ni install/, ne rien
# toucher en dehors de ~/teams/MA64, compilation sur le robot).
# Le dossier envoye est prepare et verifie par preparer_envoi.sh (appele ici).
#
#   ROBOT=utilisateur@ip ./deploy.sh              # nav_solution + outils
#   ROBOT=utilisateur@ip ./deploy.sh --parc-msgs  # + notre copie de parc_msgs (SEULEMENT si le robot ne l'a pas)
#   ROBOT=utilisateur@ip ./deploy.sh --vocal      # + Vosk et le modele francais, dans ~/teams/MA64 (rien dans le systeme)
#
# --vocal lit sur le PC : VOSK_MODELE (defaut ~/parc_offline/vosk-model-small-fr-0.22)
#                         VOSK_PAQUETS (defaut ~/parc_offline/wheels_jetson, paquets aarch64 de vosk)
#
# Variables : ROBOT (obligatoire), RMW (couche DDS du robot, ex. rmw_cyclonedds_cpp, ecrite dans env.sh),
#             DOSSIER (defaut ~/teams/MA64 sur le robot),
#             BASE_SETUP (defaut ~/ros2_ws/install/setup.bash : espace de travail de base du
#             robot, ou les organisateurs ont installe parc_msgs et l'arbitre).
# Une seule connexion SSH est ouverte et reutilisee (un seul mot de passe).
set -e
: "${ROBOT:?Indique le robot : ROBOT=utilisateur@ip ./deploy.sh}"
DOSSIER=${DOSSIER:-teams/MA64}
ICI="$(dirname "$(readlink -f "$0")")"
SOLUTION="$(dirname "$ICI")"
SSH_OPTS=(-o ControlMaster=auto -o "ControlPath=$HOME/.ssh/cm-%r@%h:%p" -o ControlPersist=30m)
PARC_MSGS=0
VOCAL=0
VOSK_MODELE=${VOSK_MODELE:-$HOME/parc_offline/vosk-model-small-fr-0.22}
VOSK_PAQUETS=${VOSK_PAQUETS:-$HOME/parc_offline/wheels_jetson}
for a in "$@"; do
  case $a in
    --parc-msgs) PARC_MSGS=1 ;;
    --vocal) VOCAL=1 ;;
    *) echo "option inconnue : $a"; exit 1 ;;
  esac
done

copie() {  # copie <source> <destination distante>
  if ssh "${SSH_OPTS[@]}" "$ROBOT" command -v rsync >/dev/null 2>&1; then
    rsync -az --delete -e "ssh ${SSH_OPTS[*]}" --exclude '__pycache__' "$1" "$ROBOT:$2"
  else
    ssh "${SSH_OPTS[@]}" "$ROBOT" "rm -rf '$2' && mkdir -p '$(dirname "$2")'"
    scp -r -o ControlPath="$HOME/.ssh/cm-%r@%h:%p" "$1" "$ROBOT:$2"
  fi
}

"$ICI/preparer_envoi.sh"
E="$ICI/envoi_robot/MA64"

echo "== Connexion a $ROBOT"
ssh "${SSH_OPTS[@]}" "$ROBOT" "mkdir -p $DOSSIER/src $DOSSIER/outils $DOSSIER/journaux $DOSSIER/cartes"

echo "== Copie du code"
copie "$E/src/nav_solution/" "$DOSSIER/src/nav_solution/"
copie "$E/outils/diagnostic_robot.py" "$DOSSIER/outils/diagnostic_robot.py"
copie "$E/LISEZMOI.txt" "$DOSSIER/LISEZMOI.txt"
copie "$E/VERSION.txt" "$DOSSIER/VERSION.txt"
copie "$E/outils/distance_tag.py" "$DOSSIER/outils/distance_tag.py"
copie "$E/lancer.sh" "$DOSSIER/lancer.sh"
# finale : positions approximatives des tags + trajet d'ouverture (repere du robot au depart,
# coin sous le panier, tourne vers le gazon), genere par sim_outils/simu_officielle/arene_finale.py
copie "$SOLUTION/sim_outils/simu_officielle/arene_finale_indices.yaml" "$DOSSIER/cartes/finale_indices.yaml"
if [ $PARC_MSGS = 1 ]; then
  ssh "${SSH_OPTS[@]}" "$ROBOT" "mkdir -p $DOSSIER/parc_msgs_ws/src"
  copie "$SOLUTION/sim_outils/parc_msgs/" "$DOSSIER/parc_msgs_ws/src/parc_msgs/"
fi
if [ $VOCAL = 1 ]; then
  [ -f "$VOSK_MODELE/conf/model.conf" ] || { echo "modele Vosk introuvable : $VOSK_MODELE"; exit 1; }
  ls "$VOSK_PAQUETS"/vosk-*aarch64*.whl >/dev/null 2>&1 || { echo "paquet vosk aarch64 introuvable dans $VOSK_PAQUETS"; exit 1; }
  echo "== Copie de Vosk (modele francais et paquets aarch64)"
  copie "$VOSK_MODELE/" "$DOSSIER/vosk-model/"
  copie "$VOSK_PAQUETS/" "$DOSSIER/vosk_paquets/"
fi

echo "== Compilation sur le robot"
ssh "${SSH_OPTS[@]}" "$ROBOT" bash -s -- "$DOSSIER" "$PARC_MSGS" "${BASE_SETUP:-ros2_ws/install/setup.bash}" "${RMW:-}" "$VOCAL" <<'DISTANT'
set -e
D=$HOME/$1; PARC_MSGS=$2; BASE=$3; RMW=$4; VOCAL=$5
# couche DDS des pilotes du robot (organisateurs : Cyclone) : sinon nos noeuds ne les voient pas
[ -n "$RMW" ] && export RMW_IMPLEMENTATION=$RMW
if [ "$VOCAL" = 1 ]; then
  # installe dans le dossier de l'equipe seulement, sans internet
  python3 -m pip install --quiet --no-index --find-links "$D/vosk_paquets" --target "$D/pylib" --upgrade \
      vosk cffi pycparser srt tqdm requests websockets charset_normalizer idna urllib3 certifi \
    && echo "vosk installe dans $D/pylib" || echo "ATTENTION : installation de vosk echouee (depart par commande seulement)"
fi
case $BASE in /*) ;; *) BASE=$HOME/$BASE ;; esac
source /opt/ros/jazzy/setup.bash
[ -n "$BASE" ] && [ -f "$BASE" ] && source "$BASE"
if [ "$PARC_MSGS" = 1 ]; then
  (cd "$D/parc_msgs_ws" && colcon build --packages-select parc_msgs > build.log 2>&1 && echo "parc_msgs compile") \
    || { echo "ECHEC compilation parc_msgs : voir $D/parc_msgs_ws/build.log"; exit 1; }
fi
[ -f "$D/parc_msgs_ws/install/local_setup.bash" ] && source "$D/parc_msgs_ws/install/local_setup.bash"
cd "$D" && colcon build --packages-select nav_solution > build.log 2>&1 \
  || { tail -20 build.log; echo "ECHEC compilation nav_solution"; exit 1; }
echo "nav_solution compile"
# env.sh : tout ce qu'il faut sourcer avant de lancer quoi que ce soit
cat > "$D/env.sh" <<EOF
${RMW:+export RMW_IMPLEMENTATION=$RMW}
source /opt/ros/jazzy/setup.bash
# DDS : le choix teste par "lancer.sh rmw" (rmw.txt), sinon Cyclone s'il est installe, sinon FastDDS
if [ -f "$D/rmw.txt" ]; then export RMW_IMPLEMENTATION=\$(cat "$D/rmw.txt")
elif ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; fi
# Cyclone : limite d'index de participant relevee (creneau du 07/11 : "Failed to find a free participant
# index", teleop / vocal / map_saver ne demarraient plus avec les pilotes et la carto lances)
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
if [ -n "$BASE" ] && [ -f "$BASE" ]; then source "$BASE"; fi
if [ -f "$D/parc_msgs_ws/install/local_setup.bash" ]; then source "$D/parc_msgs_ws/install/local_setup.bash"; fi
source "$D/install/local_setup.bash"
if [ -d "$D/pylib" ]; then export PYTHONPATH="$D/pylib:\$PYTHONPATH"; fi
true
EOF
echo "== Pret. Sur le robot : source ~/$1/env.sh"
[ -f "$BASE" ] && echo "espace de base : $BASE" || echo "ATTENTION espace de base introuvable : $BASE (BASE_SETUP=...)"
source "$D/env.sh"
echo "== Verification de ce dont notre code a besoin"
manque=()
for p in parc_msgs nav2_bringup nav2_amcl nav2_map_server nav2_regulated_pure_pursuit_controller \
         nav2_smac_planner dwb_core opennav_docking slam_toolbox laser_filters apriltag_ros \
         apriltag_msgs teleop_twist_keyboard; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || manque+=("$p")
done
python3 -c "import cv2" 2>/dev/null || manque+=("python3-opencv (module cv2, pour le JPEG des visites)")
[ -n "$RMW" ] && { ros2 pkg prefix "$RMW" >/dev/null 2>&1 || manque+=("$RMW (couche DDS du robot)"); }
[ -n "$RMW" ] && echo "couche DDS : $RMW" && ros2 daemon stop >/dev/null 2>&1
python3 -c "import numpy, yaml" 2>/dev/null || manque+=("python3-numpy / python3-yaml")
command -v tmux >/dev/null || manque+=("tmux")
if [ ${#manque[@]} -eq 0 ]; then
  echo "tout est present"
else
  echo "ATTENTION, il manque : ${manque[*]}"
  echo "-> demander aux organisateurs AVANT d'installer quoi que ce soit"
fi
DISTANT
