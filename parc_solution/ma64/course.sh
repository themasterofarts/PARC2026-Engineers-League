#!/bin/bash
# MA64 : la course de la finale. Sur le robot, apres le colcon build d'Evans :
#   ~/teams/MA64/src/ma64/course.sh            (SIM=1 : repetition sur le simulateur du laptop)
# Depart : dire "allez", ou dans un 2e terminal : ~/teams/MA64/src/ma64/depart.sh
ICI="$(dirname "$(readlink -f "$0")")"; EQ="$(readlink -f "$ICI/../..")"
source /opt/ros/jazzy/setup.bash
if [ "${SIM:-0}" = 1 ]; then
  export ROS_DOMAIN_ID=64 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET GZ_IP=127.0.0.1 GZ_PARTITION=ma64
  source /home/mr-sun/PARC/PARC2026-Engineers-League/install/setup.bash
else
  [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
fi
source "$EQ/install/setup.bash"
[ -d "$ICI/pylib" ] && export PYTHONPATH="$ICI/pylib:$PYTHONPATH"
echo "MA64 : code de $(ros2 pkg prefix nav_solution)"
if [ "${SIM:-0}" != 1 ]; then
  # Cyclone : limite d'index de participant relevee (07/11 : "Failed to find a free participant index",
  # des noeuds ne demarraient pas avec les pilotes du robot et notre pile lances ensemble)
  export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
  # DDS : Cyclone (organisateurs), sinon FastDDS ; garde celui qui recoit vraiment /scan
  for r in rmw_cyclonedds_cpp rmw_fastrtps_cpp; do
    ros2 daemon stop > /dev/null 2>&1
    if RMW_IMPLEMENTATION=$r timeout 15 ros2 topic echo --once /scan sensor_msgs/msg/LaserScan --field header.frame_id > /dev/null 2>&1; then
      export RMW_IMPLEMENTATION=$r; echo $r > /tmp/ma64_rmw; echo "MA64 : DDS $r (recoit /scan)"; break
    fi
    echo "MA64 : $r ne recoit pas /scan"
  done
fi
# modele Vosk : dans src/ma64 ou dans le dossier de l'equipe (~/teams/MA64/vosk-model)
modele=""; for m in "$ICI/vosk-model" "$EQ/vosk-model"; do [ -f "$m/conf/model.conf" ] && { modele=$m; break; }; done
# depart vocal desactive pour la finale (decision du 07/11) ; VOCAL=1 pour le reactiver
vocal=""; [ "${VOCAL:-0}" = 1 ] && [ -n "$modele" ] && python3 -c "import vosk" 2>/dev/null && vocal="vocal:=true modele_vosk:=$modele"
indices=""; [ -f "$ICI/tags.yaml" ] && indices="indices:=$ICI/tags.yaml"
echo "MA64 : voix ${vocal:+oui}${vocal:-non (depart par depart.sh)} ; positions de tags $([ -n "$indices" ] && echo oui || echo non)"
# une seule manoeuvre de depart : l'ouverture du fichier de positions (points + tour de 360) si elle y
# est, sinon la manoeuvre simple (avance puis virage)
if [ -f "$ICI/tags.yaml" ] && grep -q "^ouverture:" "$ICI/tags.yaml"; then D_AV=0.0; D_VI=0; else D_AV=3.0; D_VI=90; fi
DEPART="ouverture_avance:=${AVANCE:-$D_AV} ouverture_virage:=${VIRAGE:-$D_VI}"
echo "MA64 : depart $( [ "$D_AV" = 0.0 ] && echo 'ouverture du fichier de positions' || echo "avance ${AVANCE:-$D_AV} m puis virage ${VIRAGE:-$D_VI} deg")"
if [ "${SIM:-0}" = 1 ]; then
  exec ros2 launch nav_solution robot.launch.py sim:=true camera:=camera1 tag_size:=0.12 \
    slam_params:="$ICI/slam_imu_sim.yaml" $DEPART $indices
else
  # camera : on choisit selon les IMAGES qui arrivent vraiment (creneau du 07/11 : camera_info a 15 Hz
  # mais 0 image brute au detecteur). Brute d'abord, sinon compressee + camera_info fixe.
  C=/camera2/camera2/color
  if [ "${CAMERA_SECOURS:-0}" != 1 ] && timeout 6 ros2 topic echo --once $C/image_raw sensor_msgs/msg/Image --field width > /dev/null 2>&1 \
     && timeout 6 ros2 topic echo --once $C/camera_info sensor_msgs/msg/CameraInfo --field width > /dev/null 2>&1; then
    cam="transport:=raw info_fixe:=false"
  elif timeout 6 ros2 topic echo --once $C/image_raw/compressed sensor_msgs/msg/CompressedImage --field format > /dev/null 2>&1; then
    cam="transport:=compressed info_fixe:=true"
  else
    cam="transport:=compressed info_fixe:=true"
    echo "MA64 : PROBLEME CAMERA2 : aucune image (ni brute ni compressee). Demander a Evans de relancer la camera."
  fi
  echo "MA64 : camera ${cam}"
  exec ros2 launch nav_solution robot.launch.py camera:=camera2 tag_size:=0.165 \
    $cam $DEPART $indices $vocal
fi
