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
  # DDS : Cyclone (organisateurs), sinon FastDDS ; garde celui qui recoit vraiment /scan
  for r in rmw_cyclonedds_cpp rmw_fastrtps_cpp; do
    ros2 daemon stop > /dev/null 2>&1
    if RMW_IMPLEMENTATION=$r timeout 15 ros2 topic echo --once /scan --field header.frame_id > /dev/null 2>&1; then
      export RMW_IMPLEMENTATION=$r; echo $r > /tmp/ma64_rmw; echo "MA64 : DDS $r (recoit /scan)"; break
    fi
    echo "MA64 : $r ne recoit pas /scan"
  done
fi
vocal=""; [ -d "$ICI/vosk-model" ] && python3 -c "import vosk" 2>/dev/null && vocal="vocal:=true modele_vosk:=$ICI/vosk-model"
indices=""; [ -f "$ICI/tags.yaml" ] && indices="indices:=$ICI/tags.yaml"
echo "MA64 : voix ${vocal:+oui}${vocal:-non (depart par depart.sh)} ; positions de tags ${indices:+oui}${indices:-non}"
DEPART="ouverture_avance:=${AVANCE:-3.0} ouverture_virage:=${VIRAGE:-90}"
if [ "${SIM:-0}" = 1 ]; then
  exec ros2 launch nav_solution robot.launch.py sim:=true camera:=camera1 tag_size:=0.12 \
    slam_params:="$ICI/slam_imu_sim.yaml" $DEPART $indices
else
  exec ros2 launch nav_solution robot.launch.py camera:=camera2 tag_size:=0.165 \
    transport:=compressed info_fixe:=true $DEPART $indices $vocal
fi
