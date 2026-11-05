#!/bin/bash
# Banc de test de la localisation : simulation (monde simple, seed 21) + SLAM seul (sans Nav2 ni
# mission), parcours fixe sur /cmd_vel, erreur SLAM / verite Gazebo (test_localisation.py).
#   test_loc.sh NOM [args robot.launch...]     (resultats dans ~/ma64_courses_simu/NOM)
ICI="$(dirname "$(readlink -f "$0")")"
R=$HOME/ma64_courses_simu
NOM=$1; shift
D=$R/$NOM; mkdir -p $D
source $ICI/env.sh
SOL=/home/peace-fiacre/Projects/parc_ingé_league/src/parc_solution
nettoie() {
  for p in $(pgrep -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (launch|run)"); do kill -INT $p 2>/dev/null; done
  sleep 8
  for p in $(pgrep -f "^gz sim"); do kill $p; done
  for p in $(ps -eo pid,args | grep -E "^ *[0-9]+ (/opt/ros/jazzy/lib/|python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /home/peace-fiacre/parc2026_ws/install)" | grep -v grep | awk '{print $1}'); do kill $p 2>/dev/null; done
  sleep 3
  for p in $(ps -eo pid,args | grep -E "^ *[0-9]+ (/opt/ros/jazzy/lib/|gz sim|/bin/sh -c ruby|python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (launch|run))" | grep -v grep | awk '{print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 1
}
nettoie
echo "$(date +%T) simulation (monde simple, seed 21)"
ros2 launch $ICI/phase2_simple.launch.py seed:=21 gui:=false rviz:=false audio:=false > $D/sim.log 2>&1 &
until timeout 5 ros2 topic echo --once /clock --field clock.sec > /dev/null 2>&1; do sleep 3; done
ros2 run ros_gz_bridge parameter_bridge "/model/sitoe_robot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose" --ros-args -r /model/sitoe_robot/pose:=/verite/robot_pose -r __node:=pont_verite > $D/pont_verite.log 2>&1 &
echo "$(date +%T) SLAM seul $*"
ros2 launch nav_solution robot.launch.py sim:=true camera:=camera1 tag_size:=0.12 mission:=false nav2:=false "$@" > $D/robot.log 2>&1 &
until grep -q "lifecycle_manager_slam\].*Managed nodes are active" $D/robot.log; do sleep 3; done
sleep 5
echo "$(date +%T) parcours"
python3 $SOL/sim_outils/test_localisation.py --rapport $D/localisation.json --tf ${TF:-/tf} > $D/test.log 2>&1
echo "$(date +%T) fin"
python3 -c "
import json
for r in json.load(open('$D/localisation.json')):
    print(r.get('etape'), ': erreur', r.get('erreur_position_m'), 'm', r.get('erreur_cap_deg'), 'deg', r.get('erreur', ''))"
nettoie
