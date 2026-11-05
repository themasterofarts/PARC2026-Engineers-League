#!/bin/bash
# Course complete sur le simulateur officiel : simulation, arbitre, evaluateur, notre robot.launch.
#   course.sh SEED NOM [args robot.launch...]   (resultats dans ~/ma64_courses_simu/NOM)
#   seed:=-1 n'est pas gere ici : donner un SEED fixe (le masque lit parc_random_seed<SEED>.sdf)
#   SEED=fixe : arene fixe nba_sim.sdf de phase2.launch.py (verite dans arene_fixe.json)
#   ORDRE="1 4 9 7" : ordre impose a l'arbitre (sinon tire au hasard), pour comparer deux versions
# Avant : ~/parc2026_ws compile (voir le compte rendu), copie locale du simulateur etalonnee.
ICI="$(dirname "$(readlink -f "$0")")"
R=$HOME/ma64_courses_simu   # journaux et rapports
SEED=$1; NOM=$2; shift 2
D=$R/$NOM; mkdir -p $D
echo $$ > $R/course.pid
source $ICI/env.sh
SOL=/home/peace-fiacre/Projects/parc_ingé_league/src/parc_solution
nettoie() {
  for p in $(pgrep -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (launch|run)"); do kill -INT $p 2>/dev/null; done
  sleep 8
  for p in $(pgrep -f "^gz sim"); do kill $p; done
  for p in $(ps -eo pid,args | grep -E "^ *[0-9]+ (/opt/ros/jazzy/lib/|python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /home/peace-fiacre/parc2026_ws/install)" | grep -v grep | awk '{print $1}'); do kill $p 2>/dev/null; done
  pkill -f "evaluateur_sim.py" 2>/dev/null
  sleep 3
  for p in $(ps -eo pid,args | grep -E "^ *[0-9]+ (/opt/ros/jazzy/lib/|gz sim|/bin/sh -c ruby|python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (launch|run))" | grep -v grep | awk '{print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 1
}
nettoie
echo "$(date +%T) simulation seed $SEED"
VUE=false; [ "${GUI:-0}" = 1 ] && VUE=true   # GUI=1 : fenetres Gazebo et RViz
if [ "$SEED" = fixe ]; then
  # arene fixe nba_sim.sdf (phase2.launch.py, toujours avec fenetres) : tags au centre des murs, 3 tables
  VERITE="--verite $ICI/arene_fixe.json"
  MONDE="--sdf $(ros2 pkg prefix parc_robot_bringup)/share/parc_robot_bringup/worlds/nba_sim.sdf $VERITE"
  ros2 launch parc_robot_bringup phase2.launch.py audio:=false > $D/sim.log 2>&1 &
else
  VERITE="--seed $SEED"; MONDE="--seed $SEED"
  LANCE="parc_robot_bringup phase2_hard.launch.py"
  # SIMPLE=1 : arene peu encombree, comme celle de la competition (phase2_simple.launch.py)
  [ "${SIMPLE:-0}" = 1 ] && LANCE="$ICI/phase2_simple.launch.py"
  ros2 launch $LANCE seed:=$SEED gui:=$VUE rviz:=$VUE audio:=false > $D/sim.log 2>&1 &
fi
until timeout 5 ros2 topic echo --once /clock --field clock.sec > /dev/null 2>&1; do sleep 3; done
# vraie pose du robot pour l'evaluateur (/sim/pose ne publie rien)
ros2 run ros_gz_bridge parameter_bridge "/model/sitoe_robot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose" --ros-args -r /model/sitoe_robot/pose:=/verite/robot_pose -r __node:=pont_verite > $D/pont_verite.log 2>&1 &
echo "$(date +%T) arbitre"
ros2 launch parc_referee referee.launch.py odom_topic:=/botwheel_explorer/odom time_limit_s:=3600.0 record:=false > $D/arbitre.log 2>&1 &
python3 $SOL/sim_outils/evaluateur_sim.py $VERITE --rapport $D/evaluation.json > $D/evaluation.log 2>&1 &
EVAL=$!
# masque keepout des tables et chaises (le LiDAR ne voit que leurs pieds) ; MASQUE=0 pour s'en passer
if [ "${MASQUE:-1}" = 1 ]; then
  python3 $SOL/sim_outils/masque_simu_officielle.py $MONDE --sortie $D/masque > $D/masque.log 2>&1 \
    && set -- "$@" masque:=$D/masque.yaml
fi
echo "$(date +%T) robot.launch sim:=true $*"
ros2 launch nav_solution robot.launch.py sim:=true camera:=camera1 tag_size:=0.12 "$@" > $D/robot.log 2>&1 &
# RVIZ=1 : notre vue (carte, costmaps, masque, empreinte, camera), sans la fenetre Gazebo
[ "${RVIZ:-0}" = 1 ] && rviz2 -d $ICI/vue_ma64.rviz --ros-args -p use_sim_time:=true > $D/rviz.log 2>&1 &
until grep -q "attente de l'ordre des tags" $D/robot.log; do sleep 3; done
until grep -q "lifecycle_manager_navigation\].*Managed nodes are active" $D/robot.log; do sleep 3; done
sleep 10
echo "$(date +%T) armement"
until curl -s -X POST -H "X-Judge-Token: parc2026" -H "Content-Type: application/json" \
      -d "{\"team\": \"MA64\"${ORDRE:+, \"order\": \"$ORDRE\"}}" http://127.0.0.1:8080/api/judge/arm | tee $D/armement.json | grep -q '"ok"'; do sleep 3; done
until grep -q "attente du depart" $D/robot.log; do sleep 2; done
sleep 3
echo "$(date +%T) depart"
until grep -q "DEPART publie" $D/robot.log; do   # un --once peut partir avant la decouverte DDS
  timeout 20 ros2 topic pub --once -w 1 /ma64/start std_msgs/msg/String "{data: go}" > /dev/null 2>&1; sleep 5
done
wait $EVAL
echo "$(date +%T) fin"
cat $D/evaluation.log | tail -8
nettoie
