#!/bin/bash
# Verifie que toute la navigation se lance et que le robot POURRAIT avancer, SANS le faire bouger.
#   ./verif_nav.sh        (depuis le PC, apres deploy)
# Lance robot.launch.py SANS le noeud de mission (rien ne peut commander les roues), attend le
# demarrage, controle SLAM, Nav2, carte, costmaps, chaine de commande vers les roues, place devant
# le robot, calcul d'un chemin (calcul seul, aucun mouvement) et detecteur de tags, puis arrete tout.
ICI="$(dirname "$(readlink -f "$0")")"
source "$ICI/robot.conf"
ssh -o ControlMaster=auto -o ControlPath=~/.ssh/cm-%r@%h:%p -o ControlPersist=30m "$ROBOT" bash -s <<'DISTANT'
source ~/teams/MA64/env.sh
ok=0; ko=0
res() { if [ "$1" = 0 ]; then echo "   OK   $2"; ok=$((ok+1)); else echo "   ECHEC $2"; ko=$((ko+1)); fi; }
tmux kill-session -t verifnav 2>/dev/null
for t in course nav carto; do tmux kill-session -t $t 2>/dev/null && echo "   (session $t arretee)"; done
echo "== LiDAR du robot (pilote des organisateurs)"
lidar=0
for i in $(seq 1 6); do timeout 5 ros2 topic hz /scan 2>/dev/null | grep -q "average rate" && { lidar=1; break; }; echo "   /scan muet, nouvel essai ($i/6)"; done
res $([ $lidar = 1 ] && echo 0 || echo 1) "/scan publie"
[ $lidar = 1 ] || echo "   -> LiDAR MUET : signaler aux organisateurs (pilote du robot), le SLAM ne peut pas demarrer"
echo "== Lancement de la navigation SANS mission (aucune commande de mouvement possible)"
tmux new -d -s verifnav "bash -c 'source ~/teams/MA64/env.sh && ros2 launch nav_solution robot.launch.py camera:=camera2 tag_size:=0.165 mission:=false > /tmp/verifnav.log 2>&1'"
for i in $(seq 1 60); do grep -q "lifecycle_manager_navigation\].*Managed nodes are active" /tmp/verifnav.log 2>/dev/null && break; sleep 2; done
grep -q "lifecycle_manager_navigation\].*Managed nodes are active" /tmp/verifnav.log; res $? "Nav2 demarre (Managed nodes are active)"
sleep 5
echo "== Noeuds"
for n in slam_toolbox planner_server controller_server bt_navigator collision_monitor; do
  timeout 6 ros2 lifecycle get /$n 2>/dev/null | grep -q active; res $? "$n actif"
done
for n in /scan_retourne /laser_filter /apriltag; do ros2 node list 2>/dev/null | grep -q "^$n$"; res $? "noeud $n present"; done
echo "== Capteurs dans notre chaine"
timeout 6 ros2 topic hz /scan_corrige 2>/dev/null | grep -q "average rate"; res $? "/scan_corrige publie (LiDAR remis dans le bon sens)"
timeout 6 ros2 topic hz /scan_ma64 2>/dev/null | grep -q "average rate"; res $? "/scan_ma64 publie (LiDAR filtre, lu par SLAM et Nav2)"
timeout 6 ros2 topic hz /detections 2>/dev/null | grep -q "average rate"; res $? "/detections publie (le detecteur recoit la camera)"
echo "== Carte et position"
timeout 10 ros2 topic echo --once /map --field info.width 2>/dev/null | grep -qE "^[0-9]+"; res $? "carte SLAM publiee"
timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q Translation; res $? "TF map -> base_link (robot localise)"
for t in /global_costmap/costmap /local_costmap/costmap; do
  timeout 8 ros2 topic echo --once $t --field info.width 2>/dev/null | grep -qE "^[0-9]+"; res $? "$t publiee"
done
echo "== Chaine de commande vers les roues"
ros2 topic info /botwheel_explorer/cmd_vel -v 2>/dev/null | grep -A1 "Node name: collision_monitor" | grep -q collision_monitor; res $? "collision_monitor publie sur /botwheel_explorer/cmd_vel"
ros2 topic info /botwheel_explorer/cmd_vel 2>/dev/null | grep -q "Subscription count: [1-9]"; res $? "les roues ecoutent /botwheel_explorer/cmd_vel"
echo "== Place pour avancer et chemin (calcul seul)"
python3 - <<'PY'
import math, rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node('verif_devant'); r = {}
n.create_subscription(LaserScan, '/scan_ma64', lambda m: r.setdefault('s', m), qos_profile_sensor_data)
t = 0
while 's' not in r and t < 10: rclpy.spin_once(n, timeout_sec=1); t += 1
if 's' in r:
    s = r['s']; v = [d for i, d in enumerate(s.ranges) if math.isfinite(d) and d > 0.1
         and abs(math.degrees(math.atan2(math.sin(s.angle_min + i*s.angle_increment + math.pi), math.cos(s.angle_min + i*s.angle_increment + math.pi)))) < 15]
    d = min(v) if v else 10.0
    print(f"   {'OK   ' if d > 0.6 else 'ECHEC'} place libre devant le robot : {d:.2f} m (il faut > 0.6 m)")
    open('/tmp/verifnav_but.txt', 'w').write(f"{max(0.3, min(1.0, d - 0.5)):.2f}")
else:
    print("   ECHEC pas de /scan_ma64")
PY
# but devant le robot, en deca du premier obstacle (calcul seul)
B=$(cat /tmp/verifnav_but.txt 2>/dev/null || echo 0.8)
timeout 25 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{goal: {header: {frame_id: map}, pose: {position: {x: $B, y: 0.0}, orientation: {w: 1.0}}}, use_start: false}" > /tmp/verifnav_chemin.txt 2>&1
grep -q SUCCEEDED /tmp/verifnav_chemin.txt; res $? "Nav2 calcule un chemin $B m devant le robot (sans le suivre)"
grep -q SUCCEEDED /tmp/verifnav_chemin.txt || grep -E "planner_server\].*(ERROR|WARN)" /tmp/verifnav.log | grep -v "Inflation layer" | tail -3 | cut -c1-170 | sed 's/^/     planificateur : /'
# place sur les cotes : le robot doit pouvoir s'ecarter du mur
python3 - <<'PY2'
import math, rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node('verif_cotes'); r = {}
n.create_subscription(LaserScan, '/scan_ma64', lambda m: r.setdefault('s', m), qos_profile_sensor_data)
t = 0
while 's' not in r and t < 10: rclpy.spin_once(n, timeout_sec=1); t += 1
if 's' in r:
    s = r['s']; lat = []
    for i, d in enumerate(s.ranges):
        a = s.angle_min + i * s.angle_increment + math.pi
        x, y = d * math.cos(a), d * math.sin(a)
        if math.isfinite(d) and -0.20 < x < 0.20 and abs(y) > 0.20: lat.append(abs(y) - 0.23)   # bord du robot a 0.23 m
    m = min(lat) if lat else 9
    print(f"   {'OK   ' if m > 0.03 else 'ATTENTION'} ecart flanc du robot / obstacle le plus proche sur le cote : {m:.2f} m"
          + ("" if m > 0.08 else " -> robot presque colle : Nav2 risque de ne pas demarrer, l'ecarter de 10 cm"))
PY2
echo "== Bilan : $ok OK, $ko ECHEC"
[ $ko -gt 0 ] && { echo "   erreurs du lancement :"; grep -E "ERROR|FATAL|died" /tmp/verifnav.log | grep -v Error_code | tail -5 | cut -c1-160 | sed 's/^/     /'; }
echo "== Arret de la navigation"
tmux send-keys -t verifnav C-c 2>/dev/null; sleep 6; tmux kill-session -t verifnav 2>/dev/null
echo "   arretee (le robot n'a recu aucune commande de mouvement)"
DISTANT
