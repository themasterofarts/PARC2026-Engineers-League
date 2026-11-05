#!/bin/bash
# Arrete tout ce qui tourne pour le simulateur officiel (a appeler en fichier : le motif ne
# figure pas dans la ligne de commande de l'appelant)
[ -f $HOME/ma64_courses_simu/course.pid ] && kill $(cat $HOME/ma64_courses_simu/course.pid) 2>/dev/null; rm -f $HOME/ma64_courses_simu/course.pid
for p in $(pgrep -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (launch|run)"); do kill -INT $p 2>/dev/null; done
for p in $(pgrep -f "evaluateur_sim\.py"); do kill $p 2>/dev/null; done
sleep 8
for p in $(pgrep -f "^gz sim"); do kill $p 2>/dev/null; done
for p in $(ps -eo pid,args | grep -E "^ *[0-9]+ (/opt/ros/jazzy/lib/|python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /home/peace-fiacre/Projects/parc_ingé_league/install)" | grep -v grep | awk '{print $1}'); do kill $p 2>/dev/null; done
sleep 3
# second passage : on force ce qui resiste
for p in $(ps -eo pid,args | grep -E "^ *[0-9]+ (/opt/ros/jazzy/lib/|gz sim|/bin/sh -c ruby|python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /home/peace-fiacre/parc2026_ws/install|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (launch|run))" | grep -v grep | awk '{print $1}'); do kill -9 $p 2>/dev/null; done
sleep 1
echo "restants : $(ps -eo args | grep -cE '^/opt/ros/jazzy/lib/|^gz sim|parc2026_ws/install|ros2 launch')"
