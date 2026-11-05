#!/bin/bash
# Compile la copie locale de parc_msgs dans ~/ma64_parc_msgs_ws.
# rosidl ne supporte pas le "e" accentue du chemin du projet (parc_ingé_league),
# d'ou cet espace de travail separe. Ensuite : source ~/ma64_parc_msgs_ws/install/setup.bash
# A n'utiliser sur le robot QUE si les organisateurs ne fournissent pas parc_msgs.
set -e
WS=${1:-$HOME/ma64_parc_msgs_ws}
mkdir -p "$WS/src"
rm -rf "$WS/src/parc_msgs"
cp -r "$(dirname "$(readlink -f "$0")")/parc_msgs" "$WS/src/"
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select parc_msgs
echo "OK : source $WS/install/setup.bash"
