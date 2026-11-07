# Environnement du simulateur officiel PARC 2026 (~/parc2026_ws), isole : domaine 64, Gazebo local
[ -n "$XDG_DATA_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH XDG_DATA_HOME \
      GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR SNAP SNAP_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
# domaine dedie : invisible du robot (domaine 0) et des autres equipes, decouverte fiable
# decouverte locale uniquement : un changement de Wi-Fi (nouvelle adresse IP) cassait la decouverte
# en pleine course (TF vide, depart jamais recu) ; la simulation ne depend plus du reseau
export ROS_DOMAIN_ID=64 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_IP=127.0.0.1 GZ_PARTITION=ma64
# CycloneDDS (comme le vrai robot) ; en decouverte locale, chaque processus prend un index de
# participant, limite par defaut trop basse pour ~40 processus (masque + RViz, 7 nov : mission
# non creee, "Failed to find a free participant index") -> limite relevee a 120
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# interface reseau = boucle locale uniquement : un changement de Wi-Fi en pleine course (7 nov, 02h41 :
# 10.228.x -> 192.168.1.x) coupait DDS et figeait la mission
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
source /opt/ros/jazzy/setup.bash
[ -f ~/parc2026_ws/install/setup.bash ] && source ~/parc2026_ws/install/setup.bash
cd ~/parc2026_ws
