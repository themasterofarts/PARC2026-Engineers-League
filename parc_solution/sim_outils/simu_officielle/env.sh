# Environnement du simulateur officiel PARC 2026 (~/parc2026_ws), isole : domaine 64, Gazebo local
[ -n "$XDG_DATA_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH XDG_DATA_HOME \
      GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR SNAP SNAP_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
# domaine dedie : invisible du robot (domaine 0) et des autres equipes, decouverte fiable
# decouverte locale uniquement : un changement de Wi-Fi (nouvelle adresse IP) cassait la decouverte
# en pleine course (TF vide, depart jamais recu) ; la simulation ne depend plus du reseau
export ROS_DOMAIN_ID=64 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_IP=127.0.0.1 GZ_PARTITION=ma64
source /opt/ros/jazzy/setup.bash
[ -f ~/parc2026_ws/install/setup.bash ] && source ~/parc2026_ws/install/setup.bash
cd ~/parc2026_ws
