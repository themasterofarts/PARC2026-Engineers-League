#!/bin/bash
# Sur le robot : lance une etape du creneau, avec l'environnement de l'equipe.
#   ~/teams/MA64/lancer.sh diag | mouvement | carto CAMERA TAILLE | etalon TAG | teleop | sauver
#                    course CAMERA TAILLE | etat | ordre | depart
# Appele depuis le PC par les volets Terminator (ouvrir_terminaux.sh), dans une session tmux.
source ~/teams/MA64/env.sh
case "$1" in
  diag)      cd ~/teams/MA64/outils && python3 diagnostic_robot.py ;;
  mouvement) cd ~/teams/MA64/outils && python3 diagnostic_robot.py --mouvement --no-bag ;;
  carto)     ros2 launch nav_solution robot.launch.py mission:=false nav2:=false camera:="${2:-camera2}" tag_size:="${3:-0.165}" ;;
  etalon)    python3 ~/teams/MA64/outils/distance_tag.py "${2:-4}" ;;
  teleop)    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/botwheel_explorer/cmd_vel \
               -p stamped:=true -p frame_id:=base_link -p speed:=0.15 -p turn:=0.5 ;;
  sauver)    mkdir -p ~/teams/MA64/cartes && ros2 run nav2_map_server map_saver_cli -f ~/teams/MA64/cartes/arene \
               && ros2 run nav_solution sauver_tags ~/teams/MA64/cartes/tags.yaml ;;
  course)    masque=""   # masque des plateaux et assises, s'il a ete dessine (volet Masque du PC)
             [ -f ~/teams/MA64/cartes/arene_keepout.yaml ] && masque="masque:=$HOME/teams/MA64/cartes/arene_keepout.yaml"
             echo "masque : ${masque:-aucun}"
             vocal=""    # depart vocal (Vosk) si le module et le modele sont installes ; sinon depart par commande
             [ -d ~/teams/MA64/vosk-model ] && python3 -c "import vosk" 2>/dev/null && vocal="vocal:=true"
             echo "depart vocal : ${vocal:-non (depart par commande seulement)}"
             ros2 launch nav_solution robot.launch.py camera:="${2:-camera2}" tag_size:="${3:-0.165}" \
               indices:="$HOME/teams/MA64/cartes/tags.yaml" $masque $vocal ;;
  etat)      ros2 topic echo /parc/run_status ;;
  ordre)     ros2 topic echo /parc/tag_order --qos-durability transient_local ;;
  depart)    ros2 topic pub --once /ma64/start std_msgs/msg/String "{data: go}" ;;
  *)         echo "etape inconnue : $1" ;;
esac
exec bash
