#!/bin/bash
# Sur le robot : lance une etape du creneau, avec l'environnement de l'equipe.
#   ~/MA64/lancer.sh diag | mouvement | carto CAMERA TAILLE | etalon TAG | vocal | teleop | sauver
#                    course CAMERA TAILLE | etat | ordre | depart
# Appele depuis le PC par les volets Terminator (ouvrir_terminaux.sh), dans une session tmux.
source ~/MA64/env.sh
case "$1" in
  diag)      cd ~/MA64/outils && python3 diagnostic_robot.py ;;
  mouvement) cd ~/MA64/outils && python3 diagnostic_robot.py --mouvement --no-bag ;;
  carto)     ros2 launch nav_solution robot.launch.py mission:=false nav2:=false camera:="${2:-camera2}" tag_size:="${3:-0.165}" ;;
  etalon)    python3 ~/MA64/outils/distance_tag.py "${2:-4}" ;;
  rmw)       # essaie Cyclone puis FastDDS : garde le premier qui recoit vraiment /scan du robot
             rm -f ~/MA64/rmw.txt
             for r in rmw_cyclonedds_cpp rmw_fastrtps_cpp; do
               ros2 daemon stop > /dev/null 2>&1
               if RMW_IMPLEMENTATION=$r timeout 15 ros2 topic echo --once /scan --field header.frame_id > /dev/null 2>&1; then
                 echo $r > ~/MA64/rmw.txt; echo "OK : $r recoit /scan (enregistre dans ~/MA64/rmw.txt)"; break
               else echo "echec : $r ne recoit pas /scan"; fi
             done
             [ -f ~/MA64/rmw.txt ] || echo "AUCUN DDS ne recoit /scan : verifier le domaine (echo \$ROS_DOMAIN_ID) et le Wi-Fi" ;;
  camera)    # diagnostic camera -> detecteur de tags, conclusion en une ligne
             C=/${2:-camera2}/${2:-camera2}/color
             frequence() { PYTHONUNBUFFERED=1 timeout 7 ros2 topic hz "$1" 2>/dev/null | awk '/average rate/{r=$3} END{print (r==""?0:r)}'; }
             fiabilite() { ros2 topic info -v "$1" 2>/dev/null | awk '/Node name:/{n=$3} /Reliability/{print n":"$2; exit}'; }
             echo "topics camera presents :"; ros2 topic list | grep -i camera | sed 's/^/   /' | head -20
             if ! ros2 topic list | grep -qx "$C/image_raw"; then
               echo "PROBLEME : pas de $C/image_raw. Camera arretee ou topics renommes (voir la liste ci-dessus)."
             else
               fi_img=$(fiabilite $C/image_raw); fi_ci=$(fiabilite $C/camera_info)
               hz_ci=$(frequence $C/camera_info)
               echo "image_raw publie par ${fi_img:-?} ; camera_info par ${fi_ci:-aucun} ; camera_info a $hz_ci Hz"
               if [ -z "$fi_ci" ] || awk -v h="$hz_ci" 'BEGIN{exit !(h<1)}'; then
                 echo "PROBLEME : camera_info absent ou muet. Le detecteur ne traite aucune image sans lui."
               elif ! ros2 topic info /detections 2>/dev/null | grep -q "Publisher count: [1-9]"; then
                 echo "OK camera ($hz_ci Hz). Detecteur non lance : relancer cette etape pendant carto ou course."
               else
                 hz_d=$(frequence /detections)
                 if awk -v h="$hz_d" 'BEGIN{exit !(h<1)}'; then
                   echo "PROBLEME : camera OK ($hz_ci Hz) mais le detecteur ne publie rien ($hz_d Hz). Envoyer cette sortie."
                 else
                   echo "OK : camera2 publie images et camera_info ($hz_ci Hz), le detecteur recoit ($hz_d Hz)"
                 fi
               fi
             fi ;;
  vocal)     ros2 run nav_solution demarrage_vocal --ros-args -p model_path:="$HOME/MA64/vosk-model" ;;
  teleop)    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/botwheel_explorer/cmd_vel \
               -p stamped:=true -p frame_id:=base_link -p speed:=0.15 -p turn:=0.5 ;;
  sauver)    mkdir -p ~/MA64/cartes && ros2 run nav2_map_server map_saver_cli -f ~/MA64/cartes/arene \
               && ros2 run nav_solution sauver_tags ~/MA64/cartes/tags.yaml ;;
  course)    masque=""   # masque des plateaux et assises, s'il a ete dessine (volet Masque du PC)
             [ -f ~/MA64/cartes/arene_keepout.yaml ] && masque="masque:=$HOME/MA64/cartes/arene_keepout.yaml"
             echo "masque : ${masque:-aucun}"
             vocal=""    # depart vocal (Vosk) si le module et le modele sont installes ; sinon depart par commande
             [ -d ~/MA64/vosk-model ] && python3 -c "import vosk" 2>/dev/null && vocal="vocal:=true"
             echo "depart vocal : ${vocal:-non (depart par commande seulement)}"
             ros2 launch nav_solution robot.launch.py camera:="${2:-camera2}" tag_size:="${3:-0.165}" \
               indices:="$HOME/MA64/cartes/tags.yaml" $masque $vocal \
               ouverture_avance:="${AVANCE:-0.0}" ouverture_virage:="${VIRAGE:-0.0}" \
               arbitre:="$([ "${SANS_ARBITRE:-0}" = 1 ] && echo false || echo true)" ;;
  ordre_manuel)  # sans arbitre : donne l'ordre a la mission (ex. ~/MA64/lancer.sh ordre_manuel 4,9,1,7)
             ros2 topic pub --once -w 1 --qos-durability transient_local /ma64/tag_order parc_msgs/msg/TagOrder "{tag_ids: [${2:-4,9,1,7}]}" ;;
  etat)      ros2 topic echo /parc/run_status ;;
  ordre)     ros2 topic echo /parc/tag_order --qos-durability transient_local ;;
  depart)    ros2 topic pub --once /ma64/start std_msgs/msg/String "{data: go}" ;;
  *)         echo "etape inconnue : $1" ;;
esac
exec bash
