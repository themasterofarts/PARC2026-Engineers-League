#!/bin/bash
# Un volet Terminator : affiche le role du volet et tape la commande, sans la lancer.
# Entree la lance ; la ligne reste modifiable avant. Utilise par ouvrir_terminaux.sh.
#   volet.sh ETAPE
ICI="$(dirname "$(readlink -f "$0")")"
cd "$ICI"
C='source robot.conf &&'
# meme connexion SSH partagee que deploy.sh : un seul mot de passe pour tout le creneau
M='-o ControlMaster=auto -o ControlPath=~/.ssh/cm-%r@%h:%p -o ControlPersist=30m'
distant() {  # distant SESSION ETAPE [ARGS] : etape lancee sur le robot dans une session tmux
  echo "$C ssh $M -t \$ROBOT \"tmux new -A -s $1 '~/teams/MA64/lancer.sh ${*:2}'\""
}
case "$1" in
  nettoyer)  titre="0. Une seule fois : supprimer l'ancien dossier ~/MA64 (hors ~/teams)"
             cmd="$C ssh $M \$ROBOT 'rm -rf ~/MA64 && echo ancien dossier supprime'" ;;
  deploy)    titre="1. Deployer le code + parc_msgs (PC) - mot de passe SSH ici"
             vocal=""; [ -f ~/parc_offline/vosk-model-small-fr-0.22/conf/model.conf ] && vocal=" --vocal"   # Vosk si present sur le PC
             cmd="$C ./deploy.sh --parc-msgs$vocal" ;;
  tout)      titre="1. TOUT : deploy + verif capteurs + verif navigation (sans mouvement)"; cmd="$C ./tout_verifier.sh" ;;
  verifnav)  titre="1e. Verification navigation sans mouvement (lance puis arrete tout)"; cmd="$C ./verif_nav.sh" ;;
  verif)     titre="1d. Verification complete a distance (robot immobile)"; cmd="$C ./verif_robot.sh" ;;
  rmw)       titre="1c. Couche DDS : garde celle qui recoit /scan (Cyclone puis FastDDS)"; cmd=$(distant rmw rmw) ;;
  camera)    titre="5b. Camera -> detecteur de tags (pendant carto ou course)"; cmd=$(distant camera camera '$CAMERA') ;;
  vocal)     titre="5c. Test du micro : dire allez (Ctrl+C ensuite)"; cmd=$(distant vocal vocal) ;;
  bag)       titre="1b. Rosbag : a lancer AVANT le diagnostic, Ctrl+C a la fin"; cmd=$(distant bag bag) ;;
  diag)      titre="2. Diagnostic au repos (robot immobile)";           cmd=$(distant diag diag) ;;
  mouvement) titre="3. Diagnostic mouvement : taper OUI (robot)"; cmd=$(distant mouvement mouvement) ;;
  rapatrier) titre="13. Fin : rapatrier les resultats (PC)"
             cmd="$C scp $M -r \$ROBOT:teams/MA64/outils/diag_MA64_* \$ROBOT:teams/MA64/cartes \$ROBOT:teams/MA64/journaux \$ROBOT:teams/MA64/bags ../Ressources/" ;;
  carto)     titre="5. Cartographie / etalonnage : SLAM + AprilTag (robot)"
             cmd=$(distant carto carto '$CAMERA $TAILLE') ;;
  etalon)    titre="6. Etalonnage : distance camera -> tag, a comparer au ruban"
             cmd=$(distant etalon etalon '$TAG_ETALON') ;;
  teleop)    titre="7. Teleop : i , j l k (SANS anti-collision)"; cmd=$(distant teleop teleop) ;;
  sauver)    titre="8. Sauver carte + tags : doit afficher [1, 4, 7, 9]"; cmd=$(distant sauver sauver) ;;
  masque)    titre="9. Masque (PC) : dessiner plateaux et assises, puis s"
             cmd="$C mkdir -p ../Ressources/carte_robot && scp $M \$ROBOT:teams/MA64/cartes/arene.* ../Ressources/carte_robot/ && source /opt/ros/jazzy/setup.bash && source ../../../install/setup.bash && ros2 run nav_solution generer_masque ../Ressources/carte_robot/arene.yaml && scp $M ../Ressources/carte_robot/arene_keepout.* \$ROBOT:teams/MA64/cartes/" ;;
  course)    titre="10. Course : robot pose sur le scotch"; cmd=$(distant course course '$CAMERA $TAILLE') ;;
  etat)      titre="11. Etat de l'arbitre : attendre state: 1"; cmd=$(distant etat etat) ;;
  depart)    titre="12. DEPART : quand state: 1 et 'attente du depart'"; cmd=$(distant depart depart) ;;
  web)       titre="4. Page de l'arbitre (PC, navigateur)";                 cmd="$C xdg-open http://\${ROBOT#*@}:8080/" ;;
  *)         titre="?"; cmd="" ;;
esac
[ -n "$VOLET_TEST" ] && { echo "$cmd"; exit 0; }
printf '\033]0;%s\007' "$titre"
echo "=== $titre"
echo "    (Entree pour lancer ; robot.conf : ROBOT, CAMERA, TAILLE)"
read -e -i "$cmd" -p '$ ' ligne
eval "$ligne"
exec bash
