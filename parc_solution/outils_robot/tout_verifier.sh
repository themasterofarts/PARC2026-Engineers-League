#!/bin/bash
# Tout en une commande, SANS faire bouger le robot :
#   1. deploy (code + parc_msgs + Cyclone), 2. verif_robot.sh (capteurs, sens du LiDAR, Vosk),
#   3. verif_nav.sh (navigation lancee sans mission, chemin calcule, puis tout arrete).
#   ./tout_verifier.sh            (un seul mot de passe SSH)
#   SANS_DEPLOY=1 ./tout_verifier.sh   (code deja deploye)
ICI="$(dirname "$(readlink -f "$0")")"
cd "$ICI" && source robot.conf || exit 1
echo "################ 1/3 DEPLOIEMENT"
if [ "${SANS_DEPLOY:-0}" != 1 ]; then ./deploy.sh --parc-msgs || { echo "DEPLOIEMENT RATE : on s'arrete"; exit 1; }; else echo "(saute)"; fi
echo; echo "################ 2/3 CAPTEURS, LIDAR, VOSK"
./verif_robot.sh
echo; echo "################ 3/3 NAVIGATION (sans mouvement)"
./verif_nav.sh
echo; echo "################ FINI : envoie toute cette sortie si une ligne dit ECHEC ou PROBLEME"
