#!/bin/bash
# Fait MA64.zip au format d'Evans (src/ seulement) : nav_solution, parc_msgs, ma64 (course.sh, depart.sh)
#   ./faire_zip.sh            -> outils_robot/MA64.zip
S="$(dirname "$(readlink -f "$0")")/.."; T=$(mktemp -d)
mkdir -p $T/src && cp -r $S/nav_solution $S/ma64 $T/src/ && cp -r $S/sim_outils/parc_msgs $T/src/
find $T/src -name __pycache__ -prune -exec rm -rf {} +
(cd $T && zip -qr MA64.zip src) && mv $T/MA64.zip "$S/outils_robot/MA64.zip" && rm -rf $T
ls -la "$S/outils_robot/MA64.zip" | awk '{print "MA64.zip :", $5, "octets"}'
