#!/bin/bash
# Verification complete du robot a distance, SANS le faire bouger (lecture seule).
#   ./verif_robot.sh        (depuis le PC, apres deploy ; mot de passe SSH si besoin)
# Teste : couche DDS, frequences LiDAR / odometrie / camera2, autre robot sur le domaine, TF du LiDAR,
# sens du LiDAR (le mur doit etre a droite au depart), arbitre, audio, Vosk et modele, tags vus.
ICI="$(dirname "$(readlink -f "$0")")"
source "$ICI/robot.conf"
ssh -o ControlMaster=auto -o ControlPath=~/.ssh/cm-%r@%h:%p -o ControlPersist=30m "$ROBOT" bash -s <<'DISTANT'
source ~/teams/MA64/env.sh
echo "== DDS : ${RMW_IMPLEMENTATION:-defaut} | domaine : ${ROS_DOMAIN_ID:-0}"
echo "== Frequences (attendu : scan et odom ~10 Hz, camera2 ~15 Hz)"
for t in /scan /botwheel_explorer/odom /camera2/camera2/color/image_raw /camera2/camera2/color/camera_info; do
  hz=$(timeout 6 ros2 topic hz $t 2>/dev/null | awk '/average rate/{r=$3} END{print (r==""?0:r)}')
  echo "   $t : $hz Hz"
done
echo "== Sources de /scan (attendu : une seule, laser)"
timeout 6 ros2 topic echo /scan --field header.frame_id 2>/dev/null | grep -v -- --- | sort | uniq -c | sed 's/^/   /'
echo "== TF base_link -> laser (attendu : RPY ... 180)"
timeout 6 ros2 run tf2_ros tf2_echo base_link laser 2>/dev/null | grep -m2 -E "Translation|RPY .degree" | sed 's/^/   /'
echo "== Sens du LiDAR (le mur est a droite au depart : 'droite' doit etre la plus petite distance)"
python3 - <<'PY'
import math, rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node('verif_lidar'); r = {}
n.create_subscription(LaserScan, '/scan', lambda m: r.setdefault('s', m), qos_profile_sensor_data)
t = 0
while 's' not in r and t < 10:
    rclpy.spin_once(n, timeout_sec=1); t += 1
if 's' not in r:
    print('   PROBLEME : aucun /scan recu'); raise SystemExit
s = r['s']
def mini(base, lo, hi):
    v = []
    for i, d in enumerate(s.ranges):
        a = s.angle_min + i * s.angle_increment
        b = math.degrees(math.atan2(math.sin(base(a)), math.cos(base(a))))
        x, y = d * math.cos(math.radians(b)), d * math.sin(math.radians(b))
        corps = -0.20 < x < 0.22 and abs(y) < 0.27     # le robot lui-meme
        if math.isfinite(d) and d > 0.1 and not corps and lo <= b <= hi:
            v.append(d)
    return round(min(v), 2) if v else None
for nom, base in (('avec correction (notre code)', lambda a: math.pi - a), ('sans correction (TF seule) ', lambda a: a + math.pi)):
    g, dr = mini(base, 60, 120), mini(base, -120, -60)
    verdict = 'mur a DROITE' if (dr or 9) < (g or 9) else 'mur a GAUCHE'
    print(f'   {nom} : gauche {g} m | droite {dr} m | devant {mini(base, -20, 20)} m -> {verdict}')
print('   attendu : "avec correction" -> mur a DROITE')
# profil complet autour du robot (avec notre correction), le corps du robot exclu
pts = []
for i, d in enumerate(s.ranges):
    if not math.isfinite(d) or d < 0.1: continue
    a = math.pi - (s.angle_min + i * s.angle_increment)
    x, y = d * math.cos(a), d * math.sin(a)
    if -0.20 < x < 0.22 and abs(y) < 0.27: continue
    pts.append((math.degrees(math.atan2(y, x)), d, x, y))
print('   profil (0 devant, +90 gauche, -90 droite, 180 derriere) :')
lig = []
for c in range(-150, 210, 30):
    v = [p for p in pts if (c - 15 <= p[0] < c + 15) or (c == 180 and p[0] < -165)]
    lig.append(f'{c if c <= 180 else c - 360:+4d}: ' + (f'{min(p[1] for p in v):.2f}' if v else '-'))
print('     ' + '  '.join(lig))
der = [-p[2] - 0.20 for p in pts if p[2] < -0.20 and abs(p[3]) < 0.5]
dro = [-p[3] - 0.27 for p in pts if p[3] < -0.27 and abs(p[2]) < 0.5]
gau = [p[3] - 0.27 for p in pts if p[3] > 0.27 and abs(p[2]) < 0.5]
f = lambda v: f'{min(v):.2f} m' if v else 'rien a moins de 0.5 m de large'
print(f'   ecart arriere du robot -> obstacle derriere : {f(der)}')
print(f'   ecart flanc droit      -> obstacle a droite : {f(dro)}')
print(f'   ecart flanc gauche     -> obstacle a gauche : {f(gau)}')
print('   attendu au depart : murs derriere et a droite, rien de proche a gauche')
PY
echo "== Arbitre (attendu : /parc/run_status, /parc/tag_order, ...)"
ros2 topic list | grep /parc/ | sed 's/^/   /'
echo "== Audio"
ros2 topic list | grep -iE "audio" | sed 's/^/   /' || echo "   aucun topic audio"
echo "== Vosk"
python3 -c "import vosk; print('   vosk importable :', vosk.__file__)" 2>/dev/null || echo "   vosk NON importable (depart par commande)"
# modele francais du constructeur (CAYTU) copie dans notre dossier s'il manque (jamais ceux des autres equipes)
CAYTU=/home/sitoe/CAYTU_SitoE/sitoe_v7_complet/vosk-model-small-fr-0.22
if [ ! -f ~/teams/MA64/vosk-model/conf/model.conf ] && [ -f $CAYTU/conf/model.conf ]; then
  rm -rf ~/teams/MA64/vosk-model && cp -r $CAYTU ~/teams/MA64/vosk-model && echo "   modele CAYTU copie dans ~/teams/MA64/vosk-model"
fi
ls -d ~/teams/MA64/vosk-model >/dev/null 2>&1 && echo "   modele equipe : present" || echo "   modele equipe : absent (~/teams/MA64/vosk-model)"
echo "   autres modeles sur le robot :"; find / -maxdepth 6 -type f -path "*conf/model.conf" 2>/dev/null | sed 's|/conf/model.conf||; s/^/     /' | head -5
echo "== Tags vus (seulement si carto ou course tourne)"
if ros2 topic list | grep -qx /detections; then
  timeout 8 ros2 topic echo /detections --field detections 2>/dev/null | grep "id:" | sort | uniq -c | sed 's/^/   /'
  echo "   (rien ci-dessus = aucun tag dans le champ de la camera)"
else echo "   detecteur non lance"; fi
echo "== Fin"
DISTANT
# copie du modele sur le PC (~/parc_offline) : deploy.sh --vocal pourra le renvoyer si le robot est efface
if [ ! -f ~/parc_offline/vosk-model-small-fr-0.22/conf/model.conf ]; then
  mkdir -p ~/parc_offline && rm -f ~/parc_offline/vosk-model-small-fr-0.22.zip
  scp -r -q -o ControlPath=~/.ssh/cm-%r@%h:%p "$ROBOT:teams/MA64/vosk-model" ~/parc_offline/vosk-model-small-fr-0.22 \
    && echo "== Modele Vosk copie sur le PC : ~/parc_offline/vosk-model-small-fr-0.22" \
    || echo "== Copie du modele sur le PC ratee"
fi
