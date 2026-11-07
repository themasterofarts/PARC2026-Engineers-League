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
PY
echo "== Arbitre (attendu : /parc/run_status, /parc/tag_order, ...)"
ros2 topic list | grep /parc/ | sed 's/^/   /'
echo "== Audio"
ros2 topic list | grep -iE "audio" | sed 's/^/   /' || echo "   aucun topic audio"
echo "== Vosk"
python3 -c "import vosk; print('   vosk importable :', vosk.__file__)" 2>/dev/null || echo "   vosk NON importable (depart par commande)"
ls -d ~/teams/MA64/vosk-model >/dev/null 2>&1 && echo "   modele equipe : present" || echo "   modele equipe : absent (~/teams/MA64/vosk-model)"
echo "   autres modeles sur le robot :"; find / -maxdepth 6 -type f -path "*conf/model.conf" 2>/dev/null | sed 's|/conf/model.conf||; s/^/     /' | head -5
echo "== Tags vus (seulement si carto ou course tourne)"
if ros2 topic list | grep -qx /detections; then
  timeout 8 ros2 topic echo /detections --field detections 2>/dev/null | grep "id:" | sort | uniq -c | sed 's/^/   /'
  echo "   (rien ci-dessus = aucun tag dans le champ de la camera)"
else echo "   detecteur non lance"; fi
echo "== Fin"
DISTANT
