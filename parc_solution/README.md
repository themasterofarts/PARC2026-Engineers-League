# EQUIPE : MA64 ROBOTICS (Bénin)

**Membres :** KPOKPO Sunday (chef d'équipe), EGOUDJOBI Peace Fiacre, SIRADJO SY, GBAGUIDI Freednel, THOUKAM Yves, KOCOU.

## La solution (phase réelle, robot CAYTU Sito-É)
Le robot construit sa carte en direct (`slam_toolbox`), détecte les AprilTags 36h11 avec la caméra du haut (`apriltag_ros`), et visite les tags 1, 4, 7, 9 dans l'ordre publié par l'arbitre :
- navigation avec Nav2 ;
- approche finale guidée par la caméra : arrêt à environ 0,5 m de l'avant du robot, face au tag à moins de ±12° ;
- envoi de `TagVisit` avec l'image JPEG, puis immobilité pendant 3 s.

Départ par commande, ou **à la voix (« allez »)**, avec Vosk hors ligne.

Code : paquet `nav_solution`. Lancement robot : `launch/robot.launch.py`. Mission : `nav_solution/mission_node.py`.

## Corrections indispensables pour le vrai robot (mesurées sur un rosbag du robot, 05/11)
| Problème | Correction | Effet mesuré |
|---|---|---|
| `camera1` regarde le sol (0,53 m, inclinée de 67°) ; la caméra du haut est **`camera2`** (0,92 m) | `camera2` par défaut (`robot.launch.py`, `robot.conf`, `lancer.sh`) | 0 tag détecté → 3 tags jusqu'à 5,7 m |
| La TF `base_link → laser` du robot ne correspond pas au LiDAR, dont les angles tournent à l'envers | nœud `scan_retourne` : `/scan` → `/scan_corrige` (mesures dans l'ordre inverse), lu par le filtre, le SLAM, Nav2 et la mission | carte cassée (11,9 × 7,9 m) → arène nette (6,96 × 6,03 m) |
| L'odométrie sous-estime la rotation d'environ 6,5 % | pas de correction : le SLAM l'absorbe avec le LiDAR corrigé | — |

## Départ vocal
- `demarrage_vocal.py` : Vosk, modèle français, **la phrase entière doit être « allez »**, dite seule.
  - Avec l'ancienne grammaire go/allez/vai : 10 faux départs en 60 s de parole. Avec cette règle : 0.
  - Sur 3 min du vrai micro du robot : 0 faux départ.
- La mission **ignore un mot entendu avant l'armement** de la course.

## Dépendances
Sur le robot (Jazzy) : Nav2, `slam_toolbox`, `laser_filters`, `apriltag_ros`, `nav2_map_server`, `teleop_twist_keyboard`, OpenCV. `deploy.sh` vérifie leur présence et affiche ce qui manque.

**Vosk** s'installe **hors ligne**, uniquement dans le dossier de l'équipe :
```bash
ROBOT=sitoe@192.168.1.126 ./deploy.sh --vocal
```
Cette commande copie depuis le PC :
- les paquets aarch64 de `~/parc_offline/wheels_jetson` ;
- le modèle `~/parc_offline/vosk-model-small-fr-0.22`.

Ils sont installés dans `~/MA64/pylib` : sans `sudo`, sans internet, rien dans le système.

## Utilisation sur le robot
Depuis le PC, dans `parc_solution/outils_robot` :
```bash
ROBOT=sitoe@192.168.1.126 ./deploy.sh --vocal     # copie, compile, vérifie
```
Puis, sur le robot (`ssh sitoe@192.168.1.126`) :

| Étape | Commande |
|---|---|
| Tester le micro | `~/MA64/lancer.sh vocal` (« allez » seul → `DEPART VOCAL`) |
| Cartographier (robot sur la marque de départ) | `~/MA64/lancer.sh carto` + `~/MA64/lancer.sh teleop` |
| Sauvegarder la carte et la position des tags | `~/MA64/lancer.sh sauver` (doit annoncer `4 tag(s) sauves`) |
| **Course** | `~/MA64/lancer.sh course` (voix activée automatiquement si Vosk est installé) |
| Suivre l'arbitre | `~/MA64/lancer.sh etat` et `~/MA64/lancer.sh ordre` |
| **Départ par commande** (secours), une fois l'état `ARMED` | `ros2 topic pub --once -w 1 /ma64/start std_msgs/msg/String "{data: go}"` |
| **Départ à la voix**, une fois l'état `ARMED` | dire **« allez »**, seul |

**Si Nav2 n'affiche pas `Managed nodes are active` après environ 1 min** (raté de démarrage DDS, vu en simulation), Ctrl+C et relancer, **avant** le départ.

Détails des créneaux : `Ressources/GUIDE_ROBOT_JEUDI.md`. Marche à suivre du créneau du 06/11 : `~/parc_offline/creneau_12h.md`.

## Simulation (simulateur officiel PARC)
`sim_outils/simu_officielle/course.sh SEED NOM` lance une course complète, avec l'arbitre et l'évaluateur.
- Dans le simulateur officiel non modifié, l'odométrie surestime les rotations de 31 % (`wheel_separation` 0,3409). Sur la seed 7, cela donne 1/4 tags et un robot renversé.
- En donnant au SLAM le cap du gyroscope (outil de test `~/ma64_verif/odom_imu_pont.py`), on obtient 4/4 en 281 s simulées, sans contact.

## À confirmer avec les organisateurs
- La distance de visite : 0,5 m depuis le **centre** du robot (texte du brief) ou depuis son **avant** (consigne orale du 05/11, suivie par la mission) ?
- L'heure et la forme du gel du code.
