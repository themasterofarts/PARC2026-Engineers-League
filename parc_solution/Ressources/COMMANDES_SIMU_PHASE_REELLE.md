# Simulation phase réelle : commandes et travail réalisé

Branche : `peace/phase-reelle`, rien n'est encore commité. État au 3 novembre 2026.

---

## 1. Préparer le terminal

**Lance les commandes depuis Terminator, pas depuis le terminal de VS Code.** VS Code est installé en snap, et son environnement fait planter Gazebo et RViz (`symbol lookup error ... libpthread`).

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Projects/parc_ingé_league
source install/setup.bash
```

Mets le PC en mode performance avant de lancer la simulation. En économie d'énergie, le processeur est bridé à 800 MHz et Nav2 finit par planter.

```bash
powerprofilesctl set performance
```

### Dépendance AprilTag
`apriltag_ros` n'est pas encore installé sur le PC : il n'a tourné que depuis une copie temporaire. À installer une fois :

```bash
sudo apt install ros-jazzy-apriltag-ros
```

Sans ce paquet, ajoute `apriltags:=false` aux commandes de mapping de la section 3.

### Compiler après une modification

```bash
colcon build --packages-select parc_robot_bringup nav_solution
source install/setup.bash
```

---

## 2. Arène seule (Gazebo)

```bash
ros2 launch parc_robot_bringup arena.launch.py
```

| Option | Défaut | Rôle |
|---|---|---|
| `gui:=false` | `true` | serveur sans fenêtre, plus rapide |
| `x:=` `y:=` `yaw:=` | `0.2` `-1.4` `1.5708` | position de départ du robot |

Ce launch ne lance pas d'EKF. Sans le launch de mapping, personne ne publie la TF `odom → base_footprint` (voir la section 6).

---

## 3. Mapping en direct

### Mapping seul, en téléopération (recommandé pour cartographier)

```bash
ros2 launch nav_solution mapping.launch.py nav2:=false
```

Ce launch ouvre Gazebo et RViz (vue légère : carte, scan filtré, robot, TF, image de la caméra), et lance l'EKF, le filtre laser, slam_toolbox et la détection des tags.

### Mapping avec Nav2 (buts « Nav2 Goal » dans RViz)

```bash
ros2 launch nav_solution mapping.launch.py
```

RViz s'ouvre alors avec la vue Nav2 complète. Les costmaps de cette vue sont lourdes à afficher sur le PC.

### Options de `mapping.launch.py`

| Option | Défaut | Rôle |
|---|---|---|
| `nav2:=false` | `true` | ne lance pas Nav2 |
| `gui:=false` | `true` | pas de fenêtre Gazebo |
| `rviz:=false` | `true` | pas de RViz |
| `apriltags:=false` | `true` | pas de détection des tags |

Exemple sans aucune fenêtre, pour les tests :

```bash
ros2 launch nav_solution mapping.launch.py nav2:=false gui:=false rviz:=false apriltags:=false
```

---

## 4. Téléopération

Dans un autre terminal :

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/robot_base_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.6
```

- `i` avance, `,` recule, `j` et `l` tournent, `k` arrête.
- `q` et `z` augmentent ou baissent la vitesse.
- La commande va directement aux roues simulées, **sans anti-collision**.
- Si un but Nav2 est en cours, les deux commandes se mélangent. Annule le but avant de piloter.

---

## 5. Sauvegarder la carte

Pendant que le mapping tourne :

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/arene
```

Cette commande produit `arene.pgm` et `arene.yaml`.

---

## 6. Ce qui a été fait

### Monde de simulation de l'arène

| Fichier | Contenu |
|---|---|
| `parc_robot_bringup/worlds/arena_world.sdf` | arène de 5 × 5 m centrée en (0, 0), murets de 0,40 m, sol vert, 4 tables, tags 1, 4, 7 et 9 sur poteaux, centre du tag à 1 m |
| `parc_robot_bringup/models/school_desk/` | table d'école d'après la photo : plateau de 0,60 × 0,45 m à 0,72 m, un pied de chaque côté posé sur une barre au sol |
| `parc_robot_bringup/models/apriltag_{1,4,7,9}/` | tag 36h11, carré noir de 16,5 cm, sur panneau et poteau |
| `parc_robot_bringup/launch/arena.launch.py` | arène, robot et ponts ROS |
| `parc_robot_bringup/config/gz_bridge_arena.yaml` | copie de `gz_bridge.yaml` **sans le pont `tf`** (voir plus bas) |

La disposition des tables et des tags est aléatoire, tirée avec la graine 9959. Le tirage impose :
- au moins 0,6 m de passage entre les tables ;
- la zone de 0,5 à 1 m devant chaque tag libre ;
- un départ dégagé.

**Les dimensions sont estimées sur la photo, pas mesurées.**

### Mapping en direct

| Fichier | Contenu |
|---|---|
| `nav_solution/launch/mapping.launch.py` | arène, EKF, filtre laser, slam_toolbox, Nav2 en option, AprilTag, RViz |
| `nav_solution/config/ekf_arena.yaml` | copie de `ekf_node.yaml` avec `publish_tf: true` et une fréquence de 30 Hz |
| `nav_solution/config/nav2_params_mapping.yaml` | copie de `nav2_params.yaml` sans le filtre de zone interdite, qui était lié à la carte du café |
| `nav_solution/config/mapping.rviz` | vue RViz légère sans panneaux Nav2 |
| `nav_solution/config/slam_toolbox_params.yaml` (modifié) | `scan_topic: /scan_filtered` au lieu de `/scan` |

Les fichiers de la phase 1 (`task_world.sdf`, `task.launch.py`, `gz_bridge.yaml`, `ekf_node.yaml`, `nav2_params.yaml`, `nav2.launch.py`) **ne sont pas modifiés**.

### Problème 1 : odométrie fausse en rotation (corrigé)

Mesure par rapport à la vraie position du robot dans Gazebo :

| Mouvement | Réel | Odométrie des roues | IMU | EKF |
|---|---|---|---|---|
| Rotation à 0,3 rad/s | 157,0° | 206,3° (×1,31) | 156,9° | 157,0° |
| Rotation à 0,8 rad/s | 280,2° | 368,1° (×1,31) | 280,0° | 280,2° |
| Ligne droite de 0,45 m | 0,456 m | 0,454 m | — | — |

- **Cause** : slam_toolbox lisait la TF `odom → base_footprint` publiée par le plugin DiffDrive de Gazebo, qui surestime toutes les rotations de 31 %. L'EKF, qui est juste parce qu'il utilise l'IMU, ne publiait pas de TF (`publish_tf: false`). C'est ce qui donnait les murs dupliqués et tournés sur la carte.
- **Correction** : l'EKF publie la TF (`ekf_arena.yaml`), et la TF du plugin n'est plus transmise à ROS (`gz_bridge_arena.yaml`).
- **Après correction** : la TF est juste à ×1,000. Après plusieurs rotations, la position SLAM est à 6 cm et 0,8° de la vraie position.
- **Non élucidé** : le plugin calcule avec un écartement de 0,3409 m, alors que les roues du modèle sont écartées de 0,393 m. Cet écart n'explique que ×1,15 sur les ×1,31 mesurés.

### Problème 2 : les roues vues comme des obstacles (corrigé)

- slam_toolbox lisait `/scan` brut. Sur un scan, 101 points étaient à moins de 45 cm : le châssis et les roues.
- Le projet avait déjà le filtre `laser_filter`, défini dans `lasers_scan_filters.yaml`, qui publie `/scan_filtered` sans ces points. Nav2 l'utilisait déjà.
- slam_toolbox et RViz utilisent maintenant `/scan_filtered`.

### Approche d'un tag : test T2, dans l'ancien monde du café

Le test consistait à aller à 0,5 m face au tag avec Nav2 seul, à partir de la position du tag dans la carte.

| Tolérances Nav2 | Résultat |
|---|---|
| Actuelles (25 cm, angle libre) | 0 visite sur 2 : le robot s'arrête à environ 0,75 m |
| Serrées (8 cm, 0,1 rad) | 1 visite sur 2, en 129 s ; l'autre essai a abandonné |

Deux causes :
- la position estimée du tag est fausse de 14 à 26 cm ;
- Nav2 est imprécis à l'arrivée.

**Prochaine étape proposée** : une approche finale guidée par la caméra, en boucle fermée sur la pose du tag vue en direct.

---

## 7. Points ouverts

- **Le vrai robot a déjà un nœud `scan_filter` qui publie `/scan_filtered`.** Il faut vérifier ce qu'il retire, et ne pas publier le nôtre sous le même nom.
- **Sur le vrai robot, la TF est `odom → base_link` : il n'y a pas de `base_footprint`.** Les configs devront être adaptées.
- **IMU du vrai robot** : `/camera1/camera1/imu` et `/camera2/camera2/imu`, publiées en BEST_EFFORT.
- **Les commandes du vrai robot** passent par `/cmd_vel`, puis par le nœud `obstacle_avoidance` du robot, avant d'arriver aux roues. On ne connaît pas sa distance d'arrêt.
- **Mesures à prendre sur place** : hauteur des murets, dimensions des tables, et quelle caméra est la caméra haute.
