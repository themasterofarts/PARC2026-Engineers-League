# Compte rendu : simulateur officiel PARC 2026 (nuit du 4 au 5 novembre)

Rien n'est commité ni poussé. Les modifications sont dans le dossier de travail (`peace/phase-reelle`).

## En bref

- **Le challenge passe dans le simulateur officiel : 4/4 sur les seeds 7, 12 et 3**, avec des ordres différents (l'arbitre tire l'ordre au hasard). Les visites ont toutes été jugées valides par l'évaluateur, qui s'appuie sur la vraie pose Gazebo :
  - distance 0,51 m (centre du robot → tag) ;
  - angle de 1,3 à 1,7° (la tolérance est de ±12°) ;
  - robot immobile pendant 3 s.
- **Les courses précédentes ratées venaient surtout du robot qui se renversait sur une table.** Le LiDAR ne voit que la colonne de 4 cm des `cafe_table`. Le robot passait donc sous le plateau, montait sur le socle et basculait (tangage 90°). L'odométrie et le SLAM partaient ensuite en vrille, ce qui explique les tags mal placés et les « no valid path ».
  - La parade est **la même que celle prévue sur le vrai robot** : un masque keepout des tables et des chaises.
- **Quatre défauts de notre code, trouvés par ces courses, sont corrigés, plus un réglage.** Tous concernent aussi le vrai robot (voir plus bas, n° 2 à 5). Le paquet d'envoi au robot a été reconstruit et vérifié (`preparer_envoi.sh` : compilation, exécutables, launch, configs, Python OK).

## Résultats

| Course | Seed | Ordre | Résultat | Temps arbitre (horloge murale) | Remarques |
|---|---|---|---|---|---|
| s7c à s7f | 7 | — | 1/4 au mieux | — | sans masque : le robot monte sur la table centrale et se renverse (s7f : tangage 90°, z = 0,20 m) |
| **s7g** | 7 | 4, 9, 1, 7 | **4/4 FINISHED** | 1120 s (≈ 514 s simulées) | avec masque ; aucun contact ; inclinaison max 2,5° ; 2 « oublis » de tag à tort (corrigés depuis) |
| s12a | 12 | — | non partie | — | slam_toolbox resté `inactive` → pas de TF `map` → Nav2 abandonne (corrigé) |
| **s12b** | 12 | 7, 4, 1, 9 | **4/4 FINISHED** | 434 s | 1 contact du châssis avec une chaise pendant une exploration inutile (corrigé depuis) ; inclinaison max 7,3° |
| s12c | 12 | 9, 7, 4, 1 | **0/4, arrêtée** | — | tag 9 introuvable en 30 min : tour d'observation annulé près de lui (mur à 0,27 m sur le côté) (corrigé, n° 4) |
| **s12d** | 12 | 7, 4, 1, 9 | **4/4 FINISHED** | 1007 s | aucun contact ; inclinaison max 2,5° ; 11 min murales pour trouver le premier tag (exploration) |
| **s3a** | 3 | 4, 7, 1, 9 | **4/4 FINISHED** | 772 s | `max_distance` 4 m ; **contact avec le feuillage d'une plante**, inclinaison 12° (corrigé : plantes dans le masque) |
| **s3b** | 3 | 1, 7, 4, 9 | **4/4 FINISHED** | **527 s** (≈ 4 min simulées) | **tous les correctifs** ; aucun contact ; inclinaison max 2,5° ; meilleure course de la nuit |

Le simulateur tourne à environ 0,45 fois le temps réel sur le PC, et l'arbitre officiel chronomètre en temps réel (`time.monotonic`). Son temps est donc environ 2 fois trop long : avec `time_limit_s` par défaut (600 s), s7g aurait été coupée alors qu'elle tenait en 8,6 min simulées. Les courses ont été lancées avec `time_limit_s:=3600`.

Les rapports détaillés sont dans `~/ma64_courses_simu/<course>/evaluation.json` : pose de chaque visite, contacts, inclinaison. s7g est dans le scratchpad, perdu au redémarrage ; ses chiffres sont recopiés ci-dessus.

## Problèmes trouvés et corrigés

### 1. Le robot se renverse sur les tables (simulation, mais leçon pour le vrai robot)

- **Le modèle `cafe_table`** a un plateau de 0,91 m à 0,75 m de haut, une colonne de 4 cm et un socle de 0,56 m × 4 cm au sol. Le LiDAR ne voit que la colonne.
- **Correctif :** nouveau script `sim_outils/masque_simu_officielle.py`. Il lit le monde généré (`/tmp/parc_worlds/parc_random_seed<N>.sdf`) et dessine dans le repère `map` :
  - les tables ;
  - les chaises ;
  - les visiteurs assis (enveloppe de leur maillage de collision).

  Il ajoute aussi le **feuillage des plantes** (sphère de 0,33 m au-dessus d'un pot de 0,20 m), touché sur le seed 3. Il réutilise `generer_masque.py`. `sim_outils/simu_officielle/course.sh` le génère et le charge automatiquement (`MASQUE=0` pour s'en passer).
- **Sur le vrai robot :** c'est exactement l'étape **3c du guide (masque dessiné à la main sur la carte)**. Cette nuit a montré qu'elle est indispensable : sans elle, une table peut faire basculer le robot.
  - À dessiner : **tout ce qui déborde au-dessus de la hauteur du LiDAR** (plateaux, assises, dossiers, plantes, étagères), pas seulement ce que le LiDAR voit.

### 2. slam_toolbox parfois jamais activé (concerne aussi le vrai robot)

- **Cause :** le launch officiel de slam_toolbox l'active sur un événement de transition. L'événement s'est perdu : slam_toolbox est resté `inactive`, sans TF `map`, et Nav2 a abandonné au bout de 110 s. Le robot ne serait jamais parti.
- **Correctif dans `nav_solution/launch/robot.launch.py` :**
  - slam_toolbox est lancé avec `use_lifecycle_manager:=true` ;
  - un `lifecycle_manager_slam` (Nav2) le configure et l'active, en réessayant jusqu'à ce que les services répondent ;
  - il n'y a pas de bond (`bond_timeout: 0.0`), pour qu'un hoquet ne coupe pas le SLAM en pleine course.
- **Vérifié en simulation** (s12b, s12c) : `lifecycle_manager_slam: Managed nodes are active` 1 s après le lancement.
- **Sur le robot,** vérifier ce message dans le volet Course. `nav2_lifecycle_manager` fait partie de Nav2, donc rien à installer.

### 3. Tags « oubliés » à tort (concerne aussi le vrai robot)

- **Symptôme :** la mission arrivait au point de vue d'un tag bien localisé (à 2–6 cm près) et ne le voyait pas. Elle l'oubliait alors et repartait explorer, ce qui coûtait 1 à 3 min par tag.
- **Cause :** Nav2 a une tolérance de cap de 3,15 rad (voulue : c'est l'approche caméra qui règle le cap final). Il déclare donc « arrivé » avec le robot tourné de travers (97° au lieu de 195° sur le seed 12). Le petit balayage de ±34° ne suffisait pas à ramener le tag dans l'image.
- **Correctifs dans `mission_node.py` (`visiter`) :**
  - après la navigation, si le tag n'est pas vu, le robot **se tourne vers la position mémorisée du tag** avant le balayage ;
  - le tag n'est oublié que si Nav2 a réellement atteint le point de vue, ou après 2 échecs ;
  - le message d'oubli indique maintenant la pose du robot et le résultat de la navigation.
- **Effet de bord évité :** l'exploration inutile de s12b avait fait toucher une chaise au robot.

### 4. Tour d'observation annulé près d'un mur (concerne aussi le vrai robot)

- **Symptôme (s12c) :** pour tourner sur place, il faut 0,38 m libres autour du robot. Près d'un mur ou d'un pied de table, le tour de 360° qui cherche les tags était simplement annulé. `degager()` avance ou recule, ce qui ne sert à rien quand l'obstacle est sur le côté.
- **Correctif dans `mission_node.py` (`regarder_autour`) :** si l'obstacle est trop près, Nav2 emmène d'abord le robot 0,4 m à l'opposé, en manœuvrant. Le tour n'est annulé que si c'est encore bloqué. Message : `tour d'observation : obstacle a … m, on s'en ecarte`.
- La recherche de l'obstacle le plus proche est mise en commun avec `degager()` (`obstacle_proche()`).

### 5. Détection des tags jusqu'à 4 m (concerne aussi le vrai robot)

- **Réglage :** `max_distance` passe de 3,0 à 4,0 m dans `config/apriltag.yaml`.
  - La limite de 3 m avait été posée contre des tags mal placés, mais leur vraie cause était le robot renversé.
  - À 3 m, l'exploration passait à côté de tags visibles (s12d : point de vue à 3,3 m du tag 7, non retenu).
- **Vérifié sur s3a :** les 4 tags sont localisés à moins de 9 cm, dont 3 dès la première minute. La position reste moyennée sur 5 mesures, et l'approche finale est guidée par la caméra.
- Sur le robot, le tag fait 0,165 m (0,12 m en simulation) : il est mieux détecté de loin.

### Autres ajouts

- `evaluateur_sim.py` mesure l'**inclinaison maximale** du robot, ce qui détecte un renversement.
- `course.sh` republie le départ tant que la mission ne l'a pas reçu. Un `ros2 topic pub --once` était parti avant la découverte DDS, et le robot est resté 3 min à attendre.

## Défauts du simulateur officiel, à signaler aux organisateurs

1. **`/sim/pose` ne publie rien.** Le pont écoute le topic Gazebo `sitoe_robot/pose`, alors que la pose est sur `/model/sitoe_robot/pose`. Nous avons fait notre propre pont vers `/verite/robot_pose`.
2. **Odométrie fausse en rotation :**
   - `gz_control.xacro` déclare `wheel_separation` 0,3409 alors que les roues sont à ±0,19649 m ;
   - les rotations mesurées par l'odométrie sont donc multipliées par 1,31, et le SLAM s'en trouve gêné ;
   - nous avons étalonné **0,445 dans notre copie locale uniquement** (`~/parc2026_ws/src/PARC2026-Engineers-League/parc_robot_description/urdf/gz_control.xacro`, c'est la seule modification de leur dépôt, visible avec `git status`). **Ne jamais la pousser.** Pour revenir à l'original : `git checkout parc_robot_description/urdf/gz_control.xacro`.
3. **La caméra du bas (`/camera2/scan`) voit le sol à 0,5–0,7 m.** Nous l'avons retirée des costmaps.
4. **L'arbitre chronomètre en temps réel** (`time.monotonic`) alors que la simulation tourne plus lentement : le temps affiché est faux sur un PC moyen.
5. **L'arbitre ne vérifie ni la distance ni l'angle des visites :** c'est le juge humain qui le fait. Notre robot s'arrête à 0,47 m mesuré par la caméra, soit **0,51 m entre le centre du robot et le tag** dans Gazebo. À confirmer avec les organisateurs : d'où mesurent-ils les 0,5 m (centre, avant du robot, caméra) ? Si c'est le centre, réduire `visit_distance` de 0,43 à 0,40 dans `mission_robot.yaml`.

## Ce qui reste / prochaines étapes

- **Vrai robot (prioritaire) :**
  - faire le masque des tables et chaises de l'arène (guide, 3c) ;
  - vérifier au lancement `lifecycle_manager_slam: Managed nodes are active` ;
  - mesurer au mètre ruban la distance réelle de visite ;
  - si un tour d'observation s'annule ou si un tag est « oublié », lire le message : il donne maintenant la pose du robot et la cause.
- **Point faible restant :** le temps pour trouver le premier tag dépend du hasard de l'exploration (1 à 11 min murales selon les courses). Sur le robot, si la carte faite à l'avance donne les positions des tags (`indices:=`), ce temps disparaît. L'option existe déjà (guide, section 4).
- **Marge de temps :** sur le seed 7, la course a pris 8,6 min simulées sur 10. Les pertes de temps venaient des oublis à tort, maintenant corrigés. À surveiller sur le robot.
- **Rejouer en simulation :**
  ```bash
  ~/Projects/parc_ingé_league/src/parc_solution/sim_outils/simu_officielle/course.sh 7 essai
  # résultats : ~/ma64_courses_simu/essai/ (evaluation.log, evaluation.json, robot.log)
  # tout arrêter : sim_outils/simu_officielle/stop.sh
  ```
  Donner un seed fixe : le masque lit le monde `parc_random_seed<N>.sdf`.
- **Attention réseau :** la simulation tourne sur le domaine ROS 64 avec `GZ_IP=127.0.0.1`, donc invisible du robot (domaine 0). Ne pas la lancer pendant notre créneau sur le robot.

## Fichiers modifiés ou créés cette nuit (depuis le dernier compte rendu)

| Fichier | Changement |
|---|---|
| `nav_solution/launch/robot.launch.py` | slam_toolbox activé par `lifecycle_manager_slam` |
| `nav_solution/nav_solution/mission_node.py` | `visiter` : se tourner vers le tag, oubli seulement si navigation réussie ou 2 échecs, log détaillé ; `regarder_autour` : s'écarter avant le tour ; `obstacle_proche()` |
| `nav_solution/nav_solution/apriltag_tf_node.py` | filtres `max_distance` / `max_view_angle_deg` (détections lointaines ou rasantes ignorées) |
| `nav_solution/config/apriltag.yaml` | `max_distance` 4,0 m |
| `sim_outils/masque_simu_officielle.py` | nouveau : masque keepout depuis le monde généré (tables, chaises, visiteurs, plantes) |
| `sim_outils/evaluateur_sim.py` | inclinaison maximale |
| `sim_outils/simu_officielle/course.sh` | masque automatique, départ republié |
