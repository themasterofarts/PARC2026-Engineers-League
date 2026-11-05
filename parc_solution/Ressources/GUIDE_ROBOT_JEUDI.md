# Robot réel : commandes pour les créneaux d'essai

Branche `peace/phase-reelle`. Mis à jour le 4 novembre après les messages des organisateurs :
- commandes de vitesse sur `/botwheel_explorer/cmd_vel`, en `TwistStamped` ;
- cartes faites à l'avance autorisées ;
- l'arbitre tourne sur le robot, et `parc_msgs` est dans leur espace de travail `~/ros2_ws` ;
- démarrage vocal mis de côté : le départ se fait par commande.

**Mode de course recommandé : cartographie en direct + positions des tags notées la veille** (`indices:=`, sans `carte:=`). En simulation, ce mode a réussi 4/4 en 179 s. La carte enregistrée avec AMCL (`carte:=`) a décroché en simulation (1/4) : c'est une option à ne pas utiliser tant que ce n'est pas résolu.

Notation :
- `ROBOT` = `utilisateur@ip` du robot, donné par les organisateurs ;
- `~/teams/MA64` = dossier de l'équipe sur le robot.

## Ce que le créneau du 5 novembre a appris (à lire d'abord)

- **Consigne des organisateurs :** notre code vit uniquement dans `~/teams/MA64`, code dans `src/`, compilé sur le robot avec `colcon build` depuis `~/teams/MA64`. Jamais de `build/` ni `install/` copiés, et rien en dehors de ce dossier. `deploy.sh` fait tout cela.
- **Nettoyer une fois** l'ancien dossier, créé avant cette consigne : `rm -rf ~/MA64` sur le robot.
- **Caméra des tags = `camera2`** (0,92 m, horizontale). `camera1` est à 0,53 m et regarde le sol, inclinée de 67°. C'est l'inverse du simulateur. `robot.conf` et les commandes utilisent désormais `camera2`.
- **La carte du 5 novembre est inutilisable** : copies de l'arène tournées les unes sur les autres. Causes possibles : téléop trop rapide (0,5 m/s, 1 rad/s par défaut) ou odométrie fausse en rotation. **Au prochain créneau : diagnostic en mouvement jusqu'au bout, puis cartographie lente** (volet 7 : 0,15 m/s et 0,5 rad/s).
- **Hauteur des tags :** centre à environ 0,75 m, caméra à 0,92 m, champ vertical de ±21°. À l'arrêt (centre du robot à 0,60 m), le bas du tag est au bord de l'image : **vérifier au volet 6 que le tag est vu à 0,60 m**.

## Plan du créneau 1 (15 min)

| Min | Quoi | Section |
|---|---|---|
| 0–2 | `deploy.sh`, puis lire ce qui manque | 1 |
| 2–6 | diagnostic au repos, puis tests de mouvement | 2 |
| 6–8 | étalonnage de `tag_size` au mètre ruban | 3a |
| 8–13 | cartographie à la téléop, sauvegarde de la carte et des tags | 3b |
| 13–15 | course d'essai avec leur arbitre (si le temps le permet), sinon rapatriement | 4 |

À la fin, **coupe tout (`Ctrl+C`)** : on ne publie sur `/parc/` que pendant notre créneau.

### Les fenêtres toutes prêtes

```bash
~/Projects/parc_ingé_league/src/parc_solution/outils_robot/ouvrir_terminaux.sh
```

Le script ouvre trois fenêtres Terminator de 4 volets, numérotés de 1 à 12 dans l'ordre de lancement. Chaque volet contient sa commande déjà tapée : Entrée la lance, et la ligne reste modifiable avant.
- **« 1/3 Préparation »** : 1 déployer, 2 diagnostic au repos, 3 diagnostic en mouvement, 4 page de l'arbitre.
- **« 2/3 Cartographie »** : 5 cartographie, 6 étalonnage, 7 téléop, 8 sauvegarde de la carte et des tags.
- **« 3/3 Course »** : 9 course, 10 état de l'arbitre, 11 départ, 12 rapatrier les résultats.

Avant de commencer, remplis une fois `outils_robot/robot.conf` :
- `ROBOT=utilisateur@ip` ;
- `CAMERA` ;
- `TAILLE` : la taille de tag étalonnée, que tu mets à jour après l'étalonnage.

Les commandes lisent ce fichier au moment où tu appuies sur Entrée.

Côté robot, chaque étape tourne dans sa propre session `tmux` (`diag`, `carto`, `course`…) :
- **si le Wi-Fi coupe**, rien ne s'arrête. Relance la même commande pour revenir dans la session ;
- **pour arrêter une étape**, fais `Ctrl+C` dans son volet ;
- **pour relancer une étape** dans la même session, comme le départ pour une 2ᵉ course, retape la commande dans son volet.

Tous les volets réutilisent la connexion SSH ouverte par `deploy.sh`, donc le mot de passe ne se tape qu'une fois.

---

## 0. Avant le créneau

Sur le PC :

```bash
cd ~/Projects/parc_ingé_league
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav_solution parc_robot_bringup
```

À préparer :
- le chargeur, et Firefox fermé ;
- un mètre ruban, et du scotch pour marquer le point de départ.

À demander aux organisateurs :
- l'adresse IP et l'utilisateur SSH ;
- **leur accord avant d'installer quoi que ce soit**. `deploy.sh` affiche à la fin la liste de ce qui manque. Liste probable, si rien n'est installé :
  ```
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-laser-filters
  ros-jazzy-apriltag-ros ros-jazzy-teleop-twist-keyboard python3-opencv tmux
  ```

---

## 1. Déployer le code (environ 1 min)

```bash
cd ~/Projects/parc_ingé_league/src/parc_solution/outils_robot
ROBOT=utilisateur@ip ./deploy.sh
```

Le script :
- prépare et vérifie le dossier à envoyer (`outils_robot/preparer_envoi.sh` : compilation dans un espace propre, executables, launch, configuration, analyse du code). Il s'arrête si une vérification échoue ;
- copie ce dossier dans `~/teams/MA64` : `nav_solution`, `diagnostic_robot.py`, `LISEZMOI.txt` (les commandes en résumé) et `VERSION.txt` (la version envoyée) ;
- compile sur le robot ;
- écrit `~/teams/MA64/env.sh`, qui charge ROS, puis leur espace de travail `~/ros2_ws` (`parc_msgs`), puis le nôtre ;
- vérifie tout ce dont notre code a besoin (Nav2, slam_toolbox, laser_filters, apriltag_ros, `parc_msgs`, OpenCV, `tmux`) et affiche ce qui manque.

Options :

| Option | Quand l'utiliser |
|---|---|
| `BASE_SETUP=/chemin/setup.bash` | si l'espace de travail de base du robot n'est pas `~/ros2_ws/install/setup.bash` |
| `--parc-msgs` | **ne plus utiliser** : les organisateurs ont installé `parc_msgs`. Seulement en secours s'il disparaît |

**Sur le robot, tout se lance dans `tmux`.** Si le Wi-Fi coupe la connexion SSH, tout ce qui a été lancé directement dans cette session s'arrête avec elle. Dans `tmux`, ça continue de tourner.

```bash
ssh utilisateur@ip
tmux new -s ma64            # après une coupure : tmux attach -t ma64
source ~/teams/MA64/env.sh        # à refaire dans chaque terminal tmux
```

Raccourcis `tmux` : `Ctrl+B` puis `C` ouvre un nouveau terminal, `Ctrl+B` puis `N` passe au suivant.

**Si `tmux` n'est pas installé** (et en attendant l'accord des organisateurs) : ouvre plusieurs connexions `ssh` depuis plusieurs terminaux du PC, et garde le PC près du robot. Une coupure du Wi-Fi arrêterait alors ce qui tourne.

---

## 2. Diagnostic (environ 4 min)

```bash
cd ~/teams/MA64/outils
python3 diagnostic_robot.py                         # robot immobile
python3 diagnostic_robot.py --mouvement --no-bag    # A : un tour sur place (zone libre) ; B : robot face à un mur à 1,2-2 m
```

Les mouvements sont envoyés en `TwistStamped` sur `/botwheel_explorer/cmd_vel`. `Ctrl+C` arrête le robot. Le test B s'arrête de lui-même à 0,30 m du mur.

Rapatrier les résultats sur le PC :

```bash
scp -r ROBOT:teams/MA64/outils/diag_MA64_* ~/Projects/parc_ingé_league/src/parc_solution/Ressources/
```

Ce que ces résultats décident :

| Résultat | Décision |
|---|---|
| Fréquence de `/botwheel_explorer/cmd_vel` au repos | **au-dessus de 0 Hz, quelqu'un y publie déjà** (`obstacle_avoidance` ?). Ses commandes se mêleraient aux nôtres : demander aux organisateurs de l'arrêter pendant notre course |
| Hauteur de `camera1_link` et `camera2_link` | argument `camera:=` des commandes suivantes |
| Facteur de l'odométrie en rotation | si l'écart dépasse quelques %, plan B avec un EKF |
| Points à moins de 40 cm sur le scan | taille de la boîte dans `lasers_scan_filters_robot.yaml` |

---

## 3a. Étalonner la distance (2 min, à faire AVANT la cartographie)

**Règle précisée par l'organisateur le 5 novembre :** c'est le **point du robot le plus proche du tag** (son avant) qui doit être à environ 0,5 m. C'est jugé à l'œil : le robot doit clairement aller jusqu'au tag, pas le viser de loin.

Le robot vise 0,60 m entre **son centre** et le tag, ce qui met l'avant à environ 0,35 m si l'avant est à 0,25 m du centre. Une visite n'est acceptée qu'entre 0,55 et 0,65 m **selon la caméra**. La caméra doit donc mesurer juste, et il faut vérifier la position de l'avant.

**Vérification de l'avant :** quand le robot est arrêté devant un tag pendant la course d'essai, mesure au mètre ruban la distance entre l'avant du robot et le tag. Si elle ne te convient pas (cible : un peu moins de 0,5 m), change `visit_distance` dans `~/teams/MA64/ws/src/nav_solution/config/mission_robot.yaml` de la différence (avant à 0,45 m alors que tu veux 0,40 m : 0,60 → 0,55), recompile (`cd ~/teams/MA64/ws && colcon build --packages-select nav_solution`), puis relance.

En simulation, elle sous-estimait les distances de 6 %. Après étalonnage, l'écart est tombé à 3–5 mm.

1. Lance le terminal 1 de la section 3b ci-dessous, avec `tag_size:=0.165`.
2. Place le robot à la main face à un tag, avec son centre à environ 0,60 m du tag. Mesure au mètre ruban la distance entre **le centre du robot et la face du tag**.
3. Dans un autre terminal sur le robot, lis ce que mesure la caméra (remplace 4 par le numéro du tag) :

   ```bash
   ros2 run tf2_ros tf2_echo base_link tag_4    # distance = racine(x² + y²)
   ```

4. Nouvelle taille = `tag_size` × (distance au ruban / distance de la caméra).

   Exemple : 0,165 × 0,60 / 0,57 = 0,174. Utilise ensuite `tag_size:=0.177` **dans toutes les commandes**.
5. Relance le terminal 1 avec la nouvelle taille, puis refais la mesure pour vérifier.

Note aussi où se trouve `base_link` sur le robot, et **la distance entre `base_link` et l'avant du robot**. Les juges regardent l'avant : si cette distance n'est pas d'environ 0,25 m, dis-le-moi.

## 3b. Cartographier à la téléop (environ 5 min)

**Marque d'abord au scotch le point de départ et la direction du robot.** Ce départ définit l'origine de la carte. Pour la course, le robot devra repartir du même endroit, dans la même direction.

Terminal 1, sur le robot :

```bash
ros2 launch nav_solution robot.launch.py mission:=false nav2:=false camera:=camera2 tag_size:=TAILLE_ETALONNEE
```

Terminal 2, sur le robot. La téléop passe en `TwistStamped`, directement aux roues, **sans anti-collision** : roule doucement.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/botwheel_explorer/cmd_vel -p stamped:=true -p frame_id:=base_link -p speed:=0.15 -p turn:=0.5
```

Pendant la cartographie, chaque tag vu s'affiche dans le terminal 1 : `Tag N localise`. Passe devant chacun des 4 tags.

Les organisateurs peuvent afficher la carte sur un écran. Sinon, sur le PC, si le Wi-Fi laisse passer ROS 2 :

```bash
export ROS_DOMAIN_ID=0 && ros2 topic list
rviz2 -d ~/Projects/parc_ingé_league/install/nav_solution/share/nav_solution/config/robot.rviz
```

**Sauvegarder à la fin**, dans un 3e terminal sur le robot, sans arrêter le terminal 1 :

```bash
mkdir -p ~/teams/MA64/cartes
ros2 run nav2_map_server map_saver_cli -f ~/teams/MA64/cartes/arene     # arene.pgm + arene.yaml
ros2 run nav_solution sauver_tags ~/teams/MA64/cartes/tags.yaml          # position des 4 tags
```

Vérifie que `sauver_tags` affiche bien les 4 tags : `[1, 4, 7, 9]`. Rapatrie ensuite une copie sur le PC :

```bash
scp -r ROBOT:teams/MA64/cartes ~/Projects/parc_ingé_league/src/parc_solution/Ressources/carte_robot
```

---

## 3c. Masque des plateaux et des assises (volet 9, sur le PC, 3 min)

**Pourquoi :**
- Le LiDAR ne voit que les pieds des 2 tables et des 3 chaises. Les plateaux et les assises débordent au-dessus.
- Le masque les dessine comme des zones interdites pour Nav2, comme en phase 1. La marge d'inflation s'applique ensuite aussi autour d'eux.
- Les chaises ne seront pas déplacées : elles peuvent donc aller dans le masque.

**Ce que fait le volet 9 :**
1. Il rapatrie `arene.pgm` et `arene.yaml` dans `Ressources/carte_robot/`.
2. Il ouvre la carte dans une fenêtre de dessin.
3. Il renvoie `arene_keepout.*` dans `~/teams/MA64/cartes/` sur le robot.

**Dans la fenêtre de dessin :**
- **clic gauche** : ajoute un coin. Pour chaque table, clique les 4 coins du plateau autour de ses pieds. Pour chaque chaise, clique les 4 coins de l'assise et des accoudoirs, autour des 4 pieds ;
- **clic droit** ou **Entrée** : ferme le polygone ;
- **`u`** : annule le dernier coin, ou le dernier polygone ;
- **`s`** : enregistre et quitte. **`q`** : quitte sans rien enregistrer.

L'outil ajoute 5 cm de marge autour de chaque polygone (`--marge`). Les photos de l'arène aident à repérer les objets sur la carte.

**Ensuite :** le volet Course (10) charge automatiquement le masque s'il est sur le robot. Il affiche alors `masque : masque:=…/arene_keepout.yaml`.

Le masque est exprimé dans le repère de la carte, donc **la course doit partir du point marqué au scotch**.

En simulation, la même chose : `ros2 launch nav_solution mission_sim.launch.py monde:=finale masque:=…/carte_simu_finale/masque_finale.yaml`.

## 4. Course complète

L'arbitre officiel tourne sur le robot. **Ne lance jamais notre faux arbitre sur le robot** : il publierait aussi sur `/parc/`, ce qui est interdit et perturberait le vrai. On ne publie sur `/parc/` que pendant notre créneau.

Terminal 1, sur le robot, avec le robot posé sur le point de départ marqué :

```bash
ros2 launch nav_solution robot.launch.py camera:=camera2 tag_size:=TAILLE_ETALONNEE indices:=$HOME/teams/MA64/cartes/tags.yaml
```

- **`indices:=`** : le robot va directement là où chaque tag était pendant la cartographie. L'approche finale se fait toujours sur la caméra en direct. Si un tag n'est pas là, il explore. La carte est refaite en direct ; son origine est la pose de départ, d'où l'importance du point marqué.
- **Sans `indices:=`** : le robot explore pour trouver chaque tag (4/4 en 152 s en simulation).
- **Option `carte:=$HOME/teams/MA64/cartes/arene.yaml`** (AMCL sur la carte enregistrée) : à éviter pour l'instant, la localisation a décroché en simulation. Si la course part ailleurs que du point marqué : `x0:= y0:= yaw0:=`.

Terminal 2 : suivre l'état de la course, et le laisser ouvert.

```bash
ros2 topic echo /parc/run_status
```

**Demande à un organisateur d'armer la course.** L'ordre des tags est alors publié, et l'état passe à `ARMED` (`state: 1`). Pour vérifier l'ordre :

```bash
ros2 topic echo /parc/tag_order --qos-durability transient_local
```

**Départ**, une fois que le terminal 1 affiche `attente du depart` et que l'arbitre est à l'état `ARMED` :

```bash
ros2 topic pub --once /ma64/start std_msgs/msg/String "{data: go}"
```

**Ensuite, on ne touche plus à rien** : ni le robot, ni l'ordinateur, ni la session SSH.

Pendant et après la course, la page web de l'arbitre montre ce qu'il a reçu, avec l'ordre, les visites et **nos images** : `http://IP_DU_ROBOT:8080/`. Vérifie que chaque visite affiche bien la photo du tag, et pas « No image ».

Le journal de la mission est écrit dans `~/teams/MA64/journaux/mission_*.json`. Rapatrie-le avec `scp`.

**Arrêt d'urgence** : `Ctrl+C` dans le terminal 1. Le robot s'arrête aussi tout seul quand l'arbitre passe à `FINISHED`, `TIMED_OUT` ou `TERMINATED`.

**Sécurité** : nos commandes vont directement aux roues, sans `obstacle_avoidance`. La sécurité repose donc sur Nav2 (`collision_monitor`, inflation de la costmap) et sur les contrôles du LiDAR du nœud de mission. Au premier essai, reste prêt à faire `Ctrl+C`.

À noter : le nœud d'amarrage de Nav2 (`docking_server`) déclare un topic `/cmd_vel` en `TwistStamped`. Il n'y publie que pendant un amarrage, et nous n'en demandons jamais. Comme `obstacle_avoidance` écoute `/cmd_vel` en `Twist`, ROS 2 ne relie pas les deux.

---

## 5. Simulation (sur le PC)

**Attention au Wi-Fi du robot.** Si le PC est sur `Flybox_ED67`, une simulation non isolée est visible du robot et des autres équipes, dans le même domaine ROS 0 : ses `/tf`, son horloge `/clock`, et les `/parc/` de notre faux arbitre. Elle reçoit aussi les leurs. Le 4 novembre, c'est arrivé : une autre machine publiait `/clock`, et la course 18 a été faussée.

Nos launch de simulation (`arena`, `mapping`, `mission_sim`) limitent donc la découverte à la machine elle-même (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`). **Ne lance pas d'autre outil ROS de simulation à la main sur ce Wi-Fi sans faire avant :**

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

```bash
~/Projects/parc_ingé_league/src/parc_solution/sim_outils/build_parc_msgs.sh   # une seule fois
source ~/ma64_parc_msgs_ws/install/local_setup.bash
C=~/Projects/parc_ingé_league/src/parc_solution/Ressources/carte_simu
ros2 launch nav_solution mission_sim.launch.py rapport:=/tmp/rapport.json                    # SLAM en direct
ros2 launch nav_solution mission_sim.launch.py carte:=$C/arene.yaml indices:=$C/tags.yaml   # sur la carte enregistrée
```

Ajoute `gui:=true rviz:=true` pour voir Gazebo et RViz, et `ordre:=[7,4,9,1]` pour imposer l'ordre des tags.

Le faux arbitre :
- arme la course au bout de 25 s ;
- envoie le départ 3 s plus tard ;
- note chaque visite avec la vraie position du robot : distance au tag, angle face au tag, immobilité pendant 3 s.

Son rapport est écrit dans le fichier passé à `rapport:=`, avec une image JPEG par visite.
