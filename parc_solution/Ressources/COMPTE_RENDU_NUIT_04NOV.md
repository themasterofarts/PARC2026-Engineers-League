# Compte rendu de la nuit du 4 novembre

> **Mise à jour du 4 novembre (matin)** : le démarrage vocal a été retiré à ta demande. Les fichiers `demarrage_vocal.py` et `outils_robot/vosk/` et l'option `--vosk` n'existent plus. Le départ se fait par commande sur `/ma64/start`, et une couche vocale pourra s'y brancher plus tard. Les mentions de la voix ci-dessous décrivent l'état de la nuit.

Rien n'est commité. Tout est sur `peace/phase-reelle`, dans le dossier de travail. Les commandes pour jeudi sont dans **`GUIDE_ROBOT_JEUDI.md`**.

## En bref

- **Tout le code de la course est écrit et testé en simulation**, de bout en bout, avec un faux arbitre qui reproduit les 5 topics `/parc/` du PDF :
  - mission, exploration, approche guidée par la caméra ;
  - visite (`TagVisit` + JPEG) ;
  - démarrage vocal ;
  - launch du robot ;
  - déploiement.
- **Résultats en simulation, avec la dernière version :**
  - course 12, sans aucune aide (les tags sont trouvés par l'exploration) : 4/4 en 152 s ;
  - course 11, avec les positions notées pendant une cartographie : 4/4 en 179 s ;
  - course 14 : 3/4. Le premier tag de l'ordre, le 7, n'a pas été trouvé à temps ; les 3 suivants ont été validés.

  À chaque fois, la course s'est terminée automatiquement (`FINISHED`) au dernier tag de l'ordre.
- **Les 7 premières courses n'avaient validé qu'un seul tag.** La nuit a surtout servi à trouver pourquoi, voir « Problèmes trouvés et corrigés ».
- **Précision des visites**, mesurée avec la vraie position du robot dans Gazebo :
  - angle : 1,5 à 1,8° (le guide autorise ±12°) ;
  - distance : 0,56 à 0,59 m pour un objectif de 0,5 m, car la caméra simulée sous-estime la distance de 4 à 5 cm. Le guide ne chiffre pas de tolérance. **Sur le robot, il faut étalonner au mètre ruban** (2 min, procédure dans le guide, section 3).

## Ce qui a été écrit

| Fichier | Rôle |
|---|---|
| `nav_solution/nav_solution/mission_node.py` | la course complète (détails ci-dessous) |
| `nav_solution/nav_solution/faux_arbitre.py` | les 5 topics `/parc/`, la vraie position Gazebo, la détection d'un robot renversé, un rapport JSON avec les images |
| `nav_solution/nav_solution/demarrage_vocal.py` | Vosk hors ligne sur `/speech_audio`, grammaire go / allez / vai, publie le mot sur `/ma64/start` |
| `nav_solution/nav_solution/sauver_tags.py` | enregistre la position des tags vus pendant la cartographie (option `indices:=`) |
| `nav_solution/launch/robot.launch.py` | vrai robot : filtre LiDAR → `/scan_ma64`, SLAM, AprilTag (`camera:=`, `tag_size:=`), Nav2, mission, voix (`vocal:=true`), indices (`indices:=`). **Pas d'EKF** : le robot publie déjà `odom → base_link` |
| `nav_solution/launch/mission_sim.launch.py` | course complète en simulation (`ordre:=`, `indices:=`, `rapport:=`, `gui:=true`) |
| `config/nav2_params_robot.yaml` | `base_link`, `/botwheel_explorer/odom`, sortie `/cmd_vel`, caméras par `/camera1/scan` et `/camera2/scan`, 0,25 m/s |
| `config/slam_toolbox_robot.yaml`, `config/lasers_scan_filters_robot.yaml`, `config/mission_robot.yaml`, `config/robot.rviz` | configuration du vrai robot |
| `sim_outils/parc_msgs/` et `build_parc_msgs.sh` | copie exacte des messages de l'arbitre. Elle se compile dans `~/ma64_parc_msgs_ws`, parce que le « é » du chemin du projet casse rosidl |
| `outils_robot/deploy.sh` | copie et compilation sur le robot, écrit `~/MA64/env.sh`, signale si `parc_msgs` ou `apriltag_ros` manquent |
| `outils_robot/vosk/` | Vosk pour aarch64, ses dépendances et le modèle anglais, pour installer hors ligne (ignorés par git) |

Le nœud de mission déroule la course ainsi :
1. Il attend l'ordre des tags. Le robot ne bouge pas.
2. Il attend le départ sur `/ma64/start`, puis publie un seul `StartSignal`.
3. Pour chaque tag de l'ordre :
   - si sa position est inconnue, il explore : points de vue choisis sur la costmap de Nav2, et à chaque point, un tour par pas de 45° avec une pause pour que l'image soit nette ;
   - Nav2 l'amène à 1 m devant le tag ;
   - l'approche finale se fait en boucle fermée sur la caméra jusqu'à 0,5 m ;
   - un contrôle final vérifie la distance et l'angle ;
   - il publie le `TagVisit` avec l'image JPEG, reste immobile 3,5 s, puis recule de 0,3 m.
4. Il saute un tag si son budget de temps est dépassé.
5. Il publie `Finish` si le dernier tag n'a pas été signalé.
6. Il s'arrête sur `FINISHED`, `TIMED_OUT` ou `TERMINATED`, et écrit un journal JSON.

## Problèmes trouvés et corrigés

| Problème constaté | Cause | Correction |
|---|---|---|
| Nav2 répond « no valid path found », des zones entières sont coupées | la couche voxel de la caméra dans la costmap **globale** laisse des tables fantômes qu'aucune mesure n'efface | couche voxel retirée de la costmap globale (`nav2_params_mapping.yaml`) |
| Le robot reste bloqué, avec des milliers de « collision ahead » | même défaut dans la costmap **locale** : quand le robot simulé penche, la caméra marque le sol, et ces marques restent | couche voxel retirée aussi de la costmap locale. **1 tag sur 4 avant, 4 sur 4 après** |
| Le robot monte sur les pieds des tables et se renverse | barres au sol à 3 cm, sous le plan du LiDAR (3,8 cm) ; filtre LiDAR de ±0,35 m qui cachait aussi les obstacles proches | barres à 6 cm dans le modèle ; boîte du filtre mesurée sur le scan (x −0,20 à +0,22 m, y ±0,27 m) ; rotations sur place refusées si un obstacle est à moins de 0,38 m |
| Les roues dépassent de l'empreinte Nav2 | empreinte de ±0,20 m en y, roues à ±0,245 m | empreinte de ±0,25 m |
| Nav2 refuse de repartir après une visite | à 0,5 m du poteau, l'avant du robot est dans la zone inscrite | recul de 0,3 m en ligne droite après chaque visite |
| Tags jamais détectés pendant les tours sur place | rotation continue : trop peu d'images par tag | tour par pas de 45°, avec 1,2 s de pause |
| La mission consomme trop de CPU | images brutes décodées en permanence | abonnement à la caméra seulement au moment de la visite |
| `Ctrl+C` n'arrête pas le nœud de mission (tué de force) | le gestionnaire de signal de rclpy ; attente sur une horloge simulée arrêtée | gestionnaire désactivé, attentes bornées en temps réel. Le journal est bien écrit |
| `env.sh` renvoie une erreur sans Vosk | ligne `[ … ] && …` en fin de fichier | réécrit avec des `if` |
| Robot coincé contre un poteau, Nav2 ne repart plus | obstacle à moins de 0,3 m | avant chaque trajet, le robot recule ou avance de 20 à 25 cm à l'opposé de l'obstacle |
| Approche finale lancée de loin quand Nav2 échoue | l'approche roule en ligne droite, sans planifier de chemin | approche seulement à moins de 1,6 m du tag, sinon nouveau trajet Nav2 |

Les fichiers de la phase 1 ne sont pas touchés : `nav2_params.yaml`, `lasers_scan_filters.yaml`, `task_world.sdf`, etc. Toutes les corrections de navigation sont dans des copies propres à l'arène ou au robot.

## Résultats des 14 courses en simulation

| Course | Version | Tags signalés |
|---|---|---|
| 1 à 3 | premières versions | 0 ou 1 (le robot s'est renversé en course 3) |
| 4 à 7 | voxel dans la costmap locale | 1 sur 4 à chaque fois (le tag 4, visible depuis le départ) ; 3 inclinaisons de 15° |
| 8 à 10 | + positions notées | 1 sur 4 : la costmap locale bloquait toujours |
| **11** | sans voxel, avec les positions notées | **4/4 en 179 s** |
| **12** | sans voxel, **sans aide** | **4/4 en 152 s** |
| 13 | sans voxel, ordre [7, 4, 9, 1] | 0 : le tag 7 n'est pas trouvé et il est sauté, puis le robot reste coincé contre le poteau du tag 1 |
| **14** | + le robot se dégage d'un obstacle collé, approche seulement à moins de 1,6 m | **3/4** : 7 sauté, puis 4, 9 et 1 validés ; `FINISHED` à 419 s |

**Point faible restant** : quand le premier tag de l'ordre est loin et mal orienté (tag 7, à 0,6 m d'un mur), l'exploration peut ne pas le trouver dans son budget, environ 220 s pour le premier tag. Les tags vus en chemin sont mémorisés, ce qui accélère la suite.

**Portée de détection en simulation** (caméra à 90° de champ, 640 px) :
- 100 % des images jusqu'à 2 m de face ;
- 0 % à 3 m ;
- à 30° de côté, la limite est entre 1,5 et 2 m.

La vraie D435i a un champ de 69°, donc une meilleure portée.

## Ce qui est sûr, ce qui ne l'est pas

**Testé :**
- les messages et la logique de l'arbitre ;
- la visite, l'image JPEG et l'immobilité (0 mm de déplacement en 3 s) ;
- les sauts de tags, la fin de course et `Finish` ;
- `deploy.sh` (compilation, `parc_msgs`, `env.sh`), testé localement sans SSH : il n'y a pas de serveur SSH sur le PC ;
- `robot.launch.py` : tous les nœuds Nav2 passent leur configuration ;
- `sauver_tags`.

**Pas encore vérifié :**
- **voix** : le nœud ne se déclenche pas sur du bruit, mais « go » n'a jamais été prononcé (pas de micro, pas de synthèse vocale). Le modèle anglais ne connaît pas « allez » ; « go » suffit d'après le guide ;
- **sur le vrai robot** :
  - **Nav2** : l'activation a échoué sans robot, faute de TF ; c'est attendu ;
  - **caméra** : laquelle est en haut, et la taille réelle des tags (étalonnage) ;
  - **`obstacle_avoidance`** : sa distance d'arrêt. Le robot doit pouvoir s'arrêter à 0,5 m du tag, soit environ 0,25 m devant ;
  - **odométrie** en rotation ;
  - **pieds de table** : sont-ils vus par le LiDAR ?
  - **`audio_common_msgs`** : est-il importable ?
  - **`parc_msgs` et `apriltag_ros`** : sont-ils présents ?
- **couche voxel retirée** : je l'ai retirée en simulation parce que ses marques fantômes bloquaient tout. Sur le robot, les caméras restent dans Nav2 par leurs scans (`/camera1/scan` et `/camera2/scan`), qui s'effacent normalement. Si elles posent problème, retire-les de `nav2_params_robot.yaml`, et `obstacle_avoidance` reste en sécurité.
- **positions notées (`indices:=`)** : leur utilisation vendredi doit être confirmée par les organisateurs, comme les cartes faites à l'avance. Sans elles, la course 12 a quand même réussi.

## Plan pour jeudi

**Créneau 1 :**
1. `deploy.sh` ;
2. diagnostic (`--audio`, puis `--mouvement`) ;
3. cartographie à la téléop, avec l'étalonnage de la distance et `sauver_tags` ;
4. rapatriement des résultats.

**Entre les créneaux, je règle :**
- la caméra et `tag_size` ;
- la boîte du filtre LiDAR ;
- la distance d'arrêt ;
- le format audio.

Je teste aussi Vosk sur ton « go » enregistré.

**Créneau 2 :** course complète.
