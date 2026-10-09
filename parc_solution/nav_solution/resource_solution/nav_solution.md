# EQUIPE 1: MA64 ROBOTICS

## Introduction

Dans cette tâche le robot Sito-E doit aller tout seul d'un point de départ à un objectif, dans une salle remplie de tables, de chaises et de personnes, et ce en moins de 10 minutes. 
Pour nous la navigation est la partie de base, sans elle le robot ne peut ni guider un visiteur ni rejoindre un point d'accueil donc aucun des services prévus pour le stade ne fonctionne.
L'intérêt d'un robot comme celui-ci est qu'il peut travailler en continu et décharger le personnel de tâches répétitives. 
Ses limites sont le coût, et surtout le comportement en conditions réelles, avec de la foule, des obstacles imprévus et des capteurs qui ne sont pas parfaits. Autour de personnes, une erreur de navigation n'est pas acceptable.


**Pays de l'équipe :** 
* BENIN

**Noms des membres de l'équipe:**

*  KPOKPO Sunday (chef d'équipe)
* EGOUDJOBI Peace Fiacre



## Dépendances

**Les packages nécessaires sont :**

* `nav2_simple_commander`: API Python pour la pile Nav2, permettant d'envoyer facilement des objectifs de navigation et de récupérer le statut de la mission.

  * `$ sudo apt-get install ros-jazzy-nav2-simple-commander`

* `slam_toolbox` : Package utiliser construire  la carte de l'environnement à partir des données du LiDAR et de l'odométrie.

  * `$ sudo apt-get install ros-jazzy-slam-toolbox`


* `robot_localization` : Fusion des données(EKF) de l'odométrie et de l'IMU pour une estimation d'état plus précise.

  * `$ sudo apt-get install ros-jazzy-robot-localization`

* `laser_filters` : Noeud Ros permet de filtrer, nettoyer les données brutes du Lidar avant leur traitement.

   * `$ sudo apt-get install ros-jazzy-laser-filters`

* `navigation2 et nav2_bringup` : Utiliser pour la navigation  autonome sous ROS2, comprenant les algorithmes, d'évitement d'obstacle, les planificateurs locaux/globaux et le système de localisation d'AMCL etc.

   * `$ sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`

* `nav2_map_server` : Noeud de Nav2 responsable du chargement des cartes , de leur publication pour AMCL et de la sauvegarde des nouvelles cartes.

   * `$ sudo apt-get install ros-jazzy-nav2-map-server`



## Tâche

 Pour repondre au cahier de charge, notre solution developper sur l'envronnement ROS2 JAZZY s'apppuis sur plusieur critere.

 Pour se repérer, le robot utilise l'algorithme AMCL, qui croise les données de son LiDAR (filtrées pour retirer le corps du robot) avec l'odométrie, elle-même fusionnée avec l'IMU par un filtre de Kalman étendu.

 Une fois la lacolisation du robot établie,le cerveau du système est un script Python (`task_solution.py`) qui pilote le robot avec `nav2_simple_commander`. 
 
 Ce programme commence par lire automatiquement le point de départ et l'objectif final à partir du fichier Yaml `task_params`, ensuite nous appliquons une formule mathématique de, afin d'effectuer un `changement de rèpere` des coordonnées de l'objectif defini dans le `monde gazebo` vers le repere de la `carte Map`.

 Ces nouveaux coordonnées de l'objectif converti est envoyé à Nav2, qui calcule un chemin sûr et évite les obstacles en temps réel.
 
 Enfin une boucle de notre code surveille le déplacement du robot en continu pour estimer son temps d'arrivée en respectant le temps defini dans le cahier de cahier et recalculer une nouvelle trajectoire s'il se retrouve bloquer.


 Pour lancer notre solution:

 * ` ros2 run nav_solution task_solution`



## Défis rencontrés

* **Le LiDar qui capte les roues du robot**: 
Le capteur lisait les roues de notre propre robot et croyait qu'il y avait un obstacle collé à lui. On a utilisé le package `laser_filters` pour nettoyer les données, ce qui nous a demandé de bien remapper nos topics  pour obliger Nav2 à lire le nouveau topic `/scan_filtered` au lieu des données brutes qui provient du `/scan`. 

* **Marcage des zones interdit au robot**:
Comme notre LiDAR balaie au ras du sol, il ne détectait pas le plateau des tables du restaurant, et le robot et le robot se rapprochais trop de la table . Pour corriger ça, on a mis en place un `keepout_filter`. On a marqué sur la  carte des zones interdites et configuré les serveurs `filter_mask_server` et `costmap_filter_info_server` pour forcer le robot à les contourner.

* **Confusion entre repère Gazebo et repère map** : 
Les coordonnées de `task_params.yaml` sont dans le monde Gazebo, pas dans le repère `map`. Comme map coïncide avec la position de spawn pendant la cartographie, la pose initiale d'`AMCL` est (0, 0) (nous l'avons vérifié avec /odom), et l'objectif est converti avec changement de repère enfant( une translation et une rotation).

* **Dérive de l'odomérie des roues***:
L'odométrie des roues dérivait, nous donc l'avons fusionnée avec l'IMU grâce à `EKF` et`robot_localization`

* **Le robot qui recalculait sa trajectoir en permanence**:
Au début, on n'avait pas défini d'arbre de comportement (`Behavior Tree`) spécifique dans nos paramètres. Nav2 chargeait donc son arbre par défaut, ce qui poussait le robot à recalculer sa trajectoire en boucle pour rien. Pour corriger ça on a changé la valeur de `default_nav_to_pose_bt_xml` pour utiliser le fichier `navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml`. Grâce à cette modification, le robot garde sa trajectoir et ne cherche un nouveau chemin que si l'ancien devient vraiment invalide.