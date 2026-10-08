# EQUIPE 1: MA64 ROBOTICS

## Introduction

acompleter 

**Pays de l'équipe :** 
* BENIN

**Noms des membres de l'équipe:**

*  KPOKPO Sunday (chef d'équipe)
* EGOUDJOBI Peace Fiacre



## Dépendances

**Les packages nécessaires sont :**

* `nav2_simple_commander`: API Python pour la pile Nav2, permettant d'envoyer facilement des objectifs de navigation et de récupérer le statut de la mission.

  * `$ sudo apt-get install ros-jazzy-nav2-simple-commander`

* `slam_toolbox` : Package utiliser pour générer des cartes dynamiques de l'environnement à partir des données du Laser et de l'odometrie.

  * `$ sudo apt-get install ros-jazzy-slam-toolbox`


* `robot_localization` : Package permet la fusion données(EKF) des capteurs(IMU,odometrie) afin d'obtenir une estimation d'état précis

  * `$ sudo apt-get install ros-jazzy-robot-localization`

* `laser_filters` : Noeud Ros permet de filtrer, nettoyer les données brutes du Lidar avant leur traitement.

   * `$ sudo apt-get install ros-jazzy-laser-filters`

* `navigation2 et nav2_bringup` : Utiliser pour la navigation  autonome sous ROS2, comprenant les algorithmes, d'évitement d'
                                  d'obstacle, les planificateurs locaux/globaux et le système de localisation d'AMCL

   * `$ sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`

* `nav2_map_server` : Noeud du sysème Nav2 responsable du chargement des cartes , de leur publication pour AMCL 
                      et de la sauvegarde des nouvelles cartes.

   * `$ sudo apt-get install ros-jazzy-nav2-map-server`



## Tâche

 Pour repondre au cahier de charge, notre solution developper sur l'envronnement ROS2 JAZZY s'apppuis sur plusieur critere.

 Ainsi afin que le robot se repere dans son environnement il utilise l'algorithme AMCL qui croise les données de son capteur lidar et celui de l'odometrie.

 Une fois la lacolisation du robot etablie, le cerveau de notre systeme repose sur un scripte python pour diriger le robot a l'aide de l'outil `nav2_simple_commander`. 
 
 Ce programme commence par lire automatiquement le point de départ et l'objectif final à partir du fichier Yaml `task_params`, ensuite nous appliquons une formule mathématique de, afin d'effectuer un `changement de rèpere` des coordonnées de l'objectif defini dans le `monde gazebo` vers le repere de la `carte Map`.

 Ces nouveaux coordonnées de l'objectif est ensuite envoyée à `Nav2`, qui va calculer un chemin sécurisé en evitant les obstcales en temps réel.
 
 Enfin un boucle de notre code surveille le déplacement du robot en continu pour estimer son temps d'arrivée en respectant le temps defini dans le cahier de cahier et recalculer une nouvelle trajectoire s'il se retrouve bloquer.


 Pour lancer notre solution:

 * ` ros2 run nav_solution task_solution`



## Défis rencontrés


