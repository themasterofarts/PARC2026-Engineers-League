#! /usr/bin/env python3

##### Script pour la navigation autonome. 
#    il gère la localisation du robot par AMCL, la lecture des paramètre des disfferent coordonner du fichier Task_params.yaml
##    Puis effectue un changement de repere des coordonner(gaol_x, gaol_y) de l'objectif( repere gazebo vers le repere map)

import time
import rclpy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import yaml
import os
import math

from ament_index_python.packages import get_package_share_directory

def main():
    
    rclpy.init()
    
    ### lecture des parametres du fichier yaml
    pkg_name ='parc_robot_bringup'
    task_file = os.path.join(get_package_share_directory(pkg_name),'config','task_params.yaml')
    
    if os.path.exists(task_file):
        with open(task_file, 'r') as f:
            params = yaml.safe_load(f)
             
            # Récupération des coordonnées absolues depuis Gazebo
            spawn_x_val = params["/**"]["ros__parameters"]["x"]
            spawn_y_val = params["/**"]["ros__parameters"]["y"]
            spawn_yaw_val = params["/**"]["ros__parameters"]["yaw"]
            
            goal_x_val = params["/**"]["ros__parameters"]["goal_x"]
            goal_y_val = params["/**"]["ros__parameters"]["goal_y"]
    else:
       raise FileNotFoundError(f"Le fichier {task_file} n'existe pas!.")

    navigator = BasicNavigator()
    
    ### lire le fichier yaml pour definir la position du robot la pose du gaold   
    
    
    ### initialisation de la pose du depart #############
    
    #### pour information la possition initiale du robot dans le repere map ne correspond pas exactement a celui qui se trouve sue le fichier 
    # car en creant la carte le robot se trouver a la pose (0,0) dans le repre map""""
    
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0  #spawn_x_val
    initial_pose.pose.position.y = 0.0 #spawn_y_val
    ####covertion en quaterniion
    initial_pose.pose.orientation.z = 0.0 #math.sin(spawn_yaw_val/2)
    initial_pose.pose.orientation.w = 1.0 #math.cos(spawn_yaw_val/2)
    
    ####initilialiser la pose du robot
    navigator.setInitialPose(initial_pose)
    
    ### attendre que la pile de navigation soit lancer et que la loc du robot soit faite ####
    navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')
    
    print("Attente qu'AMCL soit stable ")
    time.sleep(3.0)
    
    # les coordonné de l'objectif fourni par le YAML est dans le repère global Gazebo.
    # On a appliquer un un chnagement de repere pour l'exprimer
    # dans le repère local de notre carte SLAM.
    
    ## Calcul du vecteur de translation 
    dx = goal_x_val - spawn_x_val
    dy = goal_y_val - spawn_y_val
    
    #Calcul de l'angle de correction (Rotation inverse au spawn)
    theta = -spawn_yaw_val
    
    # application de la matrice de rotation 
    goal_x_map = dx * math.cos(theta) - dy * math.sin(theta)
    goal_y_map = dx * math.sin(theta) + dy * math.cos(theta)
    
    # définition de l'objectif converti
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    
    goal_pose.pose.position.x = float(goal_x_map)
    goal_pose.pose.position.y = float(goal_y_map)
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    
    
    #### Navigation autonome
    
    print(f"envoi de l'objectif calculé : x={goal_x_map:.2f}, y={goal_y_map:.2f}")
    navigator.goToPose(goal_pose)
    
    heure_depart = navigator.get_clock().now()
    i = 0
    relance_goal = False
    
    #  boucle de surveillance de la navigation
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        temps_ecoule = navigator.get_clock().now() - heure_depart
        i += 1
        
        # Affichage l'etat
        if feedback and i % 10 == 0:
            estimate_time_of_arrive = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f"Estimation du temps restant : {estimate_time_of_arrive:.0f} secondes")
    
    #### Resultat de la tache navigation ###
    result = navigator.getResult()
        
    if result == TaskResult.SUCCEEDED:
        print("SUCCÈS : Le robot a atteint la zone cible !")
    elif result == TaskResult.CANCELED:
        print("ANNULÉ : La navigation a été annulée.")
    elif result == TaskResult.FAILED:
        print("ÉCHEC : Le planificateur n'a pas pu trouver de chemin valide.")
    else:
        print("Erreur : Statut de retour invalide.")

    # Fermeture propre du nœud
    rclpy.shutdown() 
    exit(0)

if __name__ == '__main__':
    main()