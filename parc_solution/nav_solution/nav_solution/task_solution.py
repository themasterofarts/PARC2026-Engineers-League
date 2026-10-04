#! /usr/bin/env python3

##### Script pour la navigation autonome.
#    Gere la localisation du robot par AMCL, la lecture des parametres
#    (x, y, yaw, goal_x, goal_y) du fichier task_params.yaml, le changement
#    de repere du but (repere Gazebo -> repere map), et le suivi de la
#    navigation avec un garde-fou de 600s (limite du concours) et une
#    relance unique en cas de blocage.

import time
import math
import os
import yaml

import rclpy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

Time_out_total = 600.0     # limite dure du concours (10 minutes)
Time_out_bloc_robot = 120.0   # au-dela, on considere que le robot est bloque
Attendre_amcl_stable = 3.0


def lire_task_params():
    """Lit x, y, yaw, goal_x, goal_y dans task_params.yaml. Leve une
    exception claire si le fichier est introuvable plutot que de planter
    plus loin avec un NameError incomprehensible."""
    pkg_name = 'parc_robot_bringup'
    task_file = os.path.join(
        get_package_share_directory(pkg_name), 'config', 'task_params.yaml'
    )
    if not os.path.exists(task_file):
        raise FileNotFoundError(f"Le fichier {task_file} n'existe pas !")

    with open(task_file, 'r') as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']

    return {
        'spawn_x': params['x'],
        'spawn_y': params['y'],
        'spawn_yaw': params['yaw'],
        'goal_x': params['goal_x'],
        'goal_y': params['goal_y'],
    }


def yaw_vers_quaternion(yaw):
    """Conversion simple pour un robot qui ne bouge que dans le plan
    (pas de roulis/tangage)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (z, w)


def convert_but_gazebo_vers_map(goal_x, goal_y, spawn_x, spawn_y, spawn_yaw):
    """Convertit le but, exprime dans le repere Gazebo, vers le
    repere map  """
    
    dx = goal_x - spawn_x
    dy = goal_y - spawn_y
    theta = -spawn_yaw
    goal_x_map = dx * math.cos(theta) - dy * math.sin(theta)
    goal_y_map = dx * math.sin(theta) + dy * math.cos(theta)
    
    return goal_x_map, goal_y_map


def main():
    rclpy.init()

    task_params = lire_task_params()
    navigator = BasicNavigator()

    ###### Pose initiale  ##############
    
    """ La pose intiale du robot  corespond a celui de la carte map qui ici (0,0,0)"""
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')

    print("Attendre q'amcl soit stable ..")
    time.sleep(Attendre_amcl_stable)

    ####### converti du repere Gazebo vers le repere map #####
    goal_x_map, goal_y_map = convert_but_gazebo_vers_map(
        task_params['goal_x'], task_params['goal_y'],
        task_params['spawn_x'], task_params['spawn_y'], task_params['spawn_yaw'],
    )

    def Goal_robot():
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(goal_x_map)
        pose.pose.position.y = float(goal_y_map)
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    print(f"Envoi de l'objectif converti : x={goal_x_map:.2f}, y={goal_y_map:.2f}")
    navigator.goToPose(Goal_robot())

    heure_depart = navigator.get_clock().now()
    i = 0
    relance_goal = False

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        temps_ecoule = navigator.get_clock().now() - heure_depart
        i += 1

        if feedback and i % 10 == 0:
            estimation = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f"Estimation du temps restant : {estimation:.0f} secondes")

       
        if temps_ecoule > Duration(seconds=Time_out_total):
            print("Time out atteint = 10min atteinte")
            navigator.cancelTask()
            break

        # Relance unique si le robot semble bloque .
        if (feedback and not relance_goal
                and Duration.from_msg(feedback.navigation_time) > Duration(seconds=Time_out_bloc_robot)):
            print("Blocage detecte, nouvelle tentative vers le meme gaol.")
            navigator.goToPose(Goal_robot())
            relance_goal = True

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        duree_totale = navigator.get_clock().now() - heure_depart
        print("SUCCES : le robot a atteint laa zone cible en vert !")
        print(f"Temps de navigation total : {duree_totale.nanoseconds / 1e9:.2f} secondes")
    elif result == TaskResult.CANCELED:
        print("ANNULE : la navigation a ete annulee .")
    elif result == TaskResult.FAILED:
        print("ECHEC : le planificateur n'a pas pu trouver de chemin valide.")
    else:
        print("Erreur : statut de retour invalide.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()