import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    #Récupérer les chemins des dossiers  des packages
    parc_bringup_dir = get_package_share_directory('parc_robot_bringup')
    
    nav_solution  = get_package_share_directory('nav_solution')
    
    
    
    
    #### lancement de la simution du robot dans gazebo
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(parc_bringup_dir, 'launch', 'task.launch.py')
        )
    )
    
    #### se fichier launch lance unique le ekf et node robot localization
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_solution, 'launch', 'slam_ekf.launch.py')
        )
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_solution,'launch','nav2.launch.py')
        )
    )
    
    return LaunchDescription([
        #### on lance d'abord le robot sit et gazebo
        bringup_launch,
        
        ### on patiente quelque seconde puis on lance slam_launch
        TimerAction(
            period=10.0,
            actions=[slam_launch]
        ),
        
        ### on patiente quelque seconde puis on lance nav2_launch
        TimerAction(
            period=10.0,
            actions=[nav2_launch]
        )
        
    ])