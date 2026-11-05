import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #### Course complete en simulation : arene, SLAM en direct, Nav2, AprilTag,
    #    faux arbitre (avec verite terrain Gazebo) et noeud de mission.
    #    Le faux arbitre arme la course apres 25 s puis envoie le depart.
    #    Necessite parc_msgs : source ~/ma64_parc_msgs_ws/install/setup.bash
    #    monde:=arena (premiere arene, 4 tables) ou monde:=finale (photos du 4 nov).
    nav_solution_dir = get_package_share_directory("nav_solution")
    use_sim_time = {"use_sim_time": True}
    # monde -> (fichier du monde, depart x y yaw, parametres mission + verite des tags)
    mondes = {
        "arena": ("arena_world.sdf", ("0.2", "-1.4", "1.5708"), "mission_sim.yaml"),
        "finale": ("arena_finale_world.sdf", ("-1.2", "-1.9", "1.5708"), "mission_sim_finale.yaml"),
    }

    declare_args = [
        DeclareLaunchArgument("monde", default_value="arena", description="arena ou finale"),
        DeclareLaunchArgument("gui", default_value="false", description="Fenetre Gazebo"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("ordre", default_value="[0]", description="Ordre des tags, [0] = aleatoire"),
        DeclareLaunchArgument("rapport", default_value="", description="Fichier JSON du faux arbitre"),
        DeclareLaunchArgument("carte", default_value="", description="Carte enregistree (YAML), sinon SLAM en direct"),
        DeclareLaunchArgument("masque", default_value="", description="Masque keepout (generer_masque)"),
        DeclareLaunchArgument("indices", default_value="",
                              description="Fichier YAML des tags notes a la cartographie (sauver_tags)"),
    ]

    def course(context):
        world, (x, y, yaw), fichier = mondes[LaunchConfiguration("monde").perform(context)]
        params = os.path.join(nav_solution_dir, "config", fichier)
        mapping = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "mapping.launch.py")),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "rviz": LaunchConfiguration("rviz"),
                "nav2": "true",
                "apriltags": "true",
                "carte": LaunchConfiguration("carte"),
                "masque": LaunchConfiguration("masque"),
                "world": world,
                "x": x, "y": y, "yaw": yaw,
            }.items(),
        )

        arbitre = Node(
            package="nav_solution",
            executable="faux_arbitre",
            name="faux_arbitre",
            output="screen",
            parameters=[params, use_sim_time, {"rapport": LaunchConfiguration("rapport")},
                        {"tag_order": LaunchConfiguration("ordre")}],
        )

        mission = Node(
            package="nav_solution",
            executable="mission_node",
            name="mission",
            output="screen",
            parameters=[params, use_sim_time, {"indices_tags": LaunchConfiguration("indices")}],
        )
        return [mapping, TimerAction(period=18.0, actions=[mission, arbitre])]

    # Simulation isolee sur cette machine : connecte au Wi-Fi du robot, le PC serait sinon
    # visible par le robot et les autres equipes (domaine 0) avec ses /tf, /clock et /parc/.
    isoler = SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    # Gazebo aussi : sa communication interne passe par le multicast du reseau ; un changement
    # de Wi-Fi la coupait (simulation figee, course 20). 127.0.0.1 : local et insensible au reseau.
    isoler_gz = SetEnvironmentVariable("GZ_IP", "127.0.0.1")

    return LaunchDescription([isoler, isoler_gz] + declare_args + [OpaqueFunction(function=course)])
