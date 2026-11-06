import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    #### Course sur le vrai robot CAYTU Sito-E.
    #    Les pilotes du robot tournent deja (LiDAR, cameras, odometrie, TF odom -> base_link,
    #    obstacle_avoidance). On ajoute : filtre LiDAR, SLAM en direct, AprilTag, Nav2,
    #    noeud de mission. Depart par commande : /ma64/start (voir GUIDE_ROBOT_JEUDI.md).
    #    Depart vocal (Vosk, mots go / allez / vai) : vocal:=true [modele_vosk:=dossier]
    #    Pas d'EKF : botwheel_explorer publie deja la TF odom -> base_link, une seconde
    #    source pour la meme TF ferait sauter la position du robot.
    #    Cartographie seule (teleop) : mission:=false nav2:=false
    #    Masque des plateaux et assises : masque:=~/MA64/cartes/arene_keepout.yaml
    #    Course sur une carte faite pendant les essais : carte:=~/MA64/cartes/arene.yaml
    #    (AMCL au lieu de slam_toolbox ; depart au meme endroit qu'au debut de la cartographie,
    #    ou x0:= y0:= yaw0:=)
    #    Simulateur officiel PARC 2026 (memes topics que le robot) : sim:=true
    #    -> fichiers *_sim.yaml generes depuis ceux du robot (sim_outils/generer_config_sim.py) :
    #       base_footprint au lieu de base_link (le base_link simule est tourne de 90 deg),
    #       horloge simulee, commandes Twist sur /cmd_vel.
    package_name = "nav_solution"
    nav_solution_dir = get_package_share_directory(package_name)
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    camera = LaunchConfiguration("camera")
    camera_prefix = PythonExpression(["'/", camera, "/", camera, "/color'"])

    declare_args = [
        DeclareLaunchArgument("sim", default_value="false", description="true : simulateur officiel PARC 2026"),
        DeclareLaunchArgument("camera", default_value="camera2",
                              description="Camera pour les tags : camera1 ou camera2"),
        DeclareLaunchArgument("tag_size", default_value="0.165", description="Cote du carre noir des tags (m)"),
        DeclareLaunchArgument("nav2", default_value="true"),
        DeclareLaunchArgument("mission", default_value="true"),
        DeclareLaunchArgument("masque", default_value="",
                              description="Masque keepout des plateaux et assises (generer_masque)"),
        DeclareLaunchArgument("carte", default_value="",
                              description="Carte enregistree (YAML) : AMCL au lieu de slam_toolbox"),
        DeclareLaunchArgument("x0", default_value="0.0", description="Pose de depart dans la carte"),
        DeclareLaunchArgument("y0", default_value="0.0"),
        DeclareLaunchArgument("yaw0", default_value="0.0"),
        DeclareLaunchArgument("indices", default_value="",
                              description="Fichier YAML des tags notes a la cartographie (sauver_tags)"),
        DeclareLaunchArgument("slam_params", default_value="",
                              description="Parametres slam_toolbox (defaut : slam_toolbox_robot[_sim].yaml)"),
        DeclareLaunchArgument("vocal", default_value="false",
                              description="Depart vocal (Vosk, /speech_audio) en plus du depart par commande"),
        DeclareLaunchArgument("modele_vosk", default_value="~/MA64/vosk-model",
                              description="Dossier du modele Vosk"),
    ]

    def noeuds(context):
        sim = LaunchConfiguration("sim").perform(context).lower() == "true"
        use_sim_time = "true" if sim else "false"

        def config(nom):
            return os.path.join(nav_solution_dir, "config", f"{nom}_sim.yaml" if sim else f"{nom}.yaml")

        slam_params = LaunchConfiguration("slam_params").perform(context) or config("slam_toolbox_robot")
        carte = LaunchConfiguration("carte")

        # Vrai robot : LiDAR monte a l'envers (rosbag du 05/11), /scan -> /scan_corrige
        scan_lidar = "/scan" if sim else "/scan_corrige"
        retourne = Node(
            package="nav_solution",
            executable="scan_retourne",
            name="scan_retourne",
            output="screen",
            condition=IfCondition("false" if sim else "true"),
        )

        # /scan sans le chassis ni les roues, publie sous /scan_ma64
        laser_filter = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="laser_filter",
            output="screen",
            parameters=[config("lasers_scan_filters_robot"), {"use_sim_time": sim}],
            remappings=[("scan", scan_lidar), ("scan_filtered", "/scan_ma64")],
        )

        # slam_toolbox est active par un lifecycle_manager : l'activation par evenement du launch
        # officiel se perd parfois (vu en simulation : slam_toolbox reste inactive, pas de TF map).
        # Pas de bond : sa perte ferait arreter slam_toolbox en pleine course.
        sans_carte = IfCondition(PythonExpression(["'", carte, "' == ''"]))
        slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")),
            launch_arguments={"use_sim_time": use_sim_time, "slam_params_file": slam_params,
                              "use_lifecycle_manager": "true"}.items(),
            condition=sans_carte,
        )
        lifecycle_slam = Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_slam",
            output="screen",
            parameters=[{"use_sim_time": sim, "autostart": True, "node_names": ["slam_toolbox"],
                         "bond_timeout": 0.0}],
            condition=sans_carte,
        )

        localisation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "localisation.launch.py")),
            launch_arguments={
                "carte": carte,
                "amcl_params": config("amcl_robot"),
                "use_sim_time": use_sim_time,
                "x0": LaunchConfiguration("x0"),
                "y0": LaunchConfiguration("y0"),
                "yaw0": LaunchConfiguration("yaw0"),
            }.items(),
            condition=IfCondition(PythonExpression(["'", carte, "' != ''"])),
        )

        apriltag = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "apriltag.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "camera_image": PathJoinSubstitution([camera_prefix, "image_raw"]),
                "camera_info": PathJoinSubstitution([camera_prefix, "camera_info"]),
                "tag_size": LaunchConfiguration("tag_size"),
            }.items(),
        )

        masque = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "masque.launch.py")),
            launch_arguments={"masque": LaunchConfiguration("masque"), "use_sim_time": use_sim_time}.items(),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("masque"), "' != ''"])),
        )

        nav2 = IncludeLaunchDescription(
            # copie du launch officiel de Nav2, avec un delai de 10 s pour les noeuds figes
            PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "navigation_ma64.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": config("nav2_params_robot"),
                "autostart": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("nav2")),
        )

        mission = Node(
            package=package_name,
            executable="mission_node",
            name="mission",
            output="screen",
            parameters=[
                config("mission_robot"),
                {"image_topic": PathJoinSubstitution([camera_prefix, "image_raw"])},
                {"indices_tags": LaunchConfiguration("indices")},
                {"scan_topic": scan_lidar},
                {"use_sim_time": sim},
            ],
            condition=IfCondition(LaunchConfiguration("mission")),
        )

        vocal = Node(
            package=package_name,
            executable="demarrage_vocal",
            name="demarrage_vocal",
            output="screen",
            parameters=[{"model_path": LaunchConfiguration("modele_vosk"), "use_sim_time": sim}],
            condition=IfCondition(LaunchConfiguration("vocal")),
        )

        return [
            retourne,
            vocal,
            laser_filter,
            slam,
            lifecycle_slam,
            localisation,
            masque,
            apriltag,
            # Nav2 attend la TF map -> odom (slam_toolbox ou AMCL)
            TimerAction(period=5.0, actions=[nav2]),
            TimerAction(period=12.0, actions=[mission]),
        ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=noeuds)])
