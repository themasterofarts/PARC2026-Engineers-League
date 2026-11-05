import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    #### Mapping en direct dans l'arene simulee : la carte se construit pendant
    #    que Nav2 deplace le robot. Aucune carte n'est chargee au depart.
    #    Les buts se donnent dans RViz avec "Nav2 Goal".
    package_name = "nav_solution"
    nav_solution_dir = get_package_share_directory(package_name)
    bringup_dir = get_package_share_directory("parc_robot_bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params = os.path.join(nav_solution_dir, "config", "nav2_params_mapping.yaml")
    slam_params = os.path.join(nav_solution_dir, "config", "slam_toolbox_params.yaml")
    # Vue Nav2 complete avec Nav2, vue legere (carte, scan, camera) sinon
    rviz_nav2 = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")
    rviz_mapping = os.path.join(nav_solution_dir, "config", "mapping.rviz")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true", description="Fenetre Gazebo"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("apriltags", default_value="true", description="Detection des tags"),
        DeclareLaunchArgument("nav2", default_value="true", description="false : mapping seul, a la teleop"),
        DeclareLaunchArgument("world", default_value="arena_world.sdf"),
        DeclareLaunchArgument("x", default_value="0.2", description="Depart du robot dans l'arene"),
        DeclareLaunchArgument("y", default_value="-1.4"),
        DeclareLaunchArgument("yaw", default_value="1.5708"),
        DeclareLaunchArgument("masque", default_value="", description="Masque keepout (generer_masque)"),
        DeclareLaunchArgument("carte", default_value="",
                              description="Carte enregistree (YAML) : AMCL au lieu de slam_toolbox"),
    ]
    with_nav2 = IfCondition(LaunchConfiguration("nav2"))
    sans_carte = IfCondition(PythonExpression(["'", LaunchConfiguration("carte"), "' == ''"]))
    avec_carte = IfCondition(PythonExpression(["'", LaunchConfiguration("carte"), "' != ''"]))

    # Arene, robot et ponts ROS <-> Gazebo
    arena = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "arena.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "world": LaunchConfiguration("world"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )

    # EKF odometrie + IMU : publie la TF odom -> base_footprint utilisee par slam_toolbox.
    # L'odometrie seule du plugin Gazebo surestime les rotations de 31 %.
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(nav_solution_dir, "config", "ekf_arena.yaml"), {"use_sim_time": use_sim_time}],
    )

    # Filtre du LiDAR (enleve le corps du robot du scan)
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter",
        output="screen",
        parameters=[
            os.path.join(nav_solution_dir, "config", "lasers_scan_filters_arena.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Carte construite en direct : publie /map et la TF map -> odom
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")),
        launch_arguments={"use_sim_time": use_sim_time, "slam_params_file": slam_params}.items(),
        condition=sans_carte,
    )

    # Ou carte enregistree : map_server + AMCL
    localisation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "localisation.launch.py")),
        launch_arguments={
            "carte": LaunchConfiguration("carte"),
            "amcl_params": os.path.join(nav_solution_dir, "config", "amcl_arena.yaml"),
            "use_sim_time": use_sim_time,
        }.items(),
        condition=avec_carte,
    )

    # Masque des plateaux et assises (keepout), si donne
    masque = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "masque.launch.py")),
        launch_arguments={"masque": LaunchConfiguration("masque"), "use_sim_time": use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("masque"), "' != ''"])),
    )

    # Nav2 sans AMCL ni map_server : la localisation vient de slam_toolbox
    nav2 = IncludeLaunchDescription(
        # copie du launch officiel de Nav2, avec un delai de 10 s pour les noeuds figes
        PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "navigation_ma64.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
        condition=with_nav2,
    )

    apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_solution_dir, "launch", "apriltag.launch.py")),
        # taille etalonnee comme sur le vrai robot : la camera simulee sous-estimait les
        # distances d'un facteur 1.063 (16 visites, verite Gazebo) ; 0.165 x 1.063 = 0.1754
        launch_arguments={"use_sim_time": use_sim_time, "tag_size": "0.1754"}.items(),
        condition=IfCondition(LaunchConfiguration("apriltags")),
    )

    def rviz_node(config, condition):
        return Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=condition,
        )

    show_rviz = LaunchConfiguration("rviz")
    rviz_with_nav2 = rviz_node(
        rviz_nav2, IfCondition(PythonExpression(["'", show_rviz, "' == 'true' and '", LaunchConfiguration("nav2"), "' == 'true'"]))
    )
    rviz_without_nav2 = rviz_node(
        rviz_mapping, IfCondition(PythonExpression(["'", show_rviz, "' == 'true' and '", LaunchConfiguration("nav2"), "' != 'true'"]))
    )

    # Simulation isolee sur cette machine : connecte au Wi-Fi du robot, le PC serait sinon
    # visible par le robot et les autres equipes (domaine 0) avec ses /tf, /clock et /parc/.
    isoler = SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    # Gazebo aussi : sa communication interne passe par le multicast du reseau ; un changement
    # de Wi-Fi la coupait (simulation figee, course 20). 127.0.0.1 : local et insensible au reseau.
    isoler_gz = SetEnvironmentVariable("GZ_IP", "127.0.0.1")

    return LaunchDescription(
        [isoler, isoler_gz]
        + declare_args
        + [
            arena,
            # on laisse Gazebo demarrer et publier /clock avant le reste
            TimerAction(period=8.0, actions=[ekf, laser_filter, slam, localisation, masque, apriltag, rviz_with_nav2, rviz_without_nav2]),
            TimerAction(period=14.0, actions=[nav2]),
        ]
    )
