import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = "nav_solution"
    
    # Configuration des arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    package_name = "nav_solution"
    
    #prametre yaml pour la fusion de capteur
    ekf_params = os.path.join(
        get_package_share_directory(package_name), "config", "ekf_node.yaml"
    )
    ## filtre de mon laser
    #laser_filter_params = os.path.join(get_package_share_directory(pkg_path), "config", "lasers_scan_filters.yaml")
    
    slam_params_file = LaunchConfiguration(
        "slam_params_file",
        default=os.path.join(get_package_share_directory(pkg_path), "config", "slam_toolbox_params.yaml"),
    )
    
    robot_localisation_node = Node(
        package = "robot_localization",
        executable="ekf_node",
        name ="ekf_filter_node",
        output="screen",
        parameters=[ekf_params,{"use_sim_time":use_sim_time}]
    )
    
    # Noud pour filtrer le LiDAR (enleve le corps du robot du scan)
    # laser_filter_node = Node(
    #     package="laser_filters",
    #     executable="scan_to_scan_filter_chain",
    #     name="laser_filter",
    #     output="screen",
    #     parameters=[laser_filter_params, {"use_sim_time": use_sim_time}],
    # )
    
    # Declaration de l'argument use_sim_time
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",       
        description="Utiliser le temps de simulation"
    )
    
    # Recupration du dossier de slam_toolbox
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # 
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        robot_localisation_node,
        #laser_filter_node,
        #slam_toolbox_launch,
        
    ])