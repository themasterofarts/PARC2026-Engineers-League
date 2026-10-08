import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "nav_solution"  
    
    ## pkg robot_brinup et rviz pour la navigation##
    pkg_robo_brinp_name = "parc_robot_bringup"
    
    mask_yaml_path = os.path.join(
        get_package_share_directory(package_name), "maps_zone_interdi", "carte_restaurant_tables_zone_keepout.yaml"
    )    
    

    # Fichier de parametres Nav2
    nav2_params = os.path.join(
        get_package_share_directory(package_name), "config", "nav2_params.yaml"
    )
    #Chemin vers le fichier yaml de keepout_filtres
    keepout_params_file = os.path.join(
        get_package_share_directory(package_name), "config", "keepout_params.yaml"
    )
    
    static_map_path = os.path.join(
        get_package_share_directory(package_name), "maps", "carte_restaurant_tables.yaml"
    )

    # Pile Nav2 officielle
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    map_yaml_file = LaunchConfiguration("map")

    

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="params_file",
                default_value=nav2_params,
                description="full path to the ROS2 parameters file to use for all launched nodes",
            ),
            
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(
                    bringup_dir, "rviz", "nav2_default_view.rviz"
                ),
                description="Full path to the RVIZ config file to use",
            ),
            ###Rviz de parc_robot pars defaut a la place de celui de nav2
            # DeclareLaunchArgument(
            #     "rviz_config_file",
            #     default_value=os.path.join(
            #         get_package_share_directory(pkg_robo_brinp_name), "rviz", "task.rviz"
            #     ),
            #     description="Full path to the RVIZ config file to use",
            # ),
            
            # pour la carte map
            DeclareLaunchArgument(
                name="map",
                default_value=static_map_path,
                description="Full path to map file to load",
            ),
            
            DeclareLaunchArgument(
                name="autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            # 
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": LaunchConfiguration("autostart"),
                    #"slam": "True",  #  active SLAM Toolbox automatiquement
                }.items(),
            ),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rviz_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "rviz_config": rviz_config_file,
                }.items(),
            ),
            
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output='screen',
                parameters=[keepout_params_file,{'yaml_filename': mask_yaml_path}]
            ),
            
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                parameters=[keepout_params_file]
            ),
            
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_filters',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}
                ]
            ),
        ]
    )