from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    #### Masque de zones interdites (comme nav2.launch.py en phase 1) :
    #    filter_mask_server publie le masque (generer_masque) sur /keepout_filter_mask,
    #    costmap_filter_info_server annonce ce masque au keepout_filter des costmaps.
    #    Sans ce launch, le keepout_filter des costmaps attend et ne bloque rien.
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    declare_args = [
        DeclareLaunchArgument("masque", description="YAML du masque keepout (generer_masque)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    filter_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="filter_mask_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": LaunchConfiguration("masque"),
            "topic_name": "/keepout_filter_mask",
            "frame_id": "map",
        }],
    )

    filter_info_server = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "type": 0,
            "filter_info_topic": "/costmap_filter_info",
            "mask_topic": "/keepout_filter_mask",
            "base": 0.0,
            "multiplier": 1.0,
        }],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_masque",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                     "node_names": ["filter_mask_server", "costmap_filter_info_server"]}],
    )

    return LaunchDescription(declare_args + [filter_mask_server, filter_info_server, lifecycle])
