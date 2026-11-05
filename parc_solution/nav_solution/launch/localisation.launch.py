from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    #### Localisation sur une carte enregistree (remplace slam_toolbox) :
    #    map_server publie /map, AMCL publie la TF map -> odom.
    #    La pose de depart (x0, y0, yaw0) est dans le repere de la carte ; 0, 0, 0 =
    #    meme endroit et meme direction qu'au debut de la cartographie.
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    declare_args = [
        DeclareLaunchArgument("carte", description="Fichier YAML de la carte (map_saver_cli)"),
        DeclareLaunchArgument("amcl_params", description="Parametres AMCL"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("x0", default_value="0.0"),
        DeclareLaunchArgument("y0", default_value="0.0"),
        DeclareLaunchArgument("yaw0", default_value="0.0"),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("carte"), "use_sim_time": use_sim_time}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            LaunchConfiguration("amcl_params"),
            {
                "use_sim_time": use_sim_time,
                "initial_pose.x": ParameterValue(LaunchConfiguration("x0"), value_type=float),
                "initial_pose.y": ParameterValue(LaunchConfiguration("y0"), value_type=float),
                "initial_pose.yaw": ParameterValue(LaunchConfiguration("yaw0"), value_type=float),
            },
        ],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localisation",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": ["map_server", "amcl"]}],
    )

    return LaunchDescription(declare_args + [map_server, amcl, lifecycle])
