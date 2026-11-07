import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("nav_solution"), "config", "apriltag.yaml"
    )

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    # Lit la camera, publie /detections et une TF camera -> tag_<id>
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "size": ParameterValue(LaunchConfiguration("tag_size"), value_type=float),
                "image_transport": LaunchConfiguration("transport"),
            },
        ],
        remappings=[
            ("image_rect", LaunchConfiguration("camera_image")),
            ("camera_info", LaunchConfiguration("camera_info")),
        ],
    )

    # Stabilise les TF recues et donne la position de chaque tag dans la carte
    apriltag_tf_node = Node(
        package="nav_solution",
        executable="apriltag_tf_node",
        name="apriltag_tf_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "target_frame": LaunchConfiguration("target_frame"),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("transport", default_value="raw",
                              description="raw ou compressed (image compressee : robot du 07/11, images brutes perdues)"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Utiliser le temps de simulation"
        ),
        DeclareLaunchArgument(
            "camera_image",
            default_value="/top_camera_color/image_raw",
            description="Topic image de la camera du haut",
        ),
        DeclareLaunchArgument(
            "camera_info",
            default_value="/top_camera_color/camera_info",
            description="Topic camera_info de la camera du haut",
        ),
        DeclareLaunchArgument(
            "target_frame", default_value="map", description="Repere de sortie des tags"
        ),
        DeclareLaunchArgument(
            "tag_size", default_value="0.165", description="Cote du carre noir des tags (m)"
        ),
        apriltag_node,
        apriltag_tf_node,
    ])
