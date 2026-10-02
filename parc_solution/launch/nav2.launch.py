"""Start static-map localization and Nav2 for the PARC simulation.

This is deliberately a test launch: it starts the navigation stack and a
velocity relay, while the operator supplies the initial pose and goal in RViz.
The final task_solution.py coordinator will automate those two actions.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    solution_share = get_package_share_directory("parc_solution")
    nav2_share = get_package_share_directory("nav2_bringup")

    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Start from Nav2's Jazzy defaults, then apply only robot-specific values.
    # base_footprint is essential: base_link is rotated by +90 degrees in the
    # supplied robot URDF, while odometry and lidar_link are aligned with
    # base_footprint.
    params_file = RewrittenYaml(
        source_file=os.path.join(nav2_share, "params", "nav2_params.yaml"),
        param_rewrites={
            "robot_base_frame": "base_footprint",
            "robot_radius": "0.30",
            "inflation_radius": "0.45",
            "amcl.ros__parameters.laser_max_range": "12.0",
            "amcl.ros__parameters.laser_min_range": "0.05",
            "amcl.ros__parameters.scan_topic": "/scan_filtered",
            "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic": "/scan_filtered",
            "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": "/scan_filtered",
            "collision_monitor.ros__parameters.scan.topic": "/scan_filtered",
            "controller_server.ros__parameters.FollowPath.vx_max": "0.30",
            "controller_server.ros__parameters.FollowPath.vx_min": "-0.15",
            "controller_server.ros__parameters.FollowPath.wz_max": "0.80",
            "behavior_server.ros__parameters.max_rotational_vel": "0.80",
        },
        convert_types=True,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "slam": "False",
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(
                    solution_share, "maps", "restaurant_map.yaml"
                ),
                description="Saved occupancy-grid map to load.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo's /clock.",
            ),
            nav2_launch,
            Node(
                package="parc_solution",
                executable="self_return_filter",
                name="self_return_filter",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="parc_solution",
                executable="cmd_vel_relay",
                name="cmd_vel_relay",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
