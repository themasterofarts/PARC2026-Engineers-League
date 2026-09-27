"""Trimmed-down Nav2 bringup for the PARC 2026 navigation task.

Based on nav2_bringup's navigation_launch.py, but only brings up the
servers this solution actually configures (config/nav2_params.yaml).
Jazzy's stock navigation_launch.py also brings up smoother_server,
route_server, collision_monitor, and docking_server, none of which are
needed for basic point-to-point navigation and some of which (collision
_monitor) crash without additional required parameters we don't set here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("parc_nav_solution")
    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={"autostart": "true"},
            convert_types=True,
        ),
        allow_substs=True,
    )

    load_nodes = GroupAction(
        [
            SetParameter("use_sim_time", True),
            # Publishes odom_imu -> odom (IMU-corrected heading); every Nav2
            # frame in nav2_params.yaml is odom_imu, so this must run first.
            Node(
                package="parc_nav_solution",
                executable="imu_odom_corrector",
                name="imu_odom_corrector",
                output="screen",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{"autostart": True, "node_names": lifecycle_nodes}],
            ),
        ]
    )

    # velocity_smoother's final output is the conventional 'cmd_vel' topic;
    # remap it straight to the robot's actual drive topic.
    remapped_navigation = GroupAction(
        [
            SetRemap(src="cmd_vel", dst="/robot_base_controller/cmd_vel_unstamped"),
            load_nodes,
        ]
    )

    return LaunchDescription(
        [SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"), remapped_navigation]
    )
