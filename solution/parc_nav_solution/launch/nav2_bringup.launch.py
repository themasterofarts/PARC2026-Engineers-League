"""Trimmed-down Nav2 bringup for the PARC 2026 navigation task.

Based on nav2_bringup's navigation_launch.py, but only brings up the
servers this solution actually configures (config/nav2_params.yaml).
Jazzy's stock navigation_launch.py also brings up smoother_server,
route_server, collision_monitor, and docking_server, none of which are
needed for basic point-to-point navigation and some of which (collision
_monitor) crash without additional required parameters we don't set here.
velocity_smoother is left out too: see the note above load_nodes.

mode:=map (default) navigates on our map (maps/cafe.yaml): it also starts
map_server and AMCL. mode:=mapping (tools/build_map.py) loads
config/nav2_params_mapping.yaml on top instead: no map yet, rolling costmaps in
odom_imu, while slam_toolbox builds the map.

use_camera:=true (task_solution.py --camera) also starts depth_obstacles and
loads config/nav2_params_camera.yaml on top, adding the top depth camera to
both costmaps; see that file for why it isn't the default.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context):
    pkg_share = get_package_share_directory("parc_nav_solution")
    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")
    use_camera = LaunchConfiguration("use_camera").perform(context).lower() == "true"
    mode = LaunchConfiguration("mode").perform(context).lower()
    if mode not in ("map", "mapping"):
        raise ValueError(f"mode must be 'map' or 'mapping', not {mode!r}")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
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
    # Overlays on top of nav2_params.yaml: a later params file overrides the
    # keys it repeats.
    params = [configured_params]
    if mode == "mapping":
        params.append(ParameterFile(os.path.join(pkg_share, "config", "nav2_params_mapping.yaml")))
    if use_camera:
        params.append(ParameterFile(os.path.join(pkg_share, "config", "nav2_params_camera.yaml")))

    localization_nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=params + [{"yaml_filename": os.path.join(pkg_share, "maps", "cafe.yaml")}],
            remappings=remappings,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=params,
            remappings=remappings,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}],
        ),
    ] if mode == "map" else []

    # controller_server and behavior_server publish straight to the robot's
    # drive topic; there is no velocity_smoother. (It used to be launched but
    # accidentally bypassed by a group-level cmd_vel remap. Wired in properly,
    # 4 of 16 runs clipped cafe_table_6's overhanging top, vs 0 of 15 without
    # it: its acceleration limiting makes the robot lag the controller and cut
    # that corner, which the floor-level LiDAR can't see.)
    drive = ("cmd_vel", "/robot_base_controller/cmd_vel_unstamped")
    camera_nodes = [
        # Thins the top depth camera's cloud for the costmaps' "depth"
        # observation source (what the floor-level LiDAR can't see).
        Node(
            package="parc_nav_solution",
            executable="depth_obstacles",
            name="depth_obstacles",
            output="screen",
        ),
    ] if use_camera else []

    load_nodes = GroupAction(
        [
            SetParameter("use_sim_time", True),
            *camera_nodes,
            *localization_nodes,
            # Publishes odom_imu -> odom (IMU-corrected heading), which AMCL
            # (map -> odom_imu) and the local costmap build on.
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
                parameters=params,
                remappings=remappings + [drive],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=params,
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=params,
                remappings=remappings + [drive],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=params + [{"default_nav_to_pose_bt_xml": os.path.join(
                    pkg_share, "behavior_trees", "navigate_to_pose_no_motion_recovery.xml")}],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=params,
                remappings=remappings,
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

    return [SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"), load_nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="map",
                              description="map: navigate on maps/cafe.yaml (AMCL); mapping: no map, for tools/build_map.py"),
        DeclareLaunchArgument("use_camera", default_value="false",
                              description="add the top depth camera to the costmaps (experimental)"),
        OpaqueFunction(function=launch_setup),
    ])
