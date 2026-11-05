import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #### Arene de la phase reelle (5 m x 5 m, tables, AprilTags 1, 4, 7, 9).
    #    Meme robot que task.launch.py, sans le but de la phase 1.
    #    La TF odom -> base_footprint doit venir d'un EKF (voir nav_solution/mapping.launch.py).

    pkg_path = FindPackageShare(package="parc_robot_bringup").find("parc_robot_bringup")
    pkg_description = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")

    # Sans la TF odom -> base_footprint du plugin : l'EKF la publie (odometrie + IMU)
    bridge_params = os.path.join(pkg_path, "config/gz_bridge_arena.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # false : serveur seul, sans fenetre (la simulation va plus vite)
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value="arena_world.sdf",
                              description="arena_world.sdf (aleatoire) ou arena_finale_world.sdf (photos du 4 nov)"),
        DeclareLaunchArgument("x", default_value="0.2"),
        DeclareLaunchArgument("y", default_value="-1.4"),
        DeclareLaunchArgument("yaw", default_value="1.5708"),
    ]

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_path, "models")
    )

    def spawn_gazebo_entities(context):
        gui = LaunchConfiguration("gui").perform(context).lower() == "true"
        gz_args = "-r " if gui else "-s -r --headless-rendering "
        world_file = os.path.join(pkg_path, "worlds", LaunchConfiguration("world").perform(context))

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")]
                ),
                launch_arguments={
                    "gz_args": [gz_args, world_file],
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic", "robot_description",
                    "-name", "sitoe_robot",
                    "-x", LaunchConfiguration("x").perform(context),
                    "-y", LaunchConfiguration("y").perform(context),
                    "-z", "0.02",
                    "-Y", LaunchConfiguration("yaw").perform(context),
                ],
            ),
        ]

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher.launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        output="screen",
    )

    start_top_camera_bridge_cmd = Node(
        package="ros_gz_image",
        namespace="top_camera",
        executable="image_bridge",
        arguments=["/top_camera_color/image_raw"],
        output="screen",
    )

    start_bottom_camera_bridge_cmd = Node(
        package="ros_gz_image",
        namespace="bottom_camera",
        executable="image_bridge",
        arguments=["/bottom_camera_color/image_raw"],
        output="screen",
    )

    # Simulation isolee sur cette machine : connecte au Wi-Fi du robot, le PC serait sinon
    # visible par le robot et les autres equipes (domaine 0) avec ses /tf, /clock et /parc/.
    isoler = SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    # Gazebo aussi : sa communication interne passe par le multicast du reseau ; un changement
    # de Wi-Fi la coupait (simulation figee, course 20). 127.0.0.1 : local et insensible au reseau.
    isoler_gz = SetEnvironmentVariable("GZ_IP", "127.0.0.1")

    ld = LaunchDescription([isoler, isoler_gz] + declare_args)
    ld.add_action(set_env_vars_resources)
    ld.add_action(OpaqueFunction(function=spawn_gazebo_entities))
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_top_camera_bridge_cmd)
    ld.add_action(start_bottom_camera_bridge_cmd)
    return ld
