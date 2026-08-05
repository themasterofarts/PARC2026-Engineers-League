import os

import yaml

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="parc_robot_bringup").find("parc_robot_bringup")
    pkg_description = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")

    bridge_params = os.path.join(pkg_path, "config/gz_bridge.yaml")
    ekf_params_file = os.path.join(pkg_path, "config/ekf.yaml")
    rviz_config_file = os.path.join(pkg_path, "rviz/task1.rviz")
    goal_location_sdf = os.path.join(pkg_path, "models/goal_location/model.sdf")
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_path, "models")
    )

    # Launch configuration variables
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_robot_localization_cmd = DeclareLaunchArgument(
        name="use_robot_localization",
        default_value="False",
        description="Use robot_localization package if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value="world1",
        description="World model to load",
        choices=["world1", "world2", "world3"],
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher.launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Function to launch Gazebo and the SITO-E robot dependent on the world file chosen
    def spawn_gazebo_entities(context):

        nonlocal world, goal_location_sdf

        # List of actions to be added to the launch description later
        actions = []

        # Set path to world parameter yaml file
        params_file = os.path.join(
            pkg_path,
            "config/",
            context.launch_configurations["world"] + "_params.yaml",
        )

        # Set path to world file
        world_file = os.path.join(
            pkg_path, "worlds", context.launch_configurations["world"] + ".sdf"
        )

        # Open world specific yaml file
        if os.path.exists(params_file):
            with open(params_file, "r") as f:
                params = yaml.safe_load(f)

                spawn_x_val = str(params["/**"]["ros__parameters"]["x"])
                spawn_y_val = str(params["/**"]["ros__parameters"]["y"])
                spawn_z_val = str(params["/**"]["ros__parameters"]["z"])
                spawn_yaw_val = str(params["/**"]["ros__parameters"]["yaw"])
                goal_x_val = str(params["/**"]["ros__parameters"]["goal_x"])
                goal_y_val = str(params["/**"]["ros__parameters"]["goal_y"])
                goal_z_val = str(params["/**"]["ros__parameters"]["goal_z"])

                # Launch configuration variables for world params file and world path
                world_params_file = LaunchConfiguration("world_params_file")
                world_path = LaunchConfiguration("world_path_file")

                # Declare world params file launch argument
                actions.append(
                    DeclareLaunchArgument(
                        name="world_params_file",
                        default_value=params_file,
                        description="Full path to the parameter file of the respective loaded world",
                    )
                )

                # Declare world path file launch argument
                actions.append(
                    DeclareLaunchArgument(
                        name="world_path_file",
                        default_value=world_file,
                        description="Full path to the world model to load",
                    )
                )

                # Load world parameters file
                actions.append(
                    Node(
                        package="parc_robot_bringup",
                        executable="load_task_params.py",
                        parameters=[world_params_file],
                    )
                )

                # Launch Gazebo
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")]
                        ),
                        launch_arguments={
                            "gz_args": ["-r -v4 ", world_path],
                            "on_exit_shutdown": "true",
                        }.items(),
                    )
                )

                # Spawn SITO-E robot in Gazebo
                actions.append(
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        output="screen",
                        arguments=[
                            "-topic",
                            "robot_description",
                            "-name",
                            "sitoe_robot",
                            "-x",
                            spawn_x_val,
                            "-y",
                            spawn_y_val,
                            "-z",
                            spawn_z_val,
                            "-Y",
                            spawn_yaw_val,
                        ],
                    )
                )

                # Spawn goal location
                actions.append(
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        output="screen",
                        arguments=[
                            "-file",
                            goal_location_sdf,
                            "-name",
                            "goal_location",
                            "-x",
                            goal_x_val,
                            "-y",
                            goal_y_val,
                            "-z",
                            goal_z_val,
                        ],
                    )
                )

        return actions

    # Start Gazebo ROS bridge
    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # Start Gazebo ROS Top Camera Color Image bridge
    start_gazebo_ros_top_camera_color_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/top_camera_color/image_raw"],
        output="screen",
    )

    # Start Gazebo ROS Bottom Camera Color Image bridge
    start_gazebo_ros_bottom_camera_color_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/bottom_camera_color/image_raw"],
        output="screen",
    )
    # Start Gazebo ROS Top Camera Depth Image bridge
    # start_gazebo_ros_top_depth_image_bridge_cmd = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/top_camera_depth_to_color/image_raw"],
    #     output="screen",
    # )

    # Start Gazebo ROS Bottom Camera Depth Image bridge
    # start_gazebo_ros_bottom_depth_image_bridge_cmd = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/bottom_camera_depth_to_color/image_raw"],
    #     output="screen",
    # )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Start teleop node
    start_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, "launch", "teleop.launch.py")]
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_localization_cmd)
    ld.add_action(set_env_vars_resources)

    # Add any actions
    # ld.add_action(start_rviz_cmd)
    # ld.add_action(start_teleop_cmd)
    ld.add_action(OpaqueFunction(function=spawn_gazebo_entities))
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_top_camera_color_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_bottom_camera_color_image_bridge_cmd)
    # ld.add_action(start_gazebo_ros_top_depth_image_bridge_cmd)
    # ld.add_action(start_gazebo_ros_bottom_depth_image_bridge_cmd)

    return ld
