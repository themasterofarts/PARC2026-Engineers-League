"""Start the self-return filter and SLAM Toolbox for a clean mapping run."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    solution_share = get_package_share_directory("parc_solution")
    slam_share = get_package_share_directory("slam_toolbox")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": os.path.join(
                solution_share, "config", "slam_mapping.yaml"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo's /clock.",
            ),
            Node(
                package="parc_solution",
                executable="self_return_filter",
                name="self_return_filter",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            slam_launch,
        ]
    )
