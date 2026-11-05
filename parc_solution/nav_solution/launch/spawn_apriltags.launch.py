import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #### Ajoute les AprilTags de test dans le monde Gazebo deja lance.
    #    Le monde de la tache de navigation reste ainsi inchange.
    models_dir = os.path.join(get_package_share_directory("parc_robot_bringup"), "models")
    tags_file = os.path.join(
        get_package_share_directory("nav_solution"), "config", "apriltags_sim.yaml"
    )

    with open(tags_file, "r") as f:
        config = yaml.safe_load(f)

    tags = config["tags"]
    tag_z = config["floor_z"] + 1.0

    actions = []
    for tag in tags:
        name = f"apriltag_{tag['id']}"
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{name}",
                output="screen",
                arguments=[
                    "-file", os.path.join(models_dir, name, "model.sdf"),
                    "-name", name,
                    "-x", str(tag["x"]),
                    "-y", str(tag["y"]),
                    "-z", str(tag_z),
                    "-Y", str(tag["yaw"]),
                ],
            )
        )

    return LaunchDescription(actions)
