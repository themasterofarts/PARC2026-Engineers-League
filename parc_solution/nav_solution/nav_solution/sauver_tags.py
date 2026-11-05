#! /usr/bin/env python3

##### Sauvegarde la position des tags vus pendant la cartographie.
#    Lit /apriltag/tag_<id>/pose (repere map, publie par apriltag_tf_node) et ecrit
#    un fichier YAML que le noeud de mission peut charger (parametre indices_tags).
#    Les positions ne servent que d'indices : la visite se fait toujours sur la
#    detection camera en direct. Le repere map d'une nouvelle course ne coincide
#    avec celui de la cartographie que si le robot part du meme endroit, dans la
#    meme direction.
#    Utilisation : ros2 run nav_solution sauver_tags ~/MA64/cartes/tags.yaml

import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile


def normale_tag(q):
    """Axe z du tag (sort de la face imprimee), projete au sol. Meme calcul que mission_node,
    recopie pour ne pas dependre de parc_msgs pendant la cartographie."""
    nx = 2 * (q.x * q.z + q.w * q.y)
    ny = 2 * (q.y * q.z - q.w * q.x)
    n = (nx * nx + ny * ny) ** 0.5
    return (nx / n, ny / n) if n > 1e-6 else None


def main():
    sortie = sys.argv[1] if len(sys.argv) > 1 else 'tags.yaml'
    rclpy.init()
    node = rclpy.create_node('sauver_tags')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tags = {}

    def on_pose(i, msg):
        n = normale_tag(msg.pose.orientation)
        if n:
            p = msg.pose.position
            tags[i] = [round(p.x, 3), round(p.y, 3), round(n[0], 4), round(n[1], 4)]

    for i in (1, 4, 7, 9):
        node.create_subscription(PoseStamped, f'/apriltag/tag_{i}/pose', lambda m, i=i: on_pose(i, m), latched)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    with open(sortie, 'w') as f:
        yaml.safe_dump({'tags': tags}, f)
    print(f'{len(tags)} tag(s) sauves dans {sortie} : {sorted(tags)}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
