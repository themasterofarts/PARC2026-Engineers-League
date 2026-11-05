#! /usr/bin/env python3

##### Noeud de suivi des AprilTags.
#    apriltag_node lit la camera et publie /detections et une TF
#    camera -> tag_<id>. Ce noeud recoit ces detections, exprime chaque tag
#    dans target_frame (map par defaut) et lisse les mesures, car la pose
#    brute saute d'une image a l'autre. Pour chaque tag, il publie :
#      - la TF fixe target_frame -> tag_<id>_stable
#      - un PoseStamped sur /apriltag/tag_<id>/pose
#    Ces deux sorties restent disponibles quand le tag sort de l'image.

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def _moyenne_quaternions(quaternions):
    """Moyenne de quaternions proches. q et -q sont la meme rotation : on les
    aligne sur le premier avant de les additionner."""
    q = np.array(quaternions)
    q[np.dot(q, q[0]) < 0.0] *= -1.0
    moyenne = q.mean(axis=0)
    return moyenne / np.linalg.norm(moyenne)


class ApriltagTfNode(Node):

    def __init__(self):
        super().__init__('apriltag_tf_node')

        detections_topic = self.declare_parameter('detections_topic', '/detections').value
        self.target_frame = self.declare_parameter('target_frame', 'map').value
        tag_ids = self.declare_parameter('tag_ids', [1, 4, 7, 9]).value
        self.prefix = self.declare_parameter('tag_frame_prefix', 'tag_').value
        self.min_margin = self.declare_parameter('min_decision_margin', 30.0).value
        window_size = self.declare_parameter('window_size', 15).value
        self.min_samples = self.declare_parameter('min_samples', 5).value
        self.max_tf_wait = Duration(seconds=self.declare_parameter('max_tf_wait', 1.0).value)
        # Un tag vu de loin ou tres en biais est mal place (pose AprilTag instable : jusqu'a 3 m
        # d'erreur sur le simulateur officiel). Ces mesures ne servent pas a placer le tag.
        self.max_distance = self.declare_parameter('max_distance', 3.0).value
        self.max_angle = math.radians(self.declare_parameter('max_view_angle_deg', 55.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # La TF du tag arrive apres /detections. Attendre dans le callback
        # bloquerait aussi la reception des TF : on met donc en file d'attente.
        self.en_attente = deque()

        # Transient local : un noeud qui demarre apres coup recoit la derniere pose
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mesures = {i: deque(maxlen=window_size) for i in tag_ids}
        self.pose_pubs = {
            i: self.create_publisher(PoseStamped, f'/apriltag/tag_{i}/pose', latched)
            for i in tag_ids
        }

        self.create_subscription(
            AprilTagDetectionArray, detections_topic, self._detections_callback, 10
        )
        self.create_timer(0.05, self._traite_en_attente)
        self.get_logger().info(f'Suivi des tags {tag_ids} dans le repere {self.target_frame}')

    def _detections_callback(self, msg):
        for detection in msg.detections:
            if detection.id in self.mesures and detection.decision_margin >= self.min_margin:
                self.en_attente.append((detection.id, Time.from_msg(msg.header.stamp), msg.header.frame_id))

    def _traite_en_attente(self):
        maintenant = self.get_clock().now()
        restants = deque()

        while self.en_attente:
            tag_id, stamp, camera = self.en_attente.popleft()
            tag_frame = f'{self.prefix}{tag_id}'
            try:
                vue = self.tf_buffer.lookup_transform(camera, tag_frame, stamp)
                transform = self.tf_buffer.lookup_transform(self.target_frame, tag_frame, stamp)
            except TransformException as erreur:
                if maintenant - stamp < self.max_tf_wait:
                    restants.append((tag_id, stamp, camera))
                else:
                    self.get_logger().warn(
                        f'TF {self.target_frame} -> {tag_frame} indisponible : {erreur}',
                        throttle_duration_sec=5.0,
                    )
                continue

            if not self._bien_vu(vue):
                continue
            self._ajoute_mesure(tag_id, transform)

        self.en_attente = restants

    def _bien_vu(self, vue):
        """Tag assez proche et assez de face pour une position fiable."""
        t, q = vue.transform.translation, vue.transform.rotation
        d = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
        if d < 1e-3 or d > self.max_distance:
            return False
        # axe z du tag (sort de la face) dans le repere camera, compare a la direction tag -> camera
        zx = 2 * (q.x * q.z + q.w * q.y)
        zy = 2 * (q.y * q.z - q.w * q.x)
        zz = 1 - 2 * (q.x ** 2 + q.y ** 2)
        cos_angle = -(zx * t.x + zy * t.y + zz * t.z) / d
        return cos_angle >= math.cos(self.max_angle)

    def _ajoute_mesure(self, tag_id, transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        mesures = self.mesures[tag_id]
        mesures.append(((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))

        if len(mesures) < self.min_samples:
            return
        if len(mesures) == self.min_samples:
            self.get_logger().info(f'Tag {tag_id} localise')

        # Mediane sur la position : une mesure aberrante n'a aucun effet
        position = np.median([m[0] for m in mesures], axis=0)
        orientation = _moyenne_quaternions([m[1] for m in mesures])
        self._publie(tag_id, position, orientation, transform.header.stamp)

    def _publie(self, tag_id, position, orientation, stamp):
        x, y, z = (float(v) for v in position)
        qx, qy, qz, qw = (float(v) for v in orientation)

        tf_stable = TransformStamped()
        tf_stable.header.stamp = stamp
        tf_stable.header.frame_id = self.target_frame
        tf_stable.child_frame_id = f'{self.prefix}{tag_id}_stable'
        tf_stable.transform.translation.x = x
        tf_stable.transform.translation.y = y
        tf_stable.transform.translation.z = z
        tf_stable.transform.rotation.x = qx
        tf_stable.transform.rotation.y = qy
        tf_stable.transform.rotation.z = qz
        tf_stable.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_stable)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = tf_stable.transform.rotation
        self.pose_pubs[tag_id].publish(pose)


def main():
    rclpy.init()
    node = ApriltagTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
