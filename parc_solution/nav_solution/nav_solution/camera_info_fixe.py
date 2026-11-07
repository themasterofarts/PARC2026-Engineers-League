#!/usr/bin/env python3
##### camera_info de secours pour le detecteur de tags
#    apriltag_ros ne traite une image que si un camera_info au MEME stamp exact l'accompagne,
#    et ne calcule la pose que si ce camera_info est etalonne. Si le camera_info du robot
#    manque, est a zero ou n'a pas les memes stamps, aucune detection utilisable ne sort.
#    Ce noeud republie, pour chaque image, un camera_info au stamp de l'image, avec
#    l'etalonnage de camera2 lu sur le rosbag du robot du 05/11 (640x480, sans distorsion).
#    Si la taille de l'image change, l'etalonnage est mis a l'echelle (et c'est ecrit au journal).
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CameraInfoFixe(Node):
    def __init__(self):
        super().__init__('camera_info_fixe')
        p = self.declare_parameter
        self.image_topic = p('image_topic', '/camera2/camera2/color/image_raw').value
        # image_transport cherche le camera_info a cote de l'image (meme dossier) : on republie
        # donc l'image ET son camera_info sous sortie/ (le detecteur lit sortie/image_raw)
        self.sortie = p('sortie', '/ma64/camera2').value
        self.info_topic = self.sortie + '/camera_info'
        self.w0 = p('width', 640).value
        self.h0 = p('height', 480).value
        self.fx = p('fx', 612.33).value
        self.fy = p('fy', 612.05).value
        self.cx = p('cx', 318.24).value
        self.cy = p('cy', 229.67).value
        self.pub = self.create_publisher(CameraInfo, self.info_topic, 10)
        self.compresse = p('compresse', False).value
        self.pub_img = self.create_publisher(CompressedImage if self.compresse else Image,
                                             self.sortie + '/image_raw' + ('/compressed' if self.compresse else ''), 10)
        if self.compresse:   # image compressee : pas de taille dans le message, on garde width/height
            self.create_subscription(CompressedImage, self.image_topic + '/compressed', self.on_image, qos_profile_sensor_data)
        else:
            self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.taille = None
        self.n = 0
        self.create_timer(10.0, self.bilan)
        self.get_logger().info(f'{self.image_topic} -> {self.info_topic} (fx {self.fx}, fy {self.fy}, '
                               f'cx {self.cx}, cy {self.cy} pour {self.w0}x{self.h0})')

    def on_image(self, img):
        w, h = (self.w0, self.h0) if self.compresse else (img.width, img.height)
        sx, sy = w / self.w0, h / self.h0
        if self.taille != (w, h):
            self.taille = (w, h)
            if (sx, sy) != (1.0, 1.0):
                self.get_logger().warn(f'image {w}x{h} : etalonnage mis a l\'echelle ({sx:.3f}, {sy:.3f})')
        fx, fy, cx, cy = self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy
        m = CameraInfo()
        m.header = img.header
        m.width, m.height = w, h
        m.distortion_model = 'plumb_bob'
        m.d = [0.0] * 5
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub.publish(m)
        self.pub_img.publish(img)
        self.n += 1

    def bilan(self):
        if self.n == 0:
            self.get_logger().warn(f'aucune image recue sur {self.image_topic} depuis 10 s')
        self.n = 0


def main():
    rclpy.init()
    n = CameraInfoFixe()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
