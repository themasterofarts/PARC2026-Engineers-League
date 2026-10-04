# #! /usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2

# from geometry_msgs.msg import Twist




# class DetecObstacle(Node):
    
#     def __init__(self):
#         super().__init__('object_detec')
        
#         self.get_logger().info('Lacement du noeud')
        
#         #### 'abonner au nuage des point de la camera du Haut
        
#         self.sub_camera = self.create_subscription(
#             PointCloud2,
#             "top_camera_depth/points",
#             self.camera_callback,
#             10
#         )
        
        
#         #### publier a nav2  une commande pour que le robot ralentie si l'obstacle es trop proche 
#         ### et on quige le robot manulement jusqu'a s'eloigner de l'obstacle
        
#         # self.pub = self.create_publisher(
            
#         # )
        
        
        
#         ###### initialisation du message pour la detection
        
#         self.