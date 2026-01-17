#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Bridge pour convertir ROS Image en OpenCV
        self.bridge = CvBridge()
        
        # Subscriber pour recevoir les images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher pour envoyer les commandes de mouvement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Line Follower Node démarré!')

    def image_callback(self, msg):
        try:
            # Convertir l'image ROS en OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Traiter l'image pour détecter la ligne
            processed = self.process_image(cv_image)
            
            # Calculer la commande de mouvement
            self.follow_line(processed)
            
            # Afficher l'image (optionnel, pour debug)
            cv2.imshow("Camera", cv_image)
            cv2.imshow("Processed", processed)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Erreur: {str(e)}')

    def process_image(self, image):
        # Convertir en niveaux de gris
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Appliquer un seuil pour détecter le noir
        # Tout ce qui est plus sombre que 50 devient blanc (255), le reste noir (0)
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        # On ne garde que la partie basse de l'image (où est la ligne)
        height = binary.shape[0]
        roi = binary[int(height * 0.6):height, :]  # 60% en bas de l'image
        
        return roi

    def follow_line(self, processed_image):
        # Créer le message de commande
        twist = Twist()
        
        # Calculer le centre de masse de la ligne (moment)
        M = cv2.moments(processed_image)
        
        if M['m00'] > 0:  # Si on détecte quelque chose
            # Calculer le centre X de la ligne
            cx = int(M['m10'] / M['m00'])
            
            # Centre de l'image
            image_center = processed_image.shape[1] // 2
            
            # Calculer l'erreur (différence entre centre ligne et centre image)
            error = cx - image_center
            
            # Commande de vitesse
            twist.linear.x = 0.1  # Avancer à 0.1 m/s
            twist.angular.z = -float(error) / 100.0  # Tourner proportionnellement à l'erreur
            
            self.get_logger().info(f'Ligne détectée à x={cx}, erreur={error}')
        else:
            # Pas de ligne détectée, on s'arrête
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('Aucune ligne détectée!')
        
        # Publier la commande
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
