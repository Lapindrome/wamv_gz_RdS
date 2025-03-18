#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")

        # Création du Subscriber sur le topic image
        self.subscription = self.create_subscription(
            Image,
            "/wamv/sensors/camera/front/image",  # Topic ROS 2
            self.image_callback,
            10,
        )  # Taille du buffer

        self.bridge = CvBridge()
        self.get_logger().info("Camera Subscriber Node has started!")

    def image_callback(self, msg):
        """Callback pour convertir et afficher l'image reçue"""
        try:
            # Convertir le message ROS Image en image OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Affichage de l'image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Nécessaire pour la mise à jour de l'affichage

        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image : {e}")


def main(args=None):
    rclpy.init(args=args)

    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    
    rclpy.shutdown()
    cv2.destroyAllWindows()  # Fermer la fenêtre OpenCV proprement


if __name__ == "__main__":
    main()
