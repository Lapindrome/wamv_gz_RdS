#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO


class YOLOImageSubscriber(Node):
    def __init__(self):
        super().__init__("yolo_image_subscriber")

        # Load YOLOv8 model (replace with your model path if needed)
        self.model = YOLO("yolo11n.pt")  # Pretrained YOLO11 model

        # ROS 2 Subscription to Image Topic
        self.subscription = self.create_subscription(
            Image, "/wamv/sensors/camera/front/image", self.image_callback, 10
        )

        # OpenCV Bridge to Convert ROS Images
        self.bridge = CvBridge()

    def image_callback(self, msg):
        """Callback function to process images from the ROS topic."""
        try:
            # Convert ROS Image message to OpenCV format (NumPy array)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Run YOLO detection
            results = self.model.predict(cv_image, classes=[8], conf=0.10, max_det=1)

            # Draw bounding boxes on the image
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = box.conf[0]
                    label = self.model.names[int(box.cls[0])]

                    # Draw rectangle
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        cv_image,
                        f"{label} {confidence:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

            # Show the image with detections
            cv2.imshow("YOLO Detection", cv_image)
            cv2.waitKey(1)  # Keep OpenCV window responsive

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLOImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
