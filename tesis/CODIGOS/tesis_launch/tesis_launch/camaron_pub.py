from __future__ import print_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_frames', 10)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        # Establecer un temporizador para publicar a una frecuencia de 10 Hz
        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('No se pudo capturar el frame')
            return

        # Convertir el frame de OpenCV a un mensaje de ROS
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(ros_image)
        self.get_logger().info('Frame publicado')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
