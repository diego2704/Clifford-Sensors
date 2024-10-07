#!/usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(Image, 'camera_frames', self.listener_callback, 10)
        self.bridge = CvBridge()
        
        # Cargar el modelo YOLOv5
        self.model = YOLO('yolov5nu.pt')

    def listener_callback(self, msg):
        # Convertir el mensaje de ROS a un frame de OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Realizar la detección de objetos
        results = self.model(frame)
        
        # Dibuja las cajas detectadas
        annotated_frame = results[0].plot()
        
        # Muestra el frame procesado
        cv2.imshow("Video (procesado)", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
