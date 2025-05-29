#!/usr/bin/env python3
# File: puzzlebot_line_follower/yolov8_detector.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import cv2

from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference

class YoloInference(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO('/home/brunene/ros2_ws/src/puzzlebot_line_follower/puzzlebot_line_follower/lastfinal.pt')
        self.img = np.zeros((480, 720, 3), dtype=np.uint8)
        self.bridge = CvBridge()

        # Suscriptor e publishers
        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        self.yolo_pub = self.create_publisher(Yolov8Inference, '/Yolov8_inference', 10)
        self.yolo_img_pub = self.create_publisher(Image, '/inference_result', 10)
        self.semaphore_pub = self.create_publisher(Int8, '/semaphore_state', 10)

        self.prev_class = None
        timer_period = 1.0/3  # 👈 frecuencia reducida para alivianar carga
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'[{self.get_name()}] Nodo YOLO listo, publicando inferencias y estado de semáforo')

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.img = cv2.rotate(cv_img, cv2.ROTATE_180)

        except CvBridgeError as e:
            self.get_logger().warn(f'[{self.get_name()}] Falló conversión de imagen: {e}')

    def timer_callback(self):
        # 1) Inferencia
        results = self.model(self.img)

        # 2) Detección de clase semáforo (tomamos la primera caja)
        new_class = None
        for r in results:
            for box in r.boxes:
                new_class = int(box.cls)
                break
            if new_class is not None:
                break

        # 3) Publicar clase del semáforo si cambia
        if new_class is not None and new_class != self.prev_class:
            if new_class == 2:
                self.get_logger().info(f'[{self.get_name()}] 🟢 Verde detectado')
            elif new_class == 0:
                self.get_logger().info(f'[{self.get_name()}] 🟡 Amarillo detectado')
            elif new_class == 1:
                self.get_logger().info(f'[{self.get_name()}] 🔴 Rojo detectado')
            else:
                self.get_logger().warn(f'[{self.get_name()}] Clase semáforo desconocida: {new_class}')

            state_msg = Int8()
            state_msg.data = new_class
            self.semaphore_pub.publish(state_msg)
            self.prev_class = new_class

        # 4) Crear y publicar mensaje Yolov8Inference (ahora es local, no global)
        yolo_msg = Yolov8Inference()
        yolo_msg.header.frame_id = 'inference'
        yolo_msg.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                inf = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = int(box.cls)
                inf.class_name = self.model.names[c]
                inf.top = int(b[0])
                inf.left = int(b[1])
                inf.bottom = int(b[2])
                inf.right = int(b[3])
                yolo_msg.yolov8_inference.append(inf)

        self.yolo_pub.publish(yolo_msg)

        # 5) (Opcional) Mostrar resultados — comenta esto si no necesitas imagen con boxes
        frame = results[0].plot()
        frame_uint8 = cv2.convertScaleAbs(frame)
        img_msg = self.bridge.cv2_to_imgmsg(frame_uint8, encoding='bgr8')
        self.yolo_img_pub.publish(img_msg)

    def destroy_node(self):
        self.get_logger().info(f'[{self.get_name()}] Nodo YOLO detenido')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloInference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

