#!/usr/bin/env python3
# File: puzzlebot_line_follower/line_light_detector.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineLightDetector(Node):
    def __init__(self):
        super().__init__('line_light_detector')

        # Parámetros dinámicos (ajusta en tu YAML)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('roi_y_start', 0.4)
        self.declare_parameter('trap_top_ratio', 0.9)
        self.declare_parameter('x_crop_ratio', 0.2)
        self.declare_parameter('morph_kernel_size', 3)
        self.declare_parameter('morph_iterations', 2)
        self.declare_parameter('use_clahe', True)

        image_topic = self.get_parameter('image_topic').value

        # Publicadores
        self.pub_error = self.create_publisher(Float32, '/line_detector/error', 10)
        self.pub_debug = self.create_publisher(Image, '/line_detector/debug_image', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Suscripción
        self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'[{self.get_name()}] Suscrito a {image_topic}')

    def image_callback(self, msg: Image):
        # 1) Convertir Image → BGR
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        # 2) Mascara trapezoidal
        y0 = int(self.get_parameter('roi_y_start').value * h)
        top_w = int(self.get_parameter('trap_top_ratio').value * w)
        x_crop = int(self.get_parameter('x_crop_ratio').value * w)
        pts = np.array([[
            (x_crop, h),
            (w - x_crop, h),
            ((w - top_w)//2, y0),
            ((w + top_w)//2, y0)
        ]], dtype=np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, pts, 255)
        roi = cv2.bitwise_and(frame, frame, mask=mask)

        # 3) Gris + blur + CLAHE
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        if self.get_parameter('use_clahe').value:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            gray = clahe.apply(gray)

        # 4) Binarización Otsu
        _, binary = cv2.threshold(gray, 0, 255,
                                  cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 5) Morfología Close
        k = np.ones((self.get_parameter('morph_kernel_size').value,)*2, np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k,
                                  iterations=self.get_parameter('morph_iterations').value)

        # 6) Contornos
        contours, _ = cv2.findContours(binary,
                                       cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.pub_error.publish(Float32(data=0.0))
            return

        # 7) Contorno mayor y centroide
        cnt = max(contours, key=cv2.contourArea)
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            self.pub_error.publish(Float32(data=0.0))
            return
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        # 8) Error lateral normalizado
        x_center = w // 2
        error_px = x_center - cx
        error_norm = float(error_px) / float(x_center)
        self.pub_error.publish(Float32(data=error_norm))

        # 9) Imagen de depuración
        debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(debug, [cnt], -1, (0,255,0), 2)
        cv2.circle(debug, (cx, cy), 5, (0,0,255), -1)
        cv2.line(debug, (x_center, 0), (x_center, h), (255,0,0), 2)
        overlay = debug.copy()
        cv2.polylines(overlay, pts, True, (255,255,0), 2)
        debug = cv2.addWeighted(overlay, 0.6, debug, 0.4, 0)
        debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
        self.pub_debug.publish(debug_msg)

    def destroy_node(self):
        self.get_logger().info(f'[{self.get_name()}] Cerrando nodo')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LineLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
