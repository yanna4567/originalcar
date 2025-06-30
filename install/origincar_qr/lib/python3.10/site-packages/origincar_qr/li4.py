#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
# import pyzbar.pyzbar as pyzbar
import numpy as np

class QRCodeDetector:
    def __init__(self):
        self.depro = '/userdata/dev_ws/config/qrcode/detect.prototxt'
        self.decaf = '/userdata/dev_ws/config/qrcode/detect.caffemodel'
        self.srpro = '/userdata/dev_ws/config/qrcode/sr.prototxt' 
        self.srcaf = '/userdata/dev_ws/config/qrcode/sr.caffemodel'
        self.detector = cv2.wechat_qrcode_WeChatQRCode(self.depro, self.decaf, self.srpro, self.srcaf)

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 调整图像尺寸（微信模型需要最小输入尺寸）
        h, w = gray.shape
        min_size = 300
        if w < min_size or h < min_size:
            scale = min_size / min(w, h)
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        
        barcodes, points = self.detector.detectAndDecode(gray)
        for qrcode_info in barcodes:
            return qrcode_info
        return None

class WriterNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.last_data_msg = None
        self.last_twist_msg = None
        self.timer = None

        # 发布器
        self.pub_qr_result = self.create_publisher(String, "/sign", 10) #qr_result

        # 订阅器
        self.sub_image = self.create_subscription(CompressedImage, "/image", self.image_callback, 10)

        self.bridge = CvBridge()
        self.QRCode = QRCodeDetector()
        self.get_logger().info("start")

    def image_callback(self, msg):
        self.get_logger().info("Received image message")
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            return 

        content = self.QRCode.detect(cv_image)

        if content is None:
            self.get_logger().info("No QR Code detected.")
            return

        try:
            qr_data = content.strip()
            self.get_logger().info(f"QR Code : {qr_data}")
            msg = String()
            msg.data = qr_data                
            # 发布消息
            self.pub_qr_result.publish(msg)
        except ValueError:
            self.get_logger().info(f"Non-integer QR Code content: {qr_data}")

def main(args=None):
    rclpy.init(args=args)
    li4_node = WriterNode("li4")
    rclpy.spin(li4_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
