import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from origincar_msg.msg import Sign  # 正确导入 Sign 消息
import cv2
import numpy as np

# 假设 origincar_msg/msg/Sign 消息定义如下：
# int32 sign_data  # 这是 Sign 消息中的字段

class LineDetector:
    def __init__(
        self,
        linear_speed=1.15,          # 默认线速度(m/s)
        proportional_gain=0.007,     # 比例控制增益
        roi_ratio=0.6,              # 感兴趣区域高度比例(从图像底部开始)
        threshold=80,               # 二值化阈值
        center_threshold=90,        # 中心区域阈值(未使用)
        min_contour_area=100        # 最小轮廓面积阈值
    ):
        self.linear_speed = linear_speed
        self.proportional_gain = proportional_gain
        
        self.roi_ratio = roi_ratio
        self.threshold = threshold
        self.center_threshold = center_threshold
        self.min_contour_area = min_contour_area

    def process_image(self, cv_image):
        """处理图像并返回控制信息"""
        # 获取图像尺寸并设置ROI区域(只处理图像下方部分)
        height, width = cv_image.shape[:2]
        roi = cv_image[int(height * self.roi_ratio):, :]
        roi_height, roi_width = roi.shape[:2]
        center_x, center_y = roi_width // 2, roi_height // 2  # 图像中心点

        # 图像预处理：灰度化、高斯模糊、二值化
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, self.threshold, 255, cv2.THRESH_BINARY_INV)
        
        # 查找轮廓
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 初始化控制信息
        control_info = {
            'error': 0,             # 中心线偏差
            'angular_z': 0.0,       # 角速度控制量，确保是浮动数字
            'contour_area': 0        # 轮廓面积
        }

        # 寻找距离中心点最近的轮廓
        closest_contour = None
        min_distance = float('inf')  # 初始设为无穷大

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:  # 忽略太小的轮廓
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:  # 防止除以零
                continue

            # 计算轮廓中心点
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            distance = np.hypot(cx - center_x, cy - center_y)  # 计算与图像中心的距离

            if distance < min_distance:  # 更新最近轮廓
                min_distance = distance
                closest_contour = cnt

        if closest_contour is not None:
            # 计算最近轮廓的矩和中心点
            M = cv2.moments(closest_contour)
            cx = int(M['m10'] / M['m00'])
            error = cx - center_x  # 计算中心偏差
            contour_area = cv2.contourArea(closest_contour)  # 计算轮廓面积

            # 更新控制信息
            control_info.update({
                'error': error,
                'angular_z': float(-error * self.proportional_gain),  # 强制转换为 float 类型
                'contour_area': contour_area
            })

        return control_info


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')  # 初始化ROS节点

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_hz', 20),                   # 控制频率(Hz)
                ('min_contour_area', 100),           # 最小轮廓面积
                ('default_linear_speed', 1.15),       # 默认线速度
                ('default_proportional_gain', 0.05),   # 默认比例增益
            ]
        )
        # 初始化线检测器
        self.detector = LineDetector(
            linear_speed=self.get_parameter('default_linear_speed').value,
            proportional_gain=self.get_parameter('default_proportional_gain').value,
            min_contour_area=self.get_parameter('min_contour_area').value
        )

        # 创建订阅者
        self.sub_image = self.create_subscription(
            CompressedImage,
            '/image',                    # 压缩图像话题
            self.image_callback,         # 图像回调函数
            qos_profile=10
        )
        self.sub_sign4return = self.create_subscription(
            Int32,
            '/sign4return',              # 返回标志话题
            self.sign4return_callback,   # 标志回调函数
            qos_profile=10
        )
        self.sub_sign_switch = self.create_subscription(
            Sign,       # 使用自定义的 Sign 消息类型
            '/sign_switch',               # 开关标志话题
            self.sign_switch_callback,    # 新增：开关标志回调
            qos_profile=10
        )
        # 新增：订阅origincar_competition的控制权转移信号
        self.sub_control_handover = self.create_subscription(
            Int32,
            '/control_handover',          # origincar_competition发布的控制权转移话题
            self.control_handover_callback,  # 控制权转移回调函数
            qos_profile=10
        )
        # 创建发布者
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)  # 速度控制话题

        # 初始化变量
        self.active = False  # 是否激活线跟踪
        self.cv_has_control = True  # 新增：CV系统是否拥有控制权
        self.last_control_time = self.get_clock().now()  # 上次控制时间
        self.control_hz = self.get_parameter('control_hz').value  # 控制频率

        self.get_logger().info("巡线初始化完成")

    def sign4return_callback(self, msg):
        """处理返回标志消息"""
        if msg.data == 6:
            # 标准跟踪模式
            self.set_detector_params(0.8, 0.007, "标准跟踪模式")
        elif msg.data == 5:
            # 紧急停止
            self.stop_robot()
        elif msg.data == 10:
            # 精确调整模式(更低速)
            self.set_detector_params(1.15, 0.007, "精确调整模式")

    def sign_switch_callback(self, msg):
        """处理开关标志消息(新增)"""
        if msg.sign_data == 3 or msg.sign_data == 4:
            # 当收到3或4时停止机器人
            self.stop_robot()

    def control_handover_callback(self, msg):
        """处理origincar_competition发布的控制权转移消息"""
        self.cv_has_control = (msg.data == 0)  # 0表示CV控制，1表示competition控制
        if not self.cv_has_control:
            self.get_logger().info("收到控制权转移信号：competition接管控制权，暂停line_follower执行")
        else:
            self.get_logger().info("收到控制权转移信号：CV系统接管控制权，恢复line_follower执行")

    def set_detector_params(self, speed, gain, mode_desc):
        """设置检测器参数并激活"""
        self.detector.linear_speed = speed
        self.detector.proportional_gain = gain
        self.active = True  # 激活线跟踪
        self.get_logger().info(
            f"{mode_desc} 已激活 - 速度: {speed}m/s, 增益: {gain}"
        )

    def stop_robot(self):
        """停止机器人运动"""
        twist = Twist()  # 创建空速度消息
        self.pub_cmd_vel.publish(twist)  # 发布零速度
        self.active = False  # 失活线跟踪
        self.get_logger().info("紧急停止已激活")

    def image_callback(self, msg):
        """处理图像消息"""
        if not self.active:  # 如果不活跃则直接返回
            return
        
        # 新增：检查CV系统是否拥有控制权
        if not self.cv_has_control:
            return  # 如果CV系统没有控制权，则暂停执行

        # 控制频率限制
        current_time = self.get_clock().now()
        if (current_time - self.last_control_time).nanoseconds < 1e9 / self.control_hz:
            return
        self.last_control_time = current_time

        try:
            # 将ROS图像消息转换为OpenCV格式
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                raise ValueError("无效的图像数据")

            # 处理图像并获取控制信息
            control_info = self.detector.process_image(cv_image)

            # 创建并发布速度控制消息
            twist = Twist()
            twist.linear.x = self.detector.linear_speed  # 设置线速度
            twist.angular.z = control_info['angular_z']  # 设置角速度

            self.pub_cmd_vel.publish(twist)  # 发布速度指令

        except Exception as e:
            self.get_logger().warn(f"图像处理失败: {e}")

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS
    node = LineFollower()  # 创建节点
    try:
        rclpy.spin(node)  # 运行节点
    except KeyboardInterrupt:
        pass  # 处理Ctrl+C
    finally:
        node.destroy_node()  # 销毁节点
        rclpy.shutdown()  # 关闭ROS

if __name__ == '__main__':
    main()
