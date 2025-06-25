import os
import tos
from volcenginesdkarkruntime import Ark
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage
import tempfile
import numpy as np
import cv2

class ImageUploadAnalyzer(Node):
    def __init__(self):
        """初始化图像上传分析节点"""
        super().__init__('image_upload_analyzer')
        
        # 声明节点参数
        self.declare_parameter('ak', 'AKLTNzdmNTliYzJjNjI1NGUzMDk0NjYzZThiNTM3MTRjMjc')
        self.declare_parameter('sk', 'TjJOaFpEaGhPVGcyTWpnek5HRmtZVGsxWlRWbU5EUTBaVGM1TjJZM1lUSQ==')
        self.declare_parameter('endpoint', 'tos-cn-beijing.volces.com')
        self.declare_parameter('region', 'cn-beijing')
        self.declare_parameter('bucket_name', 'asdd')
        self.declare_parameter('object_key', 'target.jpg')
        self.declare_parameter('ark_api_key', '50ae2215-0fc3-406c-9a82-7f5036997a30')
        
        # 创建发布者
        self.result_publisher = self.create_publisher(String, 'image_analysis_result', 10)
        
        # 订阅压缩图像话题
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image',
            self.image_callback,
            10)
        
        # 订阅触发信号话题
        self.sign_sub = self.create_subscription(
            Int32,
            '/sign4return',
            self.sign_callback,
            10)
        
        # 状态变量
        self.trigger_analysis = False
        self.latest_image_data = None
        self.get_logger().info("图像上传分析节点已初始化完成")

    def sign_callback(self, msg):
        """触发信号回调函数"""
        if msg.data == 100:
            self.get_logger().info("收到触发信号(100)，准备分析下一张图像")
            self.trigger_analysis = True

    def image_callback(self, msg):
        """图像数据回调函数"""
        self.latest_image_data = msg.data
        if self.trigger_analysis and self.latest_image_data is not None:
            self.trigger_analysis = False
            self.get_logger().info("开始处理图像...")
            self.process_image(self.latest_image_data)
            self.latest_image_data = None  # 清空已处理图像

    def process_image(self, compressed_data):
        """处理图像数据"""
        try:
            # 将压缩图像数据转换为numpy数组
            np_arr = np.frombuffer(compressed_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is None:
                self.get_logger().error("图像解码失败")
                return
                
            # 创建临时文件
            with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp_file:
                temp_path = tmp_file.name
                success = cv2.imwrite(temp_path, img)
                if not success:
                    self.get_logger().error("保存临时图像文件失败")
                    return
                    
                self.get_logger().info(f"临时图像已保存至: {temp_path}")
                self.upload_and_analyze(temp_path)
                
        except Exception as e:
            self.get_logger().error(f"图像处理过程中发生错误: {str(e)}")

    def upload_and_analyze(self, image_path):
        """上传图像并进行分析"""
        # 获取配置参数
        ak = self.get_parameter('ak').value
        sk = self.get_parameter('sk').value
        endpoint = self.get_parameter('endpoint').value
        region = self.get_parameter('region').value
        bucket_name = self.get_parameter('bucket_name').value
        object_key = self.get_parameter('object_key').value
        ark_api_key = self.get_parameter('ark_api_key').value
        
        try:
            # 初始化TOS客户端
            client = tos.TosClientV2(ak, sk, endpoint, region)
            
            # 上传到TOS
            self.get_logger().info("正在上传图像到TOS...")
            client.put_object_from_file(bucket_name, object_key, image_path)
            self.get_logger().info("图像上传成功")
            
            # 获取图片URL
            image_url = f"https://{bucket_name}.{endpoint}/{object_key}"
            
            # 使用ARK分析
            self.get_logger().info("正在调用模型分析图像数据...")
            client_ark = Ark(api_key=ark_api_key)
            resp = client_ark.chat.completions.create(
                model="doubao-1-5-vision-pro-32k-250115",
                messages=[{
                    "content": [
                        {"text": "描述图中卡片上的内容", "type": "text"},
                        {"image_url": {"url": image_url}, "type": "image_url"}
                    ],
                    "role": "user"
                }]
            )
            
            # 发布结果
            result = resp.choices[0].message.content
            msg = String()
            msg.data = f"图像分析结果：{result}"
            self.result_publisher.publish(msg)
            self.get_logger().info(f"分析完成: {result}")
            
        except Exception as e:
            self.get_logger().error(f"上传或分析过程中发生错误: {str(e)}")
        finally:
            # 删除临时文件
            try:
                os.unlink(image_path)
                self.get_logger().info("临时文件已清理")
            except Exception as e:
                self.get_logger().warning(f"清理临时文件失败: {str(e)}")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    try:
        node = ImageUploadAnalyzer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("节点已关闭")

if __name__ == '__main__':
    main()