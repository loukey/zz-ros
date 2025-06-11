#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import numpy as np

class GeminiImageSubscriber(Node):
    def __init__(self):
        super().__init__('gemini_image_subscriber')
        # 创建 CvBridge 对象
        self.bridge = CvBridge()
        
        # 存储最新图像的变量
        self.latest_image = None
        self.image_lock = threading.Lock()  # 线程锁，确保线程安全
        self.image_received = False  # 标记是否已接收到图像
        
        # 订阅彩色图像话题（假设驱动发布在 /camera/color/image_raw）
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10  # 队列深度
        )
        self.subscription  # 防止未使用警告
        self.get_logger().info('已启动 Gemini 彩色图像订阅节点，等待接收图像...')

    def image_callback(self, msg: Image):
        try:
            # 将 sensor_msgs/Image 转换为 OpenCV BGR 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge 转换失败: {e}')
            return

        # 使用锁来确保线程安全地更新图像
        with self.image_lock:
            self.latest_image = cv_image.copy()  # 复制图像数据
            self.image_received = True

        # 在窗口中显示图像（可选）
        cv2.imshow('Gemini Color Image', cv_image)
        cv2.waitKey(1)  # 必须调用，否则图像窗口无法刷新

    def get_latest_image(self):
        """
        获取最新的彩色图像
        
        Returns:
            numpy.ndarray: 最新的彩色图像 (BGR格式)，如果没有接收到图像则返回 None
        """
        with self.image_lock:
            if self.image_received and self.latest_image is not None:
                return self.latest_image.copy()  # 返回图像的副本
            else:
                return None
    
    def is_image_available(self):
        """
        检查是否有图像可用
        
        Returns:
            bool: 如果有图像可用返回 True，否则返回 False
        """
        with self.image_lock:
            return self.image_received and self.latest_image is not None
    
    def wait_for_image(self, timeout=5.0):
        """
        等待图像到达
        
        Args:
            timeout (float): 超时时间（秒）
            
        Returns:
            bool: 如果在超时时间内接收到图像返回 True，否则返回 False
        """
        import time
        start_time = time.time()
        
        while not self.is_image_available():
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return True
    
    def get_image_info(self):
        """
        获取图像信息
        
        Returns:
            dict: 包含图像尺寸等信息的字典，如果没有图像则返回 None
        """
        with self.image_lock:
            if self.image_received and self.latest_image is not None:
                height, width, channels = self.latest_image.shape
                return {
                    'width': width,
                    'height': height,
                    'channels': channels,
                    'dtype': str(self.latest_image.dtype)
                }
            else:
                return None

def main(args=None):
    rclpy.init(args=args)
    node = GeminiImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
