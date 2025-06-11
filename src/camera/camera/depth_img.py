#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class GeminiDepthSubscriber(Node):
    def __init__(self):
        super().__init__('gemini_depth_subscriber')
        # 创建 CvBridge 对象
        self.bridge = CvBridge()
        # 订阅深度图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # 深度图topic
            self.depth_callback,
            10  # 队列深度
        )
        self.subscription  # 防止未使用警告
        self.get_logger().info('已启动 Gemini 深度图像订阅节点，等待接收深度图像...')

    def depth_callback(self, msg: Image):
        try:
            # 深度图像通常是16位无符号整数或32位浮点数
            # 先尝试16UC1格式
            if msg.encoding == 'mono16' or msg.encoding == '16UC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == '32FC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                # 如果不确定编码格式，让cv_bridge自动处理
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge 转换失败: {e}')
            return

        # 处理深度图像以便显示
        # 将深度值归一化到0-255范围用于显示
        if cv_image.dtype == np.uint16:
            # 16位深度图
            depth_display = cv2.convertScaleAbs(cv_image, alpha=255.0/cv_image.max())
        elif cv_image.dtype == np.float32:
            # 32位浮点深度图
            depth_normalized = cv_image / np.nanmax(cv_image) * 255.0
            depth_display = depth_normalized.astype(np.uint8)
        else:
            depth_display = cv_image

        # 应用伪彩色映射以便更好地观察深度信息
        depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
        
        # 在窗口中显示深度图像
        cv2.imshow('Gemini Depth Image (Raw)', depth_display)
        cv2.imshow('Gemini Depth Image (Colormap)', depth_colormap)
        cv2.waitKey(1)  # 必须调用，否则图像窗口无法刷新

        # 打印一些深度信息（每100帧打印一次，避免日志过多）
        if hasattr(self, 'frame_count'):
            self.frame_count += 1
        else:
            self.frame_count = 1
            
        if self.frame_count % 100 == 0:
            if cv_image.dtype == np.uint16 or cv_image.dtype == np.float32:
                min_depth = np.nanmin(cv_image[cv_image > 0])  # 忽略0值
                max_depth = np.nanmax(cv_image)
                self.get_logger().info(f'深度范围: {min_depth:.2f} - {max_depth:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = GeminiDepthSubscriber()
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
