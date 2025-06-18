#!/usr/bin/env python3
"""
实时图像去畸变ROS2节点
使用相机标定结果对实时图像进行去畸变处理
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys

# 导入相机工具
from .camera_utils import CameraCalibrationUtils, create_your_camera_utils


class UndistortNode(Node):
    """图像去畸变节点"""
    
    def __init__(self):
        super().__init__('undistort_node')
        
        # 声明参数
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/camera/undistorted/image_raw')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('show_comparison', True)
        
        # 获取参数
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.show_comparison = self.get_parameter('show_comparison').get_parameter_value().bool_value
        
        # 初始化
        self.bridge = CvBridge()
        
        # 加载标定参数
        if self.calibration_file and os.path.exists(self.calibration_file):
            self.camera_utils = CameraCalibrationUtils(self.calibration_file)
            self.get_logger().info(f'从文件加载标定参数: {self.calibration_file}')
        else:
            # 使用你的标定结果
            self.camera_utils = create_your_camera_utils()
            self.get_logger().info('使用内置标定参数')
        
        # 预计算去畸变映射表（提高性能）
        self.map1, self.map2 = self.camera_utils.create_undistort_maps()
        
        # 订阅和发布
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        
        self.image_pub = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        
        self.frame_count = 0
        
        self.get_logger().info(f'图像去畸变节点启动')
        self.get_logger().info(f'输入话题: {self.input_topic}')
        self.get_logger().info(f'输出话题: {self.output_topic}')
        self.get_logger().info(f'显示对比: {self.show_comparison}')
        
        # 打印标定信息
        self.camera_utils.print_calibration_info()
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换ROS图像到OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.frame_count += 1
            
            # 去畸变
            undistorted_image = self.camera_utils.fast_undistort(cv_image, self.map1, self.map2)
            
            # 发布去畸变图像
            undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_image, "bgr8")
            undistorted_msg.header = msg.header  # 保持时间戳
            self.image_pub.publish(undistorted_msg)
            
            # 显示对比图像
            if self.show_comparison:
                self.show_comparison_images(cv_image, undistorted_image)
            
            # 每100帧输出一次处理信息
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'已处理 {self.frame_count} 帧图像')
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
    
    def show_comparison_images(self, original, undistorted):
        """显示原图和去畸变图像的对比"""
        # 创建对比图像
        h, w = original.shape[:2]
        comparison = np.zeros((h, w * 2, 3), dtype=np.uint8)
        
        # 左侧显示原图
        comparison[:, :w] = original
        cv2.putText(comparison, 'Original', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # 右侧显示去畸变图像
        comparison[:, w:] = undistorted
        cv2.putText(comparison, 'Undistorted', (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 添加分割线
        cv2.line(comparison, (w, 0), (w, h), (255, 255, 255), 2)
        
        # 显示对比图像
        cv2.imshow('Original vs Undistorted', comparison)
        
        # 检查按键
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC键
            self.get_logger().info('收到退出信号')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        undistort_node = UndistortNode()
        rclpy.spin(undistort_node)
    except KeyboardInterrupt:
        print("程序被中断")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 