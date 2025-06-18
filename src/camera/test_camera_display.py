#!/usr/bin/env python3
"""
简单的相机显示测试脚本
用于验证相机话题和OpenCV显示是否正常工作
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraDisplayTest(Node):
    def __init__(self):
        super().__init__('camera_display_test')
        
        self.bridge = CvBridge()
        
        # 订阅相机话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('相机显示测试启动')
        self.get_logger().info('订阅话题: /camera/color/image_raw')
        self.get_logger().info('按 ESC 键退出')
        
        self.frame_count = 0
    
    def image_callback(self, msg):
        try:
            # 将ROS图像转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.frame_count += 1
            
            # 在图像上添加信息
            text = f"Frame: {self.frame_count}"
            cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 显示图像
            cv2.imshow('Camera Test', cv_image)
            
            # 检查键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC键
                self.get_logger().info('收到退出信号')
                rclpy.shutdown()
            
            # 每30帧输出一次日志
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'已接收 {self.frame_count} 帧图像')
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_test = CameraDisplayTest()
        rclpy.spin(camera_test)
    except KeyboardInterrupt:
        print("程序被中断")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 