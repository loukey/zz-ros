#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class GeminiImageSubscriber(Node):
    def __init__(self):
        super().__init__('gemini_image_subscriber')
        # 创建 CvBridge 对象
        self.bridge = CvBridge()
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

        # 在窗口中显示图像
        cv2.imshow('Gemini Color Image', cv_image)
        cv2.waitKey(1)  # 必须调用，否则图像窗口无法刷新

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
