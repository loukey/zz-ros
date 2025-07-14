from .yolo_segmentor import YOLOSegmentor
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from interface.msg import DetectionResult
import numpy as np


class RecognitionPub(Node):
    def __init__(self):
        super().__init__('recognition_pub')
        self.color_image = None
        self.depth_image = None
        self.bridge = CvBridge()
        
        # 直接在这个节点中创建订阅者，而不是创建独立的节点
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._on_color_image_received,
            10
        )
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self._on_depth_image_received,
            10
        )
        
        self.yolo_segmentor = YOLOSegmentor(model_path='./yolo_obb.pt')
        self.get_logger().info('RecognitionPub initialized')
        
        self.publisher_ = self.create_publisher(
            DetectionResult,
            'recognition_result',
            10
        )
        
        # 创建100ms的定时器
        self.timer = self.create_timer(0.1, self.timer_callback)  # 100ms = 0.1s
        self.get_logger().info('开始每100ms检测一次')

    def _on_color_image_received(self, msg):
        """彩色图像回调函数"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'彩色图像转换失败: {e}')

    def _on_depth_image_received(self, msg):
        """深度图像回调函数"""
        try:
            # 处理不同的深度图编码格式
            if msg.encoding == 'mono16' or msg.encoding == '16UC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == '32FC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                # 尝试直接转换
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            self.depth_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'深度图像转换失败: {e}')
        except Exception as e:
            self.get_logger().error(f'深度图像处理失败: {e}')

    def timer_callback(self):
        """定时器回调函数，每100ms执行一次检测"""
        if self.color_image is not None and self.depth_image is not None:
            # 进行检测
            detection_result = self.detect(self.color_image, self.depth_image)
            
            # 如果检测到结果，发布消息
            if detection_result:
                self.publish_detection_result(detection_result)
                self.get_logger().info(f'检测到目标，角度: {detection_result.get("angle", 0):.2f}')
            else:
                self.get_logger().debug('未检测到目标')
        else:
            missing = []
            if self.color_image is None:
                missing.append('彩色图像')
            if self.depth_image is None:
                missing.append('深度图像')
            self.get_logger().debug(f'等待数据: {", ".join(missing)}')

    def detect(self, color_image, depth_image):
        result = self.yolo_segmentor.detect(color_image)
        if result:
            height, width = depth_image.shape[:2]
            
            # 计算central_center的深度
            x, y = result['central_center']
            x = max(0, min(int(x), width - 1))
            y = max(0, min(int(y), height - 1))
            depth_value = depth_image[y, x]
            result['depth'] = float(depth_value)
            
            # 计算real_center的深度
            real_x, real_y = result['real_center']
            real_x = max(0, min(int(real_x), width - 1))
            real_y = max(0, min(int(real_y), height - 1))
            real_depth_value = depth_image[real_y, real_x]
            result['real_depth'] = float(real_depth_value)
            
        return result

    def publish_detection_result(self, detection_result):
        msg = DetectionResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.head_center = [float(x) for x in detection_result['head_center']]
        msg.central_center = [float(x) for x in detection_result['central_center']]
        msg.real_center = [float(x) for x in detection_result['real_center']]
        msg.angle = float(detection_result['angle'])
        msg.depth = float(detection_result['depth'])
        msg.real_depth = float(detection_result['real_depth'])
        
        self.publisher_.publish(msg)
        self.get_logger().debug(f'发布检测结果: head_center={msg.head_center}, central_center={msg.central_center}, real_center={msg.real_center}, angle={msg.angle:.2f}, depth={msg.depth:.3f}m, real_depth={msg.real_depth:.3f}m')
    
def main():
    rclpy.init()
    recognition_pub = RecognitionPub()
    rclpy.spin(recognition_pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()