"""
摄像头数据模型
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal


class CameraModel(QObject):
    """摄像头数据模型类"""
    
    # 定义信号
    color_image_received = pyqtSignal(np.ndarray)  # 彩色图像接收信号
    depth_image_received = pyqtSignal(np.ndarray)  # 深度图像接收信号
    error_occurred = pyqtSignal(str)  # 错误信号
    connection_status_changed = pyqtSignal(str)  # 连接状态变化信号
    
    def __init__(self):
        super().__init__()
        self.color_subscriber = None
        self.depth_subscriber = None
        self.bridge = CvBridge()
        self.ros_executor = None
        self.ros_thread = None
        self.is_ros_initialized = False
        self.is_camera_connected = False
        
        # 图像数据存储
        self.latest_color_image = None
        self.latest_depth_image = None
        self.color_image_lock = threading.Lock()
        self.depth_image_lock = threading.Lock()
        
        # 连接状态
        self.color_connected = False
        self.depth_connected = False
        
    def connect_camera(self):
        """连接摄像头"""
        try:
            if self.is_camera_connected:
                return True
                
            if not rclpy.ok():
                rclpy.init()
            
            # 创建订阅节点
            self._create_subscribers()
            
            # 启动ROS执行器线程
            self._start_ros_thread()
            
            self.is_ros_initialized = True
            self.is_camera_connected = True
            self.connection_status_changed.emit("正在连接摄像头...")
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"摄像头连接失败: {str(e)}")
            return False
    
    def disconnect_camera(self):
        """断开摄像头连接"""
        try:
            self.is_camera_connected = False
            self.color_connected = False
            self.depth_connected = False
            
            # 停止ROS执行器
            if self.ros_executor:
                self.ros_executor.shutdown()
            
            # 销毁节点
            if self.color_subscriber:
                self.color_subscriber.destroy_node()
                self.color_subscriber = None
            if self.depth_subscriber:
                self.depth_subscriber.destroy_node()
                self.depth_subscriber = None
            
            # 等待线程结束
            if self.ros_thread and self.ros_thread.is_alive():
                self.ros_thread.join(timeout=1.0)
            
            self.is_ros_initialized = False
            self.connection_status_changed.emit("摄像头已断开")
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"断开摄像头失败: {str(e)}")
            return False
    
    def _create_subscribers(self):
        """创建ROS2订阅者"""
        try:
            # 创建彩色图像订阅者
            self.color_subscriber = ColorImageSubscriber(self._on_color_image_received)
            
            # 创建深度图像订阅者  
            self.depth_subscriber = DepthImageSubscriber(self._on_depth_image_received)
            
        except Exception as e:
            self.error_occurred.emit(f"创建订阅者失败: {str(e)}")
    
    def _start_ros_thread(self):
        """启动ROS执行器线程"""
        try:
            from rclpy.executors import MultiThreadedExecutor
            
            self.ros_executor = MultiThreadedExecutor()
            self.ros_executor.add_node(self.color_subscriber)
            self.ros_executor.add_node(self.depth_subscriber)
            
            self.ros_thread = threading.Thread(target=self._ros_spin_worker)
            self.ros_thread.daemon = True
            self.ros_thread.start()
            
        except Exception as e:
            self.error_occurred.emit(f"启动ROS线程失败: {str(e)}")
    
    def _ros_spin_worker(self):
        """ROS执行器工作线程"""
        try:
            self.ros_executor.spin()
        except Exception as e:
            self.error_occurred.emit(f"ROS执行器错误: {str(e)}")
    
    def _on_color_image_received(self, cv_image):
        """彩色图像接收回调"""
        with self.color_image_lock:
            self.latest_color_image = cv_image.copy()
            if not self.color_connected:
                self.color_connected = True
                self.connection_status_changed.emit("彩色摄像头已连接")
        
        # 发送信号
        self.color_image_received.emit(cv_image)
    
    def _on_depth_image_received(self, cv_image):
        """深度图像接收回调"""
        with self.depth_image_lock:
            self.latest_depth_image = cv_image.copy()
            if not self.depth_connected:
                self.depth_connected = True
                self.connection_status_changed.emit("深度摄像头已连接")
        
        # 发送信号
        self.depth_image_received.emit(cv_image)
    
    def get_latest_color_image(self):
        """获取最新的彩色图像"""
        with self.color_image_lock:
            return self.latest_color_image.copy() if self.latest_color_image is not None else None
    
    def get_latest_depth_image(self):
        """获取最新的深度图像"""
        with self.depth_image_lock:
            return self.latest_depth_image.copy() if self.latest_depth_image is not None else None
    
    def is_color_available(self):
        """检查彩色图像是否可用"""
        return self.color_connected and self.latest_color_image is not None
    
    def is_depth_available(self):
        """检查深度图像是否可用"""
        return self.depth_connected and self.latest_depth_image is not None
    
    def cleanup(self):
        """清理资源"""
        try:
            # 停止ROS执行器
            if self.ros_executor:
                self.ros_executor.shutdown()
            
            # 销毁节点
            if self.color_subscriber:
                self.color_subscriber.destroy_node()
            if self.depth_subscriber:
                self.depth_subscriber.destroy_node()
            
            # 等待线程结束
            if self.ros_thread and self.ros_thread.is_alive():
                self.ros_thread.join(timeout=1.0)
            
            # 关闭ROS
            if rclpy.ok():
                rclpy.shutdown()
                
            self.is_ros_initialized = False
            self.connection_status_changed.emit("摄像头已断开")
            
        except Exception as e:
            self.error_occurred.emit(f"清理资源失败: {str(e)}")


class ColorImageSubscriber(Node):
    """彩色图像订阅节点"""
    
    def __init__(self, callback):
        super().__init__('color_image_subscriber')
        self.callback = callback
        self.bridge = CvBridge()
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 调用回调函数
            if self.callback:
                self.callback(cv_image)
                
        except CvBridgeError as e:
            self.get_logger().error(f'彩色图像转换失败: {e}')


class DepthImageSubscriber(Node):
    """深度图像订阅节点"""
    
    def __init__(self, callback):
        super().__init__('depth_image_subscriber')
        self.callback = callback
        self.bridge = CvBridge()
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            10
        )
        
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 处理不同的深度图编码格式
            if msg.encoding == 'mono16' or msg.encoding == '16UC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == '32FC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                # 尝试直接转换
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 调用回调函数
            if self.callback:
                self.callback(cv_image)
                
        except CvBridgeError as e:
            self.get_logger().error(f'深度图像转换失败: {e}')
        except Exception as e:
            self.get_logger().error(f'深度图像处理失败: {e}') 