"""
摄像头领域服务 - Domain层
管理ROS2摄像头订阅节点
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading
from typing import Optional
from PyQt5.QtCore import QObject, pyqtSignal


class ColorImageSubscriber(Node):
    """彩色图像订阅节点"""
    
    def __init__(self, callback):
        super().__init__('color_image_subscriber')
        self.callback = callback
        self.bridge = CvBridge()
        
        # 订阅彩色图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换为OpenCV格式（BGR）
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
        
        # 订阅深度图像话题
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


class CameraDomainService(QObject):
    """
    摄像头领域服务
    
    职责：
    - 管理ROS2摄像头订阅节点的生命周期
    - 维护最新的彩色/深度图像数据（线程安全）
    - 发射图像接收信号
    - 深度图可视化处理
    """
    
    # ========== 信号定义 ==========
    color_image_received = pyqtSignal(np.ndarray)  # 彩色图像接收信号
    depth_image_received = pyqtSignal(np.ndarray)  # 深度图像接收信号
    connection_status_changed = pyqtSignal(bool, str)  # (connected, message)
    error_occurred = pyqtSignal(str)  # 错误信号
    
    def __init__(self):
        super().__init__()
        
        # ROS节点和执行器
        self.color_subscriber = None
        self.depth_subscriber = None
        self.ros_executor = None
        self.ros_thread = None
        
        # cv_bridge
        self.bridge = CvBridge()
        
        # 图像数据存储（线程安全）
        self.latest_color_image = None
        self.latest_depth_image = None
        self.color_image_lock = threading.Lock()
        self.depth_image_lock = threading.Lock()
        
        # 连接状态
        self.is_connected = False
        self.color_connected = False  # 彩色流是否有数据
        self.depth_connected = False  # 深度流是否有数据
    
    # ========== 核心方法 ==========
    
    def connect(self) -> bool:
        """连接摄像头（启动ROS订阅）"""
        try:
            if self.is_connected:
                return True
            
            # 检查并初始化ROS（灵活策略，参考旧版）
            if not rclpy.ok():
                try:
                    rclpy.init()
                    print("✅ ROS2已初始化（摄像头服务）")
                except RuntimeError as e:
                    # ROS已经初始化但状态异常
                    self.error_occurred.emit(f"ROS状态异常: {str(e)}")
                    return False
                except Exception as e:
                    self.error_occurred.emit(f"ROS初始化失败: {str(e)}")
                    return False
            
            # 创建订阅节点
            self.color_subscriber = ColorImageSubscriber(self._on_color_image_received)
            self.depth_subscriber = DepthImageSubscriber(self._on_depth_image_received)
            
            # 创建executor
            self.ros_executor = MultiThreadedExecutor()
            self.ros_executor.add_node(self.color_subscriber)
            self.ros_executor.add_node(self.depth_subscriber)
            
            # 在独立线程运行executor
            self.ros_thread = threading.Thread(target=self._ros_spin_worker, daemon=True)
            self.ros_thread.start()
            
            self.is_connected = True
            self.connection_status_changed.emit(True, "摄像头连接成功")
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"连接摄像头失败: {str(e)}")
            return False
    
    def disconnect(self) -> bool:
        """断开摄像头（关闭ROS订阅）"""
        try:
            if not self.is_connected:
                return True
            
            self.is_connected = False
            self.color_connected = False
            self.depth_connected = False
            
            # 停止executor（只影响摄像头节点）
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
            
            self.ros_executor = None
            self.ros_thread = None
            
            # 清空图像数据
            with self.color_image_lock:
                self.latest_color_image = None
            with self.depth_image_lock:
                self.latest_depth_image = None
            
            self.connection_status_changed.emit(False, "摄像头已断开")
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"断开摄像头失败: {str(e)}")
            return False
    
    def get_latest_color_image(self) -> Optional[np.ndarray]:
        """获取最新彩色图像（线程安全）"""
        with self.color_image_lock:
            return self.latest_color_image.copy() if self.latest_color_image is not None else None
    
    def get_latest_depth_image(self) -> Optional[np.ndarray]:
        """获取最新深度图像（线程安全）"""
        with self.depth_image_lock:
            return self.latest_depth_image.copy() if self.latest_depth_image is not None else None
    
    def is_color_available(self) -> bool:
        """检查彩色图像是否可用"""
        return self.color_connected and self.latest_color_image is not None
    
    def is_depth_available(self) -> bool:
        """检查深度图像是否可用"""
        return self.depth_connected and self.latest_depth_image is not None
    
    def visualize_depth_image(self, depth_image: np.ndarray) -> np.ndarray:
        """
        深度图可视化为伪彩色图
        
        Args:
            depth_image: 原始深度图（16位或32位浮点）
            
        Returns:
            伪彩色深度图（BGR, uint8）
        """
        try:
            # 过滤无效值（depth <= 0 或 nan）
            valid_mask = (depth_image > 0) & np.isfinite(depth_image)
            
            if not np.any(valid_mask):
                # 如果没有有效数据，返回黑色图像
                depth_display = np.zeros_like(depth_image, dtype=np.uint8)
            else:
                # 归一化到 0-255
                min_val = np.min(depth_image[valid_mask])
                max_val = np.max(depth_image[valid_mask])
                
                if max_val - min_val < 1e-6:
                    depth_display = np.zeros_like(depth_image, dtype=np.uint8)
                else:
                    norm = (depth_image - min_val) / (max_val - min_val)
                    norm[~valid_mask] = 0
                    depth_display = (norm * 255).astype(np.uint8)
            
            # 应用伪彩色映射
            depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
            return depth_colormap
            
        except Exception as e:
            # 如果可视化失败，返回黑色图像
            height, width = depth_image.shape[:2]
            return np.zeros((height, width, 3), dtype=np.uint8)
    
    # ========== 私有方法（回调） ==========
    
    def _on_color_image_received(self, cv_image: np.ndarray):
        """彩色图像接收回调（ROS线程调用）"""
        with self.color_image_lock:
            self.latest_color_image = cv_image.copy()
            if not self.color_connected:
                self.color_connected = True
                self.connection_status_changed.emit(True, "彩色摄像头已连接")
        
        # 发射信号（PyQt自动处理跨线程）
        self.color_image_received.emit(cv_image)
    
    def _on_depth_image_received(self, cv_image: np.ndarray):
        """深度图像接收回调（ROS线程调用）"""
        with self.depth_image_lock:
            self.latest_depth_image = cv_image.copy()
            if not self.depth_connected:
                self.depth_connected = True
                self.connection_status_changed.emit(True, "深度摄像头已连接")
        
        # 发射信号
        self.depth_image_received.emit(cv_image)
    
    def _ros_spin_worker(self):
        """ROS执行器工作线程"""
        try:
            self.ros_executor.spin()
        except Exception as e:
            self.error_occurred.emit(f"ROS执行器错误: {str(e)}")

