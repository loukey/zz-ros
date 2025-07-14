"""
检测数据模型 - 从recognition节点获取检测结果
"""
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from interface.msg import DetectionResult
import threading


class DetectionSubscriberNode(Node):
    """ROS2检测结果订阅节点"""
    
    def __init__(self, detection_model, node_id=None):
        # 使用唯一节点名称
        node_name = 'detection_subscriber'
        if node_id:
            node_name = f'detection_subscriber_{node_id}'
        super().__init__(node_name)
        self.detection_model = detection_model
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            DetectionResult,
            'recognition_result',
            self.detection_callback,
            10
        )
        
    def detection_callback(self, msg):
        """接收检测结果的回调函数"""
        try:
            # 将ROS消息转换为字典格式
            detection_result = {
                'head_center': tuple(msg.head_center),
                'central_center': tuple(msg.central_center),
                'real_center': tuple(msg.real_center),
                'angle': msg.angle,
                'depth': msg.depth,
                'real_depth': msg.real_depth
            }
            
            # 更新检测模型
            self.detection_model.update_detection_result(detection_result)
            
        except Exception as e:
            self.get_logger().error(f'处理检测结果失败: {e}')


class DetectionModel(QObject):
    """检测数据模型类 - 从recognition节点获取结果
    
    注意：此模型不依赖摄像头连接，可以独立运行
    它只负责订阅来自recognition节点的检测结果
    """
    
    detection_msg_signal = pyqtSignal(str)  # 消息信号
    
    def __init__(self):
        super().__init__()
        
        # 初始化默认检测结果
        self.latest_detection_result = {}
        
        # 初始化ROS2
        self.ros_node = None
        self.ros_thread = None
        self.ros_executor = None
        self.node_counter = 0  # 用于生成唯一节点ID
        
    def init_ros_node(self):
        """初始化ROS节点"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            
            # 创建ROS节点
            self.ros_node = DetectionSubscriberNode(self)
            
            # 创建执行器
            from rclpy.executors import SingleThreadedExecutor
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            # 在单独的线程中运行ROS节点
            self.ros_thread = threading.Thread(target=self.ros_executor.spin, daemon=True)
            self.ros_thread.start()
            
            self.detection_msg_signal.emit(f"ROS节点已启动，等待检测结果...")
            
        except Exception as e:
            self.detection_msg_signal.emit(f"ROS节点启动失败: {e}")
    
    def update_detection_result(self, detection_result):
        """更新检测结果"""
        self.latest_detection_result = detection_result
        self.detection_msg_signal.emit(f"收到检测结果: {detection_result}")
        
    def start_auto_detection(self):
        """开始自动检测 - 启动ROS节点
        
        注意：此方法不依赖摄像头连接状态，可以直接调用
        它会启动ROS节点来监听recognition节点的检测结果
        """
        if self.ros_node is None:
            self.init_ros_node()
        self.detection_msg_signal.emit("开始监听检测结果...")
    
    def stop_auto_detection(self):
        """停止自动检测 - 停止ROS节点"""
        self.stop_ros_node()
        self.detection_msg_signal.emit("停止监听检测结果")
    
    def stop_ros_node(self):
        """停止ROS节点"""
        try:
            if self.ros_executor:
                self.ros_executor.shutdown()
            
            if self.ros_thread and self.ros_thread.is_alive():
                self.ros_thread.join(timeout=1.0)
            
            self.ros_node = None
            self.ros_executor = None
            self.ros_thread = None
            
        except Exception as e:
            self.detection_msg_signal.emit(f"停止ROS节点失败: {e}")

    def is_detection_running(self):
        """检查检测是否正在运行"""
        return self.ros_node is not None and self.ros_thread is not None and self.ros_thread.is_alive()
    
    def get_latest_detection_result(self):
        """获取最新检测结果"""
        return self.latest_detection_result
    
    def clear_detection_result(self):
        """清除检测结果"""
        self.latest_detection_result = {}
    
    def cleanup(self):
        """清理资源"""
        try:
            self.stop_ros_node()
                
        except Exception as e:
            self.detection_msg_signal.emit(f"清理资源失败: {e}")
