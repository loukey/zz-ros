"""
零件识别领域服务 - Domain层
管理ROS2检测结果订阅节点（ROS2 可选，无 ROS2 时识别功能不可用）
"""
import threading
import copy
from typing import Optional, Dict
from PyQt5.QtCore import QObject, pyqtSignal

# ROS2 相关导入（可选，Windows 上无 ROS2 时跳过）
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from interface.msg import DetectionResult
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

if HAS_ROS2:
    class DetectionSubscriberNode(Node):
        """检测结果订阅节点。"""

        def __init__(self, callback):
            super().__init__('detection_subscriber')
            self.callback = callback
            self.subscription = self.create_subscription(
                DetectionResult, 'recognition_result', self.detection_callback, 10
            )

        def detection_callback(self, msg):
            try:
                detection_result = {
                    'head_center': tuple(msg.head_center),
                    'central_center': tuple(msg.central_center),
                    'real_center': tuple(msg.real_center),
                    'angle': msg.angle,
                    'depth': msg.depth,
                    'real_depth': msg.real_depth
                }
                if self.callback:
                    self.callback(detection_result)
            except Exception as e:
                self.get_logger().error(f'处理检测结果失败: {e}')


class RecognitionDomainService(QObject):
    """零件识别领域服务。

    职责：
    - 订阅 recognition_result ROS 话题
    - 维护最新检测结果（线程安全）
    - 独立于摄像头服务运行
    - 发射检测结果信号
    """

    # ========== 信号定义 ==========
    detection_result_received = pyqtSignal(dict)
    detection_status_changed = pyqtSignal(bool, str)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        """初始化零件识别领域服务。"""
        super().__init__()

        # ROS节点和执行器
        self.detection_subscriber = None
        self.ros_executor = None
        self.ros_thread = None

        # 检测结果存储（线程安全）
        self.latest_detection_result = {}
        self.detection_lock = threading.Lock()

        # 运行状态
        self.is_running = False

    # ========== 核心方法 ==========

    def start_detection(self) -> bool:
        """开始检测（启动 ROS 订阅）。无 ROS2 环境时返回 False。"""
        if not HAS_ROS2:
            self.error_occurred.emit("当前环境无 ROS2，识别功能不可用")
            return False
        try:
            if self.is_running:
                return True

            if not rclpy.ok():
                try:
                    rclpy.init()
                except RuntimeError as e:
                    self.error_occurred.emit(f"ROS状态异常: {str(e)}")
                    return False
                except Exception as e:
                    self.error_occurred.emit(f"ROS初始化失败: {str(e)}")
                    return False

            self.detection_subscriber = DetectionSubscriberNode(self._on_detection_received)

            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.detection_subscriber)

            self.ros_thread = threading.Thread(target=self._ros_spin_worker, daemon=True)
            self.ros_thread.start()

            self.is_running = True
            self.detection_status_changed.emit(True, "开始零件识别")
            return True

        except Exception as e:
            self.error_occurred.emit(f"启动检测失败: {str(e)}")
            return False

    def stop_detection(self) -> bool:
        """停止检测（关闭 ROS 订阅）。"""
        try:
            if not self.is_running:
                return True

            self.is_running = False

            if self.ros_executor:
                self.ros_executor.shutdown()

            if self.detection_subscriber:
                self.detection_subscriber.destroy_node()
                self.detection_subscriber = None

            if self.ros_thread and self.ros_thread.is_alive():
                self.ros_thread.join(timeout=1.0)

            self.ros_executor = None
            self.ros_thread = None

            with self.detection_lock:
                self.latest_detection_result = {}

            self.detection_status_changed.emit(False, "停止零件识别")
            return True

        except Exception as e:
            self.error_occurred.emit(f"停止检测失败: {str(e)}")
            return False

    def get_latest_result(self) -> Optional[Dict]:
        """获取最新检测结果（线程安全）。"""
        with self.detection_lock:
            if self.latest_detection_result:
                return copy.deepcopy(self.latest_detection_result)
            return None

    def is_detection_running(self) -> bool:
        """检查检测是否正在运行。"""
        return self.is_running

    def clear_result(self):
        """清除检测结果。"""
        with self.detection_lock:
            self.latest_detection_result = {}

    # ========== 私有方法 ==========

    def _on_detection_received(self, detection_dict: Dict):
        """检测结果接收回调（ROS 线程调用）。"""
        with self.detection_lock:
            self.latest_detection_result = detection_dict
        self.detection_result_received.emit(detection_dict)

    def _ros_spin_worker(self):
        """ROS 执行器工作线程。"""
        try:
            self.ros_executor.spin()
        except Exception as e:
            self.error_occurred.emit(f"ROS执行器错误: {str(e)}")
