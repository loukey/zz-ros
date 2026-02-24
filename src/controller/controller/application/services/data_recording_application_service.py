"""
数据录制应用服务 - Application层
协调录制UI和后台录制节点，管理录制生命周期（ROS2 可选）
"""
import threading
from PyQt5.QtCore import QObject, pyqtSignal
from controller.domain import RobotStateDomainService

# ROS2 相关导入（可选，Windows 上无 ROS2 时录制功能不可用）
try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import SetBool
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

if HAS_ROS2:
    class RecordingServiceClient(Node):
        """录制服务客户端节点 (内部类)"""
        def __init__(self):
            super().__init__('recording_service_client')
            self.cli = self.create_client(SetBool, '/recording/control')

        def send_request(self, start: bool) -> tuple:
            """发送请求，阻塞直到返回或超时"""
            if not self.cli.wait_for_service(timeout_sec=1.0):
                return False, "录制服务未运行 (Service not available)"

            req = SetBool.Request()
            req.data = start

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is not None:
                return future.result().success, future.result().message
            else:
                return False, "服务调用超时或失败"


class DataRecordingApplicationService(QObject):
    """数据录制应用服务。"""

    recording_status_changed = pyqtSignal(bool, str)

    def __init__(
        self,
        robot_state_service: RobotStateDomainService
    ):
        super().__init__()
        self.robot_state_service = robot_state_service
        self._is_recording = False
        self._service_client = None
        self._worker_thread = None

    def start_recording(self):
        """开始录制流程（非阻塞）。无 ROS2 环境时发射错误信号。"""
        if not HAS_ROS2:
            self.recording_status_changed.emit(False, "当前环境无 ROS2，录制功能不可用")
            return
        if self._is_recording:
            return

        self._worker_thread = threading.Thread(
            target=self._do_start_recording,
            daemon=True
        )
        self._worker_thread.start()

    def _do_start_recording(self):
        """实际的启动逻辑（在后台线程执行）"""
        try:
            if not rclpy.ok():
                rclpy.init()

            if not self._service_client:
                self._service_client = RecordingServiceClient()

            self._service_client.send_request(False)

            self.robot_state_service.enable_ros_publishing(True)

            success, msg = self._service_client.send_request(True)

            if success:
                self._is_recording = True
                self.recording_status_changed.emit(True, f"录制已开始: {msg}")
            else:
                self.robot_state_service.enable_ros_publishing(False)
                self.recording_status_changed.emit(False, f"启动失败: {msg}")

        except Exception as e:
            self.robot_state_service.enable_ros_publishing(False)
            self.recording_status_changed.emit(False, f"系统错误: {str(e)}")

    def stop_recording(self):
        """停止录制流程（非阻塞）"""
        if not self._is_recording:
            return

        self._worker_thread = threading.Thread(
            target=self._do_stop_recording,
            daemon=True
        )
        self._worker_thread.start()

    def _do_stop_recording(self):
        """实际的停止逻辑（在后台线程执行）"""
        try:
            if self._service_client:
                success, msg = self._service_client.send_request(False)
                final_msg = f"录制已停止: {msg}" if success else f"停止指令发送失败: {msg}"
            else:
                final_msg = "客户端异常"

            self.robot_state_service.enable_ros_publishing(False)

            self._is_recording = False
            self.recording_status_changed.emit(False, final_msg)

        except Exception as e:
            self.recording_status_changed.emit(False, f"停止失败: {str(e)}")
            self.robot_state_service.enable_ros_publishing(False)
            self._is_recording = False

    def is_recording(self) -> bool:
        return self._is_recording
