"""
数据录制应用服务 - Application层
协调录制UI和后台录制节点，管理录制生命周期
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from PyQt5.QtCore import QObject, pyqtSignal
from controller.domain import RobotStateDomainService

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
        # 注意：在PyQt主线程中spin可能会卡UI，但由于响应很快，暂时先这样做
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
        
    def start_recording(self):
        """开始录制流程"""
        if self._is_recording:
            return

        try:
            if not rclpy.ok():
                try:
                    rclpy.init()
                except:
                    pass
                
            # 1. 开启 Controller 端的广播
            self.robot_state_service.enable_ros_publishing(True)
            
            # 2. 初始化 Client 并调用服务
            if not self._service_client:
                self._service_client = RecordingServiceClient()
                
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
        """停止录制流程"""
        if not self._is_recording:
            return
            
        try:
            # 1. 调用服务停止
            if self._service_client:
                success, msg = self._service_client.send_request(False)
                final_msg = f"录制已停止: {msg}" if success else f"停止指令发送失败: {msg}"
            else:
                final_msg = "客户端异常"
            
            # 2. 关闭广播
            self.robot_state_service.enable_ros_publishing(False)
            
            self._is_recording = False
            self.recording_status_changed.emit(False, final_msg)
            
        except Exception as e:
            self.recording_status_changed.emit(False, f"停止失败: {str(e)}")
            self.robot_state_service.enable_ros_publishing(False)
            self._is_recording = False

    def is_recording(self) -> bool:
        return self._is_recording
