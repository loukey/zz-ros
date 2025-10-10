"""
摄像头应用服务 - Application层
协调摄像头Domain服务、检测服务和消息显示
"""
from PyQt5.QtCore import QObject, pyqtSignal
from controller.domain import CameraDomainService, RecognitionDomainService
from ..commands import MessageDisplay


class CameraApplicationService(QObject):
    """
    摄像头应用服务
    
    职责：
    - 协调CameraDomainService和RecognitionDomainService
    - 处理业务逻辑
    - 统一消息显示
    """
    
    # ========== 信号定义 ==========
    connection_status_changed = pyqtSignal(bool)  # 摄像头连接状态
    detection_status_changed = pyqtSignal(bool)  # 检测状态
    
    def __init__(
        self,
        camera_service: CameraDomainService,
        recognition_service: RecognitionDomainService,
        message_display: MessageDisplay
    ):
        super().__init__()
        self.camera_service = camera_service
        self.recognition_service = recognition_service
        self.message_display = message_display
        
        # 连接Domain Service信号
        self._connect_signals()
    
    # ========== 核心方法 ==========
    
    def connect_camera(self):
        """连接摄像头"""
        self.message_display.clear_messages()
        self._display_message("正在连接摄像头...", "摄像头")
        
        success = self.camera_service.connect()
        
        if success:
            self._display_message("摄像头连接成功", "摄像头")
            self.connection_status_changed.emit(True)
        else:
            self._display_message("摄像头连接失败", "错误")
            self.connection_status_changed.emit(False)
    
    def disconnect_camera(self):
        """断开摄像头"""
        self.message_display.clear_messages()
        self._display_message("正在断开摄像头...", "摄像头")
        
        success = self.camera_service.disconnect()
        
        if success:
            self._display_message("摄像头已断开", "摄像头")
            self.connection_status_changed.emit(False)
        else:
            self._display_message("断开摄像头失败", "错误")
    
    def start_detection(self):
        """开始检测"""
        self.message_display.clear_messages()
        self._display_message("正在启动检测...", "检测")
        
        success = self.recognition_service.start_detection()
        
        if success:
            self._display_message("开始零件识别", "检测")
            self.detection_status_changed.emit(True)
        else:
            self._display_message("启动检测失败", "错误")
            self.detection_status_changed.emit(False)
    
    def stop_detection(self):
        """停止检测"""
        self.message_display.clear_messages()
        self._display_message("正在停止检测...", "检测")
        
        success = self.recognition_service.stop_detection()
        
        if success:
            self._display_message("停止零件识别", "检测")
            self.detection_status_changed.emit(False)
        else:
            self._display_message("停止检测失败", "错误")
    
    # ========== 私有方法 ==========
    
    def _connect_signals(self):
        """连接Domain Service信号"""
        # 摄像头服务信号
        self.camera_service.connection_status_changed.connect(
            self._on_camera_connection_status_changed
        )
        self.camera_service.error_occurred.connect(self._on_camera_error)
        
        # 检测服务信号
        self.recognition_service.detection_status_changed.connect(
            self._on_detection_status_changed
        )
        self.recognition_service.detection_result_received.connect(
            self._on_detection_result_received
        )
        self.recognition_service.error_occurred.connect(self._on_detection_error)
    
    def _on_camera_connection_status_changed(self, connected: bool, message: str):
        """处理摄像头连接状态变化"""
        # 显示消息（Domain Service提供的详细消息）
        msg_type = "摄像头" if not message.startswith("错误") else "错误"
        self._display_message(message, msg_type)
        
        # 发射信号给ViewModel
        self.connection_status_changed.emit(connected)
    
    def _on_camera_error(self, error_msg: str):
        """处理摄像头错误"""
        self._display_message(error_msg, "错误")
    
    def _on_detection_status_changed(self, is_running: bool, message: str):
        """处理检测状态变化"""
        # 显示消息
        msg_type = "检测"
        self._display_message(message, msg_type)
        
        # 发射信号给ViewModel
        self.detection_status_changed.emit(is_running)
    
    def _on_detection_result_received(self, detection: dict):
        """处理检测结果"""
        # 格式化检测结果并显示
        try:
            angle = detection.get('angle', 0.0)
            depth = detection.get('depth', 0.0)
            real_depth = detection.get('real_depth', 0.0)
            central_center = detection.get('central_center', (0, 0))
            
            info_msg = (
                f"检测结果 - "
                f"中心点: ({central_center[0]:.1f}, {central_center[1]:.1f}), "
                f"角度: {angle:.2f}rad, "
                f"深度: {depth:.3f}m, "
                f"实际深度: {real_depth:.3f}m"
            )
            self._display_message(info_msg, "检测")
        except Exception as e:
            self._display_message(f"检测结果格式错误: {e}", "错误")
    
    def _on_detection_error(self, error_msg: str):
        """处理检测错误"""
        self._display_message(error_msg, "错误")
    
    def _display_message(self, message: str, msg_type: str = "摄像头"):
        """显示消息的统一接口"""
        self.message_display.display_message(message, msg_type)

