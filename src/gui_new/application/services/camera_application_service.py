"""
摄像头应用服务 - Application层
协调摄像头Domain服务和消息显示
"""
from PyQt5.QtCore import QObject, pyqtSignal
from domain.services.vision import CameraDomainService
from infrastructure.communication import MessageDisplay


class CameraApplicationService(QObject):
    """
    摄像头应用服务
    
    职责：
    - 协调CameraDomainService
    - 处理业务逻辑
    - 统一消息显示
    """
    
    # ========== 信号定义 ==========
    connection_status_changed = pyqtSignal(bool)  # 连接状态
    
    def __init__(
        self,
        camera_service: CameraDomainService,
        message_display: MessageDisplay
    ):
        super().__init__()
        self.camera_service = camera_service
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
    
    # ========== 私有方法 ==========
    
    def _connect_signals(self):
        """连接Domain Service信号"""
        self.camera_service.connection_status_changed.connect(
            self._on_connection_status_changed
        )
        self.camera_service.error_occurred.connect(self._on_error)
    
    def _on_connection_status_changed(self, connected: bool, message: str):
        """处理连接状态变化"""
        # 显示消息（Domain Service提供的详细消息）
        msg_type = "摄像头" if not message.startswith("错误") else "错误"
        self._display_message(message, msg_type)
        
        # 发射信号给ViewModel
        self.connection_status_changed.emit(connected)
    
    def _on_error(self, error_msg: str):
        """处理错误"""
        self._display_message(error_msg, "错误")
    
    def _display_message(self, message: str, msg_type: str = "摄像头"):
        """显示消息的统一接口"""
        self.message_display.display_message(message, msg_type)

