from PyQt5.QtCore import QObject, pyqtSignal


class MessageDisplay(QObject):
    """消息显示服务。
    
    负责在界面上显示日志和状态消息，通过信号分发给 UI 组件。
    
    Attributes:
        message_display_signal (pyqtSignal): 消息显示信号，携带 (消息内容, 消息类型)。
        clear_requested (pyqtSignal): 清除消息请求信号。
    """
    
    message_display_signal = pyqtSignal(str, str)
    clear_requested = pyqtSignal()

    def __init__(self):
        """初始化消息显示服务。"""
        super().__init__()

    def display_message(self, message: str, message_type: str) -> None:
        """显示一条消息。
        
        Args:
            message (str): 消息内容。
            message_type (str): 消息类型（如 "info", "error", "warning"）。
        """
        self.message_display_signal.emit(message, message_type)

    def clear_messages(self) -> None:
        """清除所有消息。"""
        self.clear_requested.emit()
        