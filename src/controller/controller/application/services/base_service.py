from PyQt5.QtCore import QObject
from ..commands import MessageDisplay


class BaseService(QObject):
    """应用层基础服务。
    
    所有 Application Service 的基类，提供通用的消息显示功能。
    
    Attributes:
        message_display (MessageDisplay): 消息显示服务实例。
    """
    
    def __init__(self, message_display: MessageDisplay):
        """初始化基础服务。
        
        Args:
            message_display (MessageDisplay): 消息显示服务实例。
        """
        super().__init__()
        self.message_display = message_display

    def _display_message(self, message: str, message_type: str) -> None:
        """显示消息的辅助方法。
        
        Args:
            message (str): 消息内容。
            message_type (str): 消息类型（如 "info", "error"）。
        """
        self.message_display.display_message(message, message_type)
