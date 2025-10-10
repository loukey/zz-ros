"""
显示视图模型 - Presentation层
作为消息显示的中心分发器，其他服务调用它的方法来显示消息
"""
from PyQt5.QtCore import pyqtSignal
from typing import List, Dict, Any, Optional
from .base_view_model import BaseViewModel
from controller.application import MessageDisplay


class DisplayViewModel(BaseViewModel):
    """显示视图模型 - 消息显示中心分发器"""
    
    # 消息显示信号，供DataDisplayFrame连接
    message_display_signal = pyqtSignal(str, str)  # (消息内容, 消息类型)
    # 清除消息请求信号
    clear_requested = pyqtSignal()
    
    def __init__(self, message_display: MessageDisplay, parent=None):
        """初始化显示视图模型"""
        super().__init__(parent)
        self.message_display = message_display
        self.message_display.message_display_signal.connect(self.message_display_signal.emit)
        self.message_display.clear_requested.connect(self.clear_requested.emit)

    def clear_messages(self) -> None:
        """清除所有消息"""
        # 发送清除消息信号
        self.clear_requested.emit()
    
    def cleanup(self) -> None:
        """清理资源"""
        super().cleanup() 
        