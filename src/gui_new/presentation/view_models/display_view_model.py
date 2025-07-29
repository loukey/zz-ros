"""
显示视图模型 - Presentation层
作为消息显示的中心分发器，其他服务调用它的方法来显示消息
"""
from PyQt5.QtCore import pyqtSignal
from typing import List, Dict, Any, Optional
from presentation.view_models.base_view_model import BaseViewModel


class DisplayViewModel(BaseViewModel):
    """显示视图模型 - 消息显示中心分发器"""
    
    # 消息显示信号，供DataDisplayFrame连接
    message_display = pyqtSignal(str, str)  # (消息内容, 消息类型)
    
    def __init__(self, parent=None):
        """初始化显示视图模型"""
        super().__init__(parent)
    
    def append_message(self, message: str, message_type: str = "接收") -> None:
        """添加消息 - 供其他服务调用的公共方法"""
        # 发送消息显示信号，由DataDisplayFrame接收并显示
        self.message_display.emit(message, message_type)
    
    def clear_messages(self) -> None:
        """清除所有消息"""
        # 发送清除消息信号
        self.clear_requested.emit()
    
    # 清除消息请求信号
    clear_requested = pyqtSignal()
    
    def cleanup(self) -> None:
        """清理资源"""
        super().cleanup() 