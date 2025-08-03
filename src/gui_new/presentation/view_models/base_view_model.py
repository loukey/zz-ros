"""
基础视图模型 - 提供通用的视图模型功能
"""
from PyQt5.QtCore import QObject, pyqtSignal


class BaseViewModel(QObject):
    """基础视图模型类"""
    
    # 通用信号
    status_message_changed = pyqtSignal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_initialized = False
    
    def emit_status(self, message: str):
        """发出状态消息"""
        self.status_message_changed.emit(message)
    
    def cleanup(self):
        """清理资源 - 子类可以重写"""
        pass 
    