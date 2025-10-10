"""
基础视图模型 - 提供通用的视图模型功能
"""
from PyQt5.QtCore import QObject, pyqtSignal


class BaseViewModel(QObject):
    """基础视图模型类"""
    connection_status_changed = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
    
    def cleanup(self):
        """清理资源 - 子类可以重写"""
        pass 
    