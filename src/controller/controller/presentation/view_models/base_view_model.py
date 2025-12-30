"""
基础视图模型 - 提供通用的视图模型功能
"""
from PyQt5.QtCore import QObject, pyqtSignal


class BaseViewModel(QObject):
    """基础视图模型类。
    
    所有视图模型的基类，提供通用的信号和清理接口。
    
    Attributes:
        connection_status_changed (pyqtSignal): 连接状态变更信号，携带 bool 状态。
    """
    connection_status_changed = pyqtSignal(bool)

    def __init__(self, parent=None):
        """初始化基础视图模型。
        
        Args:
            parent (QObject, optional): 父对象. Defaults to None.
        """
        super().__init__(parent)
    
    def cleanup(self):
        """清理资源。
        
        子类应重写此方法以释放资源（如关闭连接、停止定时器等）。
        """
        pass 
    