"""
摄像头视图模型 - Presentation层
"""
from .base_view_model import BaseViewModel


class CameraViewModel(BaseViewModel):
    """摄像头视图模型"""
    
    def __init__(self, parent=None):
        super().__init__(parent)

