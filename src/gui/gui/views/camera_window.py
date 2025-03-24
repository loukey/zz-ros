"""
仿真窗口视图
"""
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGroupBox, QComboBox, QSlider
from PyQt5.QtCore import Qt
import numpy as np


class CameraWindow(QWidget):
    """相机窗口类"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        # 创建主布局
        main_layout = QVBoxLayout(self)
        
