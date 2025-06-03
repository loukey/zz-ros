"""
动力学相关组件
"""
from PyQt5.QtWidgets import (QWidget, QPushButton, QVBoxLayout, QHBoxLayout, 
                           QGroupBox, QFrame)
from PyQt5.QtCore import pyqtSignal
from .base_components import default_font


class DynamicsFrame(QGroupBox):
    """动力学控制框架"""
    
    # 定义信号
    start_cyclic_torque_requested = pyqtSignal()
    teaching_mode_toggle_requested = pyqtSignal(bool)  # True为开启，False为关闭
    
    def __init__(self, parent=None):
        """
        初始化动力学控制框架
        
        参数:
            parent: 父控件
        """
        super().__init__("动力学控制", parent)
        self.teaching_mode_enabled = False
        self.buttons = []
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 创建按钮区域
        button_layout = QHBoxLayout()
        
        # 启动周期力矩模式按钮
        self.cyclic_torque_button = QPushButton("启动周期力矩模式")
        self.cyclic_torque_button.setFont(default_font)
        self.cyclic_torque_button.clicked.connect(self.start_cyclic_torque_requested.emit)
        self.cyclic_torque_button.setEnabled(False)
        button_layout.addWidget(self.cyclic_torque_button)
        self.buttons.append(self.cyclic_torque_button)
        
        # 示教模式按钮
        self.teaching_mode_button = QPushButton("开启示教模式")
        self.teaching_mode_button.setFont(default_font)
        self.teaching_mode_button.clicked.connect(self._toggle_teaching_mode)
        self.teaching_mode_button.setEnabled(False)
        button_layout.addWidget(self.teaching_mode_button)
        self.buttons.append(self.teaching_mode_button)
        
        # 添加伸缩项让按钮居左
        button_layout.addStretch()
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def _toggle_teaching_mode(self):
        """切换示教模式状态"""
        self.teaching_mode_enabled = not self.teaching_mode_enabled
        
        if self.teaching_mode_enabled:
            self.teaching_mode_button.setText("关闭示教模式")
        else:
            self.teaching_mode_button.setText("开启示教模式")
        
        # 发射信号
        self.teaching_mode_toggle_requested.emit(self.teaching_mode_enabled)
    
    def update_connection_status(self, connected):
        """
        更新连接状态
        
        参数:
            connected: 是否已连接
        """
        for button in self.buttons:
            button.setEnabled(connected)
    
    def set_teaching_mode_enabled(self, enabled):
        """
        设置示教模式状态（外部调用）
        
        参数:
            enabled: 是否启用示教模式
        """
        self.teaching_mode_enabled = enabled
        if enabled:
            self.teaching_mode_button.setText("关闭示教模式")
        else:
            self.teaching_mode_button.setText("开启示教模式")
