"""
动力学相关组件
"""
from PyQt5.QtWidgets import (QWidget, QPushButton, QVBoxLayout, QHBoxLayout, 
                           QGroupBox, QFrame, QLabel, QLineEdit, QGridLayout)
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QDoubleValidator
from .base_components import default_font


class DynamicsFrame(QGroupBox):
    """动力学控制框架"""
    
    # 定义信号
    start_cyclic_torque_requested = pyqtSignal()
    teaching_mode_toggle_requested = pyqtSignal(bool)  # True为开启，False为关闭
    send_torque_requested = pyqtSignal(list)  # 发送力矩信号
    
    def __init__(self, parent=None):
        """
        初始化动力学控制框架
        
        参数:
            parent: 父控件
        """
        super().__init__("动力学控制", parent)
        self.teaching_mode_enabled = False
        self.buttons = []
        self.torque_inputs = []
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 创建基本控制按钮区域
        basic_control_layout = QHBoxLayout()
        
        # 启动周期力矩模式按钮
        self.cyclic_torque_button = QPushButton("启动周期力矩模式")
        self.cyclic_torque_button.setFont(default_font)
        self.cyclic_torque_button.clicked.connect(self.start_cyclic_torque_requested.emit)
        self.cyclic_torque_button.setEnabled(False)
        basic_control_layout.addWidget(self.cyclic_torque_button)
        self.buttons.append(self.cyclic_torque_button)
        
        # 示教模式按钮
        self.teaching_mode_button = QPushButton("开启示教模式")
        self.teaching_mode_button.setFont(default_font)
        self.teaching_mode_button.clicked.connect(self._toggle_teaching_mode)
        self.teaching_mode_button.setEnabled(False)
        basic_control_layout.addWidget(self.teaching_mode_button)
        self.buttons.append(self.teaching_mode_button)
        
        # 添加伸缩项让按钮居左
        basic_control_layout.addStretch()
        
        layout.addLayout(basic_control_layout)
        
        # 添加分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        
        # 创建力矩输入区域
        torque_group = QGroupBox("力矩输入 (Nm)")
        torque_layout = QVBoxLayout(torque_group)
        
        # 创建力矩输入网格布局
        torque_grid = QGridLayout()
        torque_grid.setSpacing(8)
        
        # 创建6个力矩输入框
        validator = QDoubleValidator(-1000.0, 1000.0, 3)  # 允许-1000到1000，3位小数
        for i in range(6):
            row = i // 3
            col = (i % 3) * 2
            
            # 标签
            label = QLabel(f"力矩{i+1}:")
            label.setFont(default_font)
            torque_grid.addWidget(label, row, col)
            
            # 输入框
            torque_input = QLineEdit("0.0")
            torque_input.setFont(default_font)
            torque_input.setValidator(validator)
            torque_input.setMaximumWidth(80)
            torque_input.setMinimumWidth(80)
            torque_grid.addWidget(torque_input, row, col + 1)
            self.torque_inputs.append(torque_input)
        
        torque_layout.addLayout(torque_grid)
        
        # 力矩控制按钮区域
        torque_button_layout = QHBoxLayout()
        
        # 发送力矩按钮
        self.send_torque_button = QPushButton("发送力矩")
        self.send_torque_button.setFont(default_font)
        self.send_torque_button.clicked.connect(self._send_torque)
        self.send_torque_button.setEnabled(False)
        torque_button_layout.addWidget(self.send_torque_button)
        self.buttons.append(self.send_torque_button)
        
        # 清零按钮
        clear_button = QPushButton("清零")
        clear_button.setFont(default_font)
        clear_button.clicked.connect(self._clear_torque_inputs)
        torque_button_layout.addWidget(clear_button)
        
        # 添加伸缩项
        torque_button_layout.addStretch()
        
        torque_layout.addLayout(torque_button_layout)
        layout.addWidget(torque_group)
        
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
    
    def _send_torque(self):
        """发送力矩数据"""
        try:
            torque_values = []
            for i, torque_input in enumerate(self.torque_inputs):
                value = float(torque_input.text() or "0.0")
                torque_values.append(value)
            
            # 发射信号
            self.send_torque_requested.emit(torque_values)
            
        except ValueError:
            # 如果输入不是有效数字，可以在这里处理错误
            print("力矩输入格式错误")
    
    def _clear_torque_inputs(self):
        """清零所有力矩输入框"""
        for torque_input in self.torque_inputs:
            torque_input.setText("0.0")
    
    def get_torque_values(self):
        """获取当前力矩值"""
        try:
            return [float(torque_input.text() or "0.0") for torque_input in self.torque_inputs]
        except ValueError:
            return [0.0] * 6
    
    def set_torque_values(self, torque_values):
        """设置力矩值"""
        if len(torque_values) == 6:
            for i, value in enumerate(torque_values):
                self.torque_inputs[i].setText(f"{value:.3f}")
    
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
