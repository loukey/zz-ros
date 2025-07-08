"""
控制相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QLineEdit, QPushButton, QRadioButton, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QButtonGroup, QComboBox,
                           QGroupBox, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator
from .base_components import default_font, GroupFrame
from math import pi


class ControlButtonsFrame(QGroupBox):
    send_command_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None):
        """
        初始化控制按钮框架
        
        参数:
            send_control_command_callback: 发送控制命令的回调函数
            parent: 父控件
        """
        super().__init__("参数配置和控制命令", parent)
        self.buttons = []
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 创建参数配置区域
        config_layout = QVBoxLayout()
        
        # 编码格式选择
        encoding_layout = QHBoxLayout()
        encoding_layout.addWidget(QLabel("编码格式:"))
        self.encoding_combo = QComboBox()
        self.encoding_combo.addItems(['hex', 'string'])
        self.encoding_combo.setFont(default_font)
        encoding_layout.addWidget(self.encoding_combo)
        encoding_layout.addWidget(QLabel("(hex: 十六进制格式, string: 字符串格式)"))
        encoding_layout.addStretch()
        config_layout.addLayout(encoding_layout)
        
        # 运行模式选择
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("运行模式:"))
        self.run_modes = [
            ('轮廓位置模式', 0x01),
            ('轮廓速度模式', 0x03),
            ('轮廓扭矩模式', 0x04),
            ('回零模式(暂不支持)', 0x06),
            ('位置插补模式(暂不支持)', 0x07),
            ('周期同步位置模式', 0x08),
            ('周期同步速度模式', 0x09),
            ('周期同步扭矩模式', 0x0A)
        ]
        self.run_mode_combo = QComboBox()
        self.run_mode_combo.addItems([f"{mode[0]} ({mode[1]:02X})" for mode in self.run_modes])
        self.run_mode_combo.setCurrentText('周期同步位置模式 (08)')
        self.run_mode_combo.setFont(default_font)
        mode_layout.addWidget(self.run_mode_combo)
        mode_layout.addStretch()
        config_layout.addLayout(mode_layout)
        
        layout.addLayout(config_layout)
        
        # 添加分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        
        # 创建控制按钮
        button_layout = QHBoxLayout()
        commands = [
            ("使能", 0x01),
            ("取消使能", 0x02),
            ("释放刹车", 0x03),
            ("锁止刹车", 0x04),
            ("立刻停止", 0x05),
            ("暂停", 0x08)
        ]
        
        for text, command in commands:
            button = QPushButton(text)
            button.setFont(default_font)
            button.clicked.connect(lambda checked, cmd=command: self.send_command_requested.emit({'control': cmd, 
                                                                                                  'encoding': self.get_encoding_type(), 
                                                                                                  'mode': self.get_run_mode()}))
            button.setEnabled(False)
            button_layout.addWidget(button)
            self.buttons.append(button)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def update_connection_status(self, connected):
        """
        更新连接状态
        
        参数:
            connected: 是否已连接
        """
        for button in self.buttons:
            button.setEnabled(connected)
    
    def get_encoding_type(self):
        """获取当前选择的编码格式"""
        return self.encoding_combo.currentText()
    
    def get_run_mode(self):
        """获取当前选择的运行模式"""
        selected = self.run_mode_combo.currentText()
        for mode_name, mode_code in self.run_modes:
            if mode_name in selected:
                return mode_code
        return '0x01'  # 默认返回轮廓位置模式


class AngleControlFrame(QGroupBox):
    """角度控制框架"""
    send_angles_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None, 
                 get_contour=None,
                 get_encoding_type=None,
                 get_run_mode=None):
        super().__init__("角度控制 (弧度值)", parent)
        self.get_contour = get_contour
        self.get_encoding_type = get_encoding_type
        self.get_run_mode = get_run_mode
        self.angle_vars = []
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 创建角度输入区域
        angles_layout = QGridLayout()
        
        # 创建6个角度输入框
        for i in range(6):
            row = i // 3
            col = i % 3
            label = QLabel(f"角度{i+1}")
            label.setFont(default_font)
            angles_layout.addWidget(label, row, col*2)
            
            var = QLineEdit("0.0")
            var.setFont(default_font)
            self.angle_vars.append(var)
            angles_layout.addWidget(var, row, col*2+1)
        
        layout.addLayout(angles_layout)
        
        # 创建曲线类型和按钮区域
        control_layout = QHBoxLayout()
        control_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)  # 设置左对齐
        
        # 曲线类型选择
        curve_group = QButtonGroup()
        self.trapezoidal = QRadioButton("梯形")  # 保存为实例变量
        self.s_curve = QRadioButton("S形")      # 保存为实例变量
        self.s_curve.setChecked(True)
        curve_group.addButton(self.trapezoidal)
        curve_group.addButton(self.s_curve)
        control_layout.addWidget(self.trapezoidal)
        control_layout.addWidget(self.s_curve)
        
        # 时长输入
        time_layout = QHBoxLayout()
        time_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)  # 设置左对齐
        time_layout.addWidget(QLabel("时长:"))
        self.duration_var = QLineEdit("4.0")
        self.duration_var.setFont(default_font)
        self.duration_var.setMaximumWidth(60)
        time_layout.addWidget(self.duration_var)
        time_layout.addWidget(QLabel("s"))
        control_layout.addLayout(time_layout)
        
        # 发送频率输入
        freq_layout = QHBoxLayout()
        freq_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)  # 设置左对齐
        freq_layout.addWidget(QLabel("发送频率:"))
        self.frequency_var = QLineEdit("0.01")
        self.frequency_var.setFont(default_font)
        self.frequency_var.setMaximumWidth(60)
        freq_layout.addWidget(self.frequency_var)
        freq_layout.addWidget(QLabel("s"))
        control_layout.addLayout(freq_layout)
        
        # 添加伸缩项，让前面的内容向左对齐
        control_layout.addStretch(1)
        
        layout.addLayout(control_layout)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        
        self.send_button = QPushButton("发送角度")
        self.send_button.setFont(default_font)
        self.send_button.clicked.connect(lambda: self.send_angles_requested.emit({'target_angles': self.get_angles(), 
                                                                                   'curve_type': self.get_curve_type(), 
                                                                                   'frequency': self.get_frequency(),
                                                                                   'contour_params': self.get_contour(),
                                                                                   'encoding_type': self.get_encoding_type(),
                                                                                   'run_mode': self.get_run_mode()}))
        self.send_button.setEnabled(False)
        button_layout.addWidget(self.send_button)
        
        convert_button = QPushButton("度数转弧度")
        convert_button.setFont(default_font)
        convert_button.clicked.connect(self.convert_angles)
        button_layout.addWidget(convert_button)
        
        zero_button = QPushButton("全部归零")
        zero_button.setFont(default_font)
        zero_button.clicked.connect(self.zero_angles)
        button_layout.addWidget(zero_button)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def get_angles(self):
        """
        获取当前角度值
        
        返回:
            angles: 角度值列表
        """
        try:
            return [float(var.text()) for var in self.angle_vars]
        except ValueError:
            return [0.0] * 6
    
    def get_curve_type(self):
        """
        获取曲线类型和时间参数
        
        返回:
            tuple: (曲线类型, 时长, 发送频率)
        """
        try:
            duration = float(self.duration_var.text())
            frequency = float(self.frequency_var.text())
            curve_type = "trapezoidal" if self.trapezoidal.isChecked() else "s_curve"
            return curve_type, duration, frequency
        except ValueError:
            return "trapezoidal", 4.0, 0.1
    
    def get_frequency(self):
        """获取当前的发送频率"""
        return float(self.frequency_var.text())

    def set_angles(self, angles):
        """
        设置角度值
        
        参数:
            angles: 角度值列表
        """
        for i, angle in enumerate(angles):
            if i < len(self.angle_vars):
                self.angle_vars[i].setText(f"{angle:.6f}")
        
    def update_connection_status(self, connected):
        """更新连接状态"""
        self.send_button.setEnabled(connected)
    
    def convert_angles(self):
        angle_values = []
        for angle_var in self.angle_vars:
            try:
                angle_degrees = float(angle_var.text())
                angle_values.append(angle_degrees)
            except ValueError:
                # 转换失败时使用0.0作为默认值
                angle_values.append(0.0)
        
        # 将角度转换为弧度 (角度 * pi / 180)
        from math import pi
        radian_values = [angle * (pi / 180) for angle in angle_values]
        
        # 更新输入框显示弧度值
        for i, radian in enumerate(radian_values):
            self.angle_vars[i].setText(f"{radian:.4f}")
    
    def zero_angles(self):
        """归零处理"""
        # 将所有角度值设置为0
        zero_angles = [0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]
        for i, angle_var in enumerate(self.angle_vars):
            angle_var.setText(f"{zero_angles[i]:.4f}")
