"""
控制相关UI组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QLineEdit, QPushButton, QRadioButton, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QButtonGroup, QComboBox,
                           QGroupBox, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator
from .base_component import (BaseComponent, default_font, LabeledComboBox, 
                           InputGrid, RadioButtonGroup, ConfigRow, HorizontalLine)
from math import pi


class ControlButtonsFrame(BaseComponent):
    """控制按钮框架"""
    send_command_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None, view_model=None):
        self.buttons = []
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("参数配置和控制命令")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        layout = QVBoxLayout(group_box)
        
        # 创建参数配置区域
        # 注释：编码格式固定为hex模式，无需用户选择
        
        # 运行模式选择 - 使用LabeledComboBox基础组件
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
        
        mode_items = [f"{mode[0]} ({mode[1]:02X})" for mode in self.run_modes]
        self.run_mode_combo = LabeledComboBox(
            "运行模式:",
            items=mode_items
        )
        self.run_mode_combo.set_current_text('周期同步位置模式 (08)')
        
        mode_row = ConfigRow(None)
        mode_row.add_widget(self.run_mode_combo)
        mode_row.add_stretch()
        layout.addWidget(mode_row)
        
        # 添加分隔线
        layout.addWidget(HorizontalLine())
        
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
            button.clicked.connect(lambda checked, cmd=command: self.send_command_requested.emit({
                'control': cmd, 
                'mode': self.get_run_mode()
            }))
            button.setEnabled(False)
            button_layout.addWidget(button)
            self.buttons.append(button)
        
        layout.addLayout(button_layout)
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.connection_status_changed.connect(self.update_connection_status)
            self.send_command_requested.connect(self.view_model.send_command)
    
    def update_connection_status(self, connected):
        """更新连接状态"""
        for button in self.buttons:
            button.setEnabled(connected)
    
    def get_run_mode(self):
        """获取当前选择的运行模式"""
        selected = self.run_mode_combo.current_text()
        for mode_name, mode_code in self.run_modes:
            if mode_name in selected:
                return mode_code
        return 0x01  # 默认返回轮廓位置模式


class AngleControlFrame(BaseComponent):
    """角度控制框架"""
    send_angles_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None, view_model=None,
                 get_contour=None,
                 get_run_mode=None):
        self.get_contour = get_contour
        self.get_run_mode = get_run_mode
        self.angle_vars = []
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("角度控制 (弧度值)")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        layout = QVBoxLayout(group_box)
        
        # 创建角度输入区域 - 使用InputGrid基础组件
        angle_labels = [f"角度{i+1}" for i in range(6)]
        self.angle_grid = InputGrid(
            labels=angle_labels,
            rows=2, cols=3,
            default_value="0.0",
            validator=QDoubleValidator()
        )
        layout.addWidget(self.angle_grid)
        self.angle_vars = self.angle_grid.inputs  # 保存输入框引用以保持兼容性
        
        # 创建控制行
        control_row = ConfigRow(None)
        
        # 曲线类型选择 - 使用RadioButtonGroup基础组件
        self.curve_type_group = RadioButtonGroup(
            None,
            options=["梯形", "S形"],
            default_option="S形"
        )
        control_row.add_widget(self.curve_type_group)
        
        # 时长输入
        time_layout = QHBoxLayout()
        time_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)
        time_layout.addWidget(QLabel("时长:"))
        self.duration_var = QLineEdit("4.0")
        self.duration_var.setFont(default_font)
        self.duration_var.setMaximumWidth(60)
        time_layout.addWidget(self.duration_var)
        time_layout.addWidget(QLabel("s"))
        control_row.add_widget(QWidget())
        control_row.layout.addLayout(time_layout)
        
        # 发送频率输入
        freq_layout = QHBoxLayout()
        freq_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)
        freq_layout.addWidget(QLabel("发送频率:"))
        self.frequency_var = QLineEdit("0.01")
        self.frequency_var.setFont(default_font)
        self.frequency_var.setMaximumWidth(60)
        freq_layout.addWidget(self.frequency_var)
        freq_layout.addWidget(QLabel("s"))
        control_row.layout.addLayout(freq_layout)
        
        control_row.add_stretch()
        layout.addWidget(control_row)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        
        self.send_button = QPushButton("发送角度")
        self.send_button.setFont(default_font)
        self.send_button.clicked.connect(lambda: self.send_angles_requested.emit({
            'target_angles': self.get_angles(), 
            'curve_type': self.get_curve_type(), 
            'frequency': self.get_frequency(),
            'contour_params': self.get_contour() if self.get_contour else None,
            'run_mode': self.get_run_mode() if self.get_run_mode else 0x01
        }))
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
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.connection_status_changed.connect(self.update_connection_status)
    
    def get_angles(self):
        """获取当前角度值"""
        return self.angle_grid.get_float_values()
    
    def get_curve_type(self):
        """获取曲线类型和时间参数"""
        try:
            duration = float(self.duration_var.text())
            frequency = float(self.frequency_var.text())
            curve_type = "trapezoidal" if self.curve_type_group.get_selected() == "梯形" else "s_curve"
            return curve_type, duration, frequency
        except ValueError:
            return "trapezoidal", 4.0, 0.1
    
    def get_frequency(self):
        """获取当前的发送频率"""
        try:
            return float(self.frequency_var.text())
        except ValueError:
            return 0.01

    def set_angles(self, angles):
        """设置角度值"""
        angle_strings = [f"{angle:.6f}" for angle in angles[:6]]
        self.angle_grid.set_values(angle_strings)
        
    def update_connection_status(self, connected):
        """更新连接状态"""
        self.send_button.setEnabled(connected)
    
    def convert_angles(self):
        """度数转弧度"""
        angle_values = self.angle_grid.get_float_values()
        
        # 将角度转换为弧度 (角度 * pi / 180)
        radian_values = [angle * (pi / 180) for angle in angle_values]
        
        # 更新输入框显示弧度值
        radian_strings = [f"{radian:.4f}" for radian in radian_values]
        self.angle_grid.set_values(radian_strings)
    
    def zero_angles(self):
        """归零处理"""
        # 将所有角度值设置为0
        zero_angles = [0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]
        zero_strings = [f"{angle:.4f}" for angle in zero_angles]
        self.angle_grid.set_values(zero_strings) 