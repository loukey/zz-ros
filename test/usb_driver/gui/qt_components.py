"""
PyQt版本的UI组件模块
包含各种UI元素的创建和管理
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                           QComboBox, QPushButton, QFrame, QGroupBox, 
                           QTextEdit, QRadioButton, QButtonGroup, QLineEdit,
                           QTabWidget, QGridLayout, QScrollArea, QMessageBox)
from PyQt5.QtCore import Qt, QDateTime
from PyQt5.QtGui import QFont
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np
import platform
import math
import matplotlib
from .kinematic.velocity_planning import trapezoidal_velocity_planning, s_curve_velocity_planning
from .kinematic.kinematic_6dof import Kinematic6DOF
matplotlib.use('Qt5Agg')

# 设置matplotlib支持中文显示
if platform.system() == 'Windows':
    plt.rcParams['font.family'] = ['Microsoft YaHei']
else:
    plt.rcParams['font.family'] = ['WenQuanYi Micro Hei']
    try:
        plt.rcParams['font.family'] = ['WenQuanYi Micro Hei']
    except:
        print("警告：未找到文泉驿微米黑字体，将使用系统默认字体")
        plt.rcParams['font.family'] = ['sans-serif']

# 设置matplotlib全局字体大小
plt.rcParams['font.size'] = 12
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['xtick.labelsize'] = 10
plt.rcParams['ytick.labelsize'] = 10
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 设置全局字体
default_font = QFont('WenQuanYi Micro Hei', 12)
text_font = QFont('WenQuanYi Micro Hei', 11)


class PortSelectionFrame(QWidget):
    """串口选择框架"""
    
    def __init__(self, parent=None, refresh_callback=None):
        super().__init__(parent)
        self.refresh_callback = refresh_callback
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QHBoxLayout()
        
        # 创建串口选择下拉列表
        layout.addWidget(QLabel("选择串口:"))
        self.port_combobox = QComboBox()
        self.port_combobox.setFont(default_font)
        self.port_combobox.setMinimumWidth(300)
        layout.addWidget(self.port_combobox)
        
        # 创建刷新按钮
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.setFont(default_font)
        self.refresh_button.clicked.connect(self.refresh_callback)
        layout.addWidget(self.refresh_button)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def set_ports(self, port_list):
        """
        设置可用串口列表
        
        参数:
            port_list: 包含(port, description)元组的列表
        """
        self.port_combobox.clear()
        formatted_ports = []
        for port, desc in port_list:
            formatted_ports.append(f"{port} - {desc}")
        
        self.port_combobox.addItems(formatted_ports)
        
        # 如果有可用串口，默认选择第一个
        if formatted_ports:
            self.port_combobox.setCurrentIndex(0)
        else:
            self.port_combobox.setCurrentText("未找到可用串口")
    
    def get_selected_port(self):
        """
        获取当前选择的串口设备名称
        
        返回:
            port: 串口设备名称，如果未选择则返回None
        """
        selection = self.port_combobox.currentText()
        if selection and " - " in selection:
            return selection.split(" - ")[0]
        return None


class SerialConfigFrame(QGroupBox):
    """串口参数配置框架"""
    
    def __init__(self, parent=None, connect_callback=None):
        super().__init__("串口通信参数配置", parent)
        self.connect_callback = connect_callback
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 常用波特率列表
        self.baud_rates = ['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600']
        # 数据位选项
        self.data_bits = ['5', '6', '7', '8']
        # 校验位选项
        self.parity_bits = [('无', 'N'), ('奇校验', 'O'), ('偶校验', 'E'), ('标记', 'M'), ('空格', 'S')]
        # 停止位选项
        self.stop_bits = [('1', 1), ('1.5', 1.5), ('2', 2)]
        # 流控制选项
        self.flow_controls = [('无', ''), ('XON/XOFF (软件)', 'xonxoff'), 
                            ('RTS/CTS (硬件)', 'rtscts'), ('DSR/DTR (硬件)', 'dsrdtr')]
        
        # 第一行：波特率和数据位
        row1_layout = QHBoxLayout()
        row1_layout.addWidget(QLabel("波特率:"))
        self.baud_rate = QComboBox()
        self.baud_rate.addItems(self.baud_rates)
        self.baud_rate.setCurrentText('115200')
        self.baud_rate.setFont(default_font)
        row1_layout.addWidget(self.baud_rate)
        
        row1_layout.addWidget(QLabel("数据位:"))
        self.data_bit = QComboBox()
        self.data_bit.addItems(self.data_bits)
        self.data_bit.setCurrentText('8')
        self.data_bit.setFont(default_font)
        row1_layout.addWidget(self.data_bit)
        row1_layout.addStretch()
        layout.addLayout(row1_layout)
        
        # 第二行：校验位
        row2_layout = QHBoxLayout()
        row2_layout.addWidget(QLabel("校验位:"))
        self.parity_group = QButtonGroup()
        for i, (text, value) in enumerate(self.parity_bits):
            radio = QRadioButton(text)
            radio.setFont(default_font)
            radio.setProperty('parity_value', value)  # 存储实际的校验位值
            if value == 'N':
                radio.setChecked(True)
            self.parity_group.addButton(radio, i)
            row2_layout.addWidget(radio)
        row2_layout.addStretch()
        layout.addLayout(row2_layout)
        
        # 第三行：停止位
        row3_layout = QHBoxLayout()
        row3_layout.addWidget(QLabel("停止位:"))
        self.stop_bit_group = QButtonGroup()
        for text, value in self.stop_bits:
            radio = QRadioButton(text)
            radio.setFont(default_font)
            if value == 1:
                radio.setChecked(True)
            self.stop_bit_group.addButton(radio)
            row3_layout.addWidget(radio)
        row3_layout.addStretch()
        layout.addLayout(row3_layout)
        
        # 第四行：流控制
        row4_layout = QHBoxLayout()
        row4_layout.addWidget(QLabel("流控制:"))
        self.flow_control_group = QButtonGroup()
        for text, value in self.flow_controls:
            radio = QRadioButton(text)
            radio.setFont(default_font)
            if value == '':
                radio.setChecked(True)
            self.flow_control_group.addButton(radio)
            row4_layout.addWidget(radio)
        row4_layout.addStretch()
        layout.addLayout(row4_layout)
        
        # 连接按钮
        self.connect_button = QPushButton("连接串口")
        self.connect_button.setFont(default_font)
        self.connect_button.clicked.connect(self.connect_callback)
        layout.addWidget(self.connect_button)
        
        self.setLayout(layout)
    
    def get_config(self):
        """
        获取当前配置参数
        
        返回:
            config: 包含所有配置参数的字典
        """
        return {
            'baud_rate': int(self.baud_rate.currentText()),
            'data_bits': int(self.data_bit.currentText()),
            'parity': self.parity_group.checkedButton().property('parity_value'),  # 获取实际的校验位值
            'stop_bits': float(self.stop_bit_group.checkedButton().text()),
            'flow_control': self.flow_control_group.checkedButton().text()
        }
    
    def set_connect_button_state(self, is_connected):
        """
        设置连接按钮状态
        
        参数:
            is_connected: 是否已连接
        """
        self.connect_button.setText("断开连接" if is_connected else "连接串口")


class ControlButtonsFrame(QGroupBox):
    """控制按钮框架"""
    
    def __init__(self, parent=None, send_command_callback=None):
        super().__init__("参数配置和控制命令", parent)
        self.send_command_callback = send_command_callback
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
        self.encoding_combo.addItems(['string', 'hex'])
        self.encoding_combo.setFont(default_font)
        encoding_layout.addWidget(self.encoding_combo)
        encoding_layout.addWidget(QLabel("(string: 字符串格式, hex: 十六进制格式)"))
        encoding_layout.addStretch()
        config_layout.addLayout(encoding_layout)
        
        # 运行模式选择
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("运行模式:"))
        self.run_modes = [
            ('轮廓位置模式', '01'),
            ('轮廓速度模式', '03'),
            ('轮廓扭矩模式', '04'),
            ('回零模式(暂不支持)', '06'),
            ('位置插补模式(暂不支持)', '07'),
            ('周期同步位置模式', '08'),
            ('周期同步速度模式', '09'),
            ('周期同步扭矩模式', '0A')
        ]
        self.run_mode_combo = QComboBox()
        self.run_mode_combo.addItems([f"{mode[0]} ({mode[1]})" for mode in self.run_modes])
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
            ("使能", "ENABLE"),
            ("取消使能", "DISABLE"),
            ("释放刹车", "RELEASE"),
            ("锁止刹车", "LOCK"),
            ("立刻停止", "STOP"),
            ("运动状态", "MOTION")
        ]
        
        for text, command in commands:
            button = QPushButton(text)
            button.setFont(default_font)
            button.clicked.connect(lambda checked, cmd=command: self.send_command_callback(cmd))
            button_layout.addWidget(button)
            self.buttons.append(button)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def set_buttons_state(self, is_connected):
        """
        设置按钮状态
        
        参数:
            is_connected: 是否已连接
        """
        for button in self.buttons:
            button.setEnabled(is_connected)
    
    def get_encoding_type(self):
        """获取当前选择的编码格式"""
        return self.encoding_combo.currentText()
    
    def get_run_mode(self):
        """获取当前选择的运行模式"""
        selected = self.run_mode_combo.currentText()
        for mode_name, mode_code in self.run_modes:
            if mode_name in selected:
                return mode_code
        return '01'  # 默认返回轮廓位置模式


class AngleControlFrame(QGroupBox):
    """角度控制框架"""
    
    def __init__(self, parent=None, send_callback=None, convert_callback=None, zero_callback=None):
        super().__init__("角度控制 (弧度值)", parent)
        self.send_callback = send_callback
        self.convert_callback = convert_callback
        self.zero_callback = zero_callback
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
        self.trapezoidal.setChecked(True)
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
        self.frequency_var = QLineEdit("0.1")
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
        self.send_button.clicked.connect(self.send_callback)
        self.send_button.setEnabled(False)
        button_layout.addWidget(self.send_button)
        
        convert_button = QPushButton("度数转弧度")
        convert_button.setFont(default_font)
        convert_button.clicked.connect(self.convert_callback)
        button_layout.addWidget(convert_button)
        
        zero_button = QPushButton("全部归零")
        zero_button.setFont(default_font)
        zero_button.clicked.connect(self.zero_callback)
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
    
    def set_angles(self, angles):
        """
        设置角度值
        
        参数:
            angles: 角度值列表
        """
        for i, angle in enumerate(angles):
            if i < len(self.angle_vars):
                self.angle_vars[i].setText(f"{angle:.6f}")
    
    def set_send_button_state(self, is_connected):
        """
        设置发送按钮状态
        
        参数:
            is_connected: 是否已连接
        """
        self.send_button.setEnabled(is_connected)


class DataDisplayFrame(QGroupBox):
    """数据显示区域"""
    
    def __init__(self, parent=None):
        super().__init__("数据显示", parent)
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 创建发送区
        send_group = QGroupBox("发送区")
        send_layout = QVBoxLayout()
        self.send_text = QTextEdit()
        self.send_text.setReadOnly(True)
        send_layout.addWidget(self.send_text)
        send_group.setLayout(send_layout)
        
        # 创建接收区
        receive_group = QGroupBox("接收区")
        receive_layout = QVBoxLayout()
        self.receive_text = QTextEdit()
        self.receive_text.setReadOnly(True)
        receive_layout.addWidget(self.receive_text)
        receive_group.setLayout(receive_layout)
        
        # 创建按钮区
        button_layout = QHBoxLayout()
        self.clear_send_btn = QPushButton("清除发送区")
        self.clear_receive_btn = QPushButton("清除接收区")
        self.clear_all_btn = QPushButton("清除所有")
        
        button_layout.addWidget(self.clear_send_btn)
        button_layout.addWidget(self.clear_receive_btn)
        button_layout.addWidget(self.clear_all_btn)
        
        # 添加所有组件到主布局
        layout.addWidget(send_group)
        layout.addWidget(receive_group)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def append_message(self, message, message_type="接收"):
        """添加消息到显示区"""
        if message_type == "发送":
            self.send_text.append(message)
            self.send_text.verticalScrollBar().setValue(
                self.send_text.verticalScrollBar().maximum()
            )
        elif message_type == "接收":
            self.receive_text.append(message)
            self.receive_text.verticalScrollBar().setValue(
                self.receive_text.verticalScrollBar().maximum()
            )
        elif message_type == "错误":
            self.receive_text.append(f"错误: {message}")
            self.receive_text.verticalScrollBar().setValue(
                self.receive_text.verticalScrollBar().maximum()
            )
        elif message_type == "信息":
            self.receive_text.append(f"信息: {message}")
            self.receive_text.verticalScrollBar().setValue(
                self.receive_text.verticalScrollBar().maximum()
            )
        elif message_type == "参数":
            self.receive_text.append(f"参数: {message}")
            self.receive_text.verticalScrollBar().setValue(
                self.receive_text.verticalScrollBar().maximum()
            )
    
    def clear_display(self, display_type="all"):
        """清除显示区"""
        if display_type in ["send", "all"]:
            self.send_text.clear()
        if display_type in ["receive", "all"]:
            self.receive_text.clear()


class CurvePlotFrame(QGroupBox):
    """曲线显示区域"""
    
    def __init__(self, parent=None):
        super().__init__("曲线显示", parent)
        self._init_ui()
        self.kinematics = Kinematic6DOF()  # 创建运动学对象
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 创建标签页控件
        self.tab_widget = QTabWidget()
        
        # 创建位置曲线标签页
        self.position_frame = QWidget()
        self.position_layout = QVBoxLayout()
        self.position_fig, self.position_ax = plt.subplots(figsize=(10, 6), dpi=100)
        self.position_canvas = FigureCanvas(self.position_fig)
        self.position_layout.addWidget(self.position_canvas)
        self.position_frame.setLayout(self.position_layout)
        self.tab_widget.addTab(self.position_frame, "位置曲线")
        
        # 创建速度曲线标签页
        self.velocity_frame = QWidget()
        self.velocity_layout = QVBoxLayout()
        self.velocity_fig, self.velocity_ax = plt.subplots(figsize=(10, 6), dpi=100)
        self.velocity_canvas = FigureCanvas(self.velocity_fig)
        self.velocity_layout.addWidget(self.velocity_canvas)
        self.velocity_frame.setLayout(self.velocity_layout)
        self.tab_widget.addTab(self.velocity_frame, "速度曲线")
        
        # 创建加速度曲线标签页
        self.acceleration_frame = QWidget()
        self.acceleration_layout = QVBoxLayout()
        self.acceleration_fig, self.acceleration_ax = plt.subplots(figsize=(10, 6), dpi=100)
        self.acceleration_canvas = FigureCanvas(self.acceleration_fig)
        self.acceleration_layout.addWidget(self.acceleration_canvas)
        self.acceleration_frame.setLayout(self.acceleration_layout)
        self.tab_widget.addTab(self.acceleration_frame, "加速度曲线")
        
        layout.addWidget(self.tab_widget)
        self.setLayout(layout)
    
    def plot_curves(self, angles, curve_type="trapezoidal", duration=4.0, frequency=0.1):
        """绘制曲线"""
        # 根据曲线类型选择规划函数
        if curve_type == "trapezoidal":
            times, velocities, accelerations, positions = trapezoidal_velocity_planning(
                angles, v_max=math.pi, t_acc=duration/4, dt=frequency
            )
        else:  # s_curve
            times, velocities, accelerations, positions = s_curve_velocity_planning(
                angles, v_max=math.pi, t_acc=duration/4, dt=frequency
            )
        
        # 清除现有曲线
        self.position_ax.clear()
        self.velocity_ax.clear()
        self.acceleration_ax.clear()
        
        # 绘制位置曲线
        for i in range(positions.shape[1]):
            self.position_ax.plot(times, positions[:, i], label=f'关节{i+1}')
        self.position_ax.set_title('位置曲线')
        self.position_ax.set_xlabel('时间 (s)')
        self.position_ax.set_ylabel('位置 (rad)')
        self.position_ax.grid(True)
        self.position_ax.legend()
        
        # 绘制速度曲线
        self.velocity_ax.plot(times, velocities, 'r-', label='速度')
        self.velocity_ax.set_title('速度曲线')
        self.velocity_ax.set_xlabel('时间 (s)')
        self.velocity_ax.set_ylabel('速度 (rad/s)')
        self.velocity_ax.grid(True)
        self.velocity_ax.legend()
        
        # 绘制加速度曲线
        self.acceleration_ax.plot(times, accelerations, 'g-', label='加速度')
        self.acceleration_ax.set_title('加速度曲线')
        self.acceleration_ax.set_xlabel('时间 (s)')
        self.acceleration_ax.set_ylabel('加速度 (rad/s²)')
        self.acceleration_ax.grid(True)
        self.acceleration_ax.legend()
        
        # 调整布局
        self.position_fig.tight_layout()
        self.velocity_fig.tight_layout()
        self.acceleration_fig.tight_layout()
        
        # 更新画布
        self.position_canvas.draw()
        self.velocity_canvas.draw()
        self.acceleration_canvas.draw()
        
        # 切换到速度曲线标签页
        self.tab_widget.setCurrentIndex(1)


class InverseKinematicFrame(QWidget):
    """逆运动学输入框架"""
    
    def __init__(self, parent=None, inverse_callback=None):
        super().__init__(parent)
        self.inverse_callback = inverse_callback
        self.kinematics = Kinematic6DOF()  # 创建运动学对象
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 创建输入区域
        input_group = QGroupBox("输入参数")
        input_layout = QGridLayout()
        
        # 位置输入
        input_layout.addWidget(QLabel("X 位置"), 0, 0)
        input_layout.addWidget(QLabel("Y 位置"), 0, 1)
        input_layout.addWidget(QLabel("Z 位置"), 0, 2)
        
        self.position_vars = []
        for i in range(3):
            var = QLineEdit("0.0")
            var.setFont(default_font)
            self.position_vars.append(var)
            input_layout.addWidget(var, 1, i)
        
        # 欧拉角输入
        input_layout.addWidget(QLabel("A 角度"), 2, 0)
        input_layout.addWidget(QLabel("B 角度"), 2, 1)
        input_layout.addWidget(QLabel("C 角度"), 2, 2)
        
        self.euler_vars = []
        for i in range(3):
            var = QLineEdit("0.0")
            var.setFont(default_font)
            self.euler_vars.append(var)
            input_layout.addWidget(var, 3, i)
        
        # 计算按钮
        self.calculate_button = QPushButton("计算逆运动学")
        self.calculate_button.setFont(default_font)
        self.calculate_button.clicked.connect(self.calculate_inverse_kinematics)
        input_layout.addWidget(self.calculate_button, 4, 0, 1, 3)
        
        input_group.setLayout(input_layout)
        layout.addWidget(input_group)
        
        # 创建结果显示区域
        result_group = QGroupBox("计算结果")
        result_layout = QVBoxLayout()
        
        self.result_label = QLabel("请输入位置和欧拉角后点击计算")
        self.result_label.setFont(default_font)
        result_layout.addWidget(self.result_label)
        
        self.result_values_row1 = QLabel("")
        self.result_values_row1.setFont(default_font)
        result_layout.addWidget(self.result_values_row1)
        
        self.result_values_row2 = QLabel("")
        self.result_values_row2.setFont(default_font)
        result_layout.addWidget(self.result_values_row2)
        
        self.apply_button = QPushButton("应用到角度控制")
        self.apply_button.setFont(default_font)
        self.apply_button.setEnabled(False)
        result_layout.addWidget(self.apply_button)
        
        result_group.setLayout(result_layout)
        layout.addWidget(result_group)
        
        self.setLayout(layout)
    
    def calculate_inverse_kinematics(self):
        """计算逆运动学"""
        try:
            # 获取位置和欧拉角输入
            position, euler = self.get_position_and_euler()
            
            # 调用逆运动学求解
            angles = self.kinematics.inverse_kinematic(
                euler[0], euler[1], euler[2],  # A, B, C
                position[0], position[1], position[2]  # X, Y, Z
            )
            
            # 显示结果
            self.set_result(angles, True)
            
        except Exception as e:
            QMessageBox.warning(self, "错误", f"计算逆运动学失败: {str(e)}")
            self.set_result(None, False)
    
    def get_position_and_euler(self):
        """
        获取当前位置和欧拉角值
        
        返回:
            position: 位置值列表 [X, Y, Z]
            euler: 欧拉角值列表 [A, B, C]
        """
        try:
            position = [float(var.text()) for var in self.position_vars]
            euler = [float(var.text()) for var in self.euler_vars]
            return position, euler
        except ValueError:
            return [0.0] * 3, [0.0] * 3
    
    def set_result(self, angles, success=True):
        """
        设置计算结果
        
        参数:
            angles: 计算得到的关节角度列表
            success: 计算是否成功
        """
        if success and angles is not None:
            self.result_label.setText("计算成功，关节角度值(弧度):")
            
            # 将角度分为两行显示
            if len(angles) >= 6:
                # 第一行显示角度1-3
                row1_text = f"角度1: {angles[0]:.6f}, 角度2: {angles[1]:.6f}, 角度3: {angles[2]:.6f}"
                # 第二行显示角度4-6
                row2_text = f"角度4: {angles[3]:.6f}, 角度5: {angles[4]:.6f}, 角度6: {angles[5]:.6f}"
                
                self.result_values_row1.setText(row1_text)
                self.result_values_row2.setText(row2_text)
            else:
                # 如果角度数量不足6个，则全部显示在第一行
                self.result_values_row1.setText(", ".join([f"{angle:.6f}" for angle in angles]))
                self.result_values_row2.setText("")
            
            self.apply_button.setEnabled(True)
            # 存储计算结果以便应用按钮使用
            self.calculated_angles = angles
        else:
            self.result_label.setText("计算失败，请检查输入值")
            self.result_values_row1.setText("")
            self.result_values_row2.setText("")
            self.apply_button.setEnabled(False)
    
    def set_apply_callback(self, callback):
        """
        设置应用按钮回调函数
        
        参数:
            callback: 应用按钮回调函数
        """
        self.apply_button.clicked.connect(lambda: callback(self.calculated_angles))


class EndPositionFrame(QGroupBox):
    """末端姿态显示区域"""
    
    def __init__(self, parent=None):
        super().__init__("末端姿态", parent)
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QGridLayout()
        
        # 理论位置
        layout.addWidget(QLabel("理论位置:"), 0, 0)
        self.theoretical_x = QLineEdit()
        self.theoretical_y = QLineEdit()
        self.theoretical_z = QLineEdit()
        layout.addWidget(self.theoretical_x, 0, 1)
        layout.addWidget(self.theoretical_y, 0, 2)
        layout.addWidget(self.theoretical_z, 0, 3)
        
        # 实际位置
        layout.addWidget(QLabel("实际位置:"), 1, 0)
        self.actual_x = QLineEdit()
        self.actual_y = QLineEdit()
        self.actual_z = QLineEdit()
        layout.addWidget(self.actual_x, 1, 1)
        layout.addWidget(self.actual_y, 1, 2)
        layout.addWidget(self.actual_z, 1, 3)
        
        # 理论姿态
        layout.addWidget(QLabel("理论姿态:"), 2, 0)
        self.theoretical_roll = QLineEdit()
        self.theoretical_pitch = QLineEdit()
        self.theoretical_yaw = QLineEdit()
        layout.addWidget(self.theoretical_roll, 2, 1)
        layout.addWidget(self.theoretical_pitch, 2, 2)
        layout.addWidget(self.theoretical_yaw, 2, 3)
        
        # 实际姿态
        layout.addWidget(QLabel("实际姿态:"), 3, 0)
        self.actual_roll = QLineEdit()
        self.actual_pitch = QLineEdit()
        self.actual_yaw = QLineEdit()
        layout.addWidget(self.actual_roll, 3, 1)
        layout.addWidget(self.actual_pitch, 3, 2)
        layout.addWidget(self.actual_yaw, 3, 3)
        
        # 设置所有输入框为只读
        for widget in [self.theoretical_x, self.theoretical_y, self.theoretical_z,
                      self.actual_x, self.actual_y, self.actual_z,
                      self.theoretical_roll, self.theoretical_pitch, self.theoretical_yaw,
                      self.actual_roll, self.actual_pitch, self.actual_yaw]:
            widget.setReadOnly(True)
        
        self.setLayout(layout)
    
    def update_theoretical_position(self, x, y, z):
        """更新理论位置"""
        self.theoretical_x.setText(f"{x:.2f}")
        self.theoretical_y.setText(f"{y:.2f}")
        self.theoretical_z.setText(f"{z:.2f}")
    
    def update_actual_position(self, x, y, z):
        """更新实际位置"""
        self.actual_x.setText(f"{x:.2f}")
        self.actual_y.setText(f"{y:.2f}")
        self.actual_z.setText(f"{z:.2f}")
    
    def update_theoretical_attitude(self, roll, pitch, yaw):
        """更新理论姿态"""
        self.theoretical_roll.setText(f"{roll:.2f}")
        self.theoretical_pitch.setText(f"{pitch:.2f}")
        self.theoretical_yaw.setText(f"{yaw:.2f}")
    
    def update_actual_attitude(self, roll, pitch, yaw):
        """更新实际姿态"""
        self.actual_roll.setText(f"{roll:.2f}")
        self.actual_pitch.setText(f"{pitch:.2f}")
        self.actual_yaw.setText(f"{yaw:.2f}")
    
    def parse_and_update_actual_position(self, data, kinematics_solver):
        """解析数据并更新实际位置和姿态"""
        try:
            # 这里需要根据实际的数据格式进行解析
            # 示例：假设数据格式为 "X:123.45 Y:234.56 Z:345.67 R:12.34 P:23.45 Y:34.56"
            parts = data.split()
            for part in parts:
                if part.startswith('X:'):
                    x = float(part[2:])
                elif part.startswith('Y:'):
                    y = float(part[2:])
                elif part.startswith('Z:'):
                    z = float(part[2:])
                elif part.startswith('R:'):
                    roll = float(part[2:])
                elif part.startswith('P:'):
                    pitch = float(part[2:])
                elif part.startswith('Y:'):
                    yaw = float(part[2:])
            
            # 更新实际位置和姿态
            self.update_actual_position(x, y, z)
            self.update_actual_attitude(roll, pitch, yaw)
            
            # 如果有运动学求解器，更新理论位置和姿态
            if kinematics_solver:
                # TODO: 使用运动学求解器计算理论位置和姿态
                pass
                
        except Exception as e:
            print(f"解析数据失败: {e}")
            return False
        return True 