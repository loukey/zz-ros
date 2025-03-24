"""
运动学相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QLineEdit, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox, QTabWidget)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from gui.components.base_components import default_font, GroupFrame
from math import pi, degrees, radians
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np
import platform
import math
from kinematic import Kinematic6DOF, trapezoidal_velocity_planning, s_curve_velocity_planning


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


class InverseKinematicFrame(QWidget):
    """逆运动学计算框架"""
    
    def __init__(self, parent=None, inverse_callback=None):
        super().__init__(parent)
        self.inverse_callback = inverse_callback
        self.apply_callback = None
        self.calculated_angles = None
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        # 创建主布局
        layout = QVBoxLayout()
        
        # 创建输入参数组
        input_group = QGroupBox("末端位置与姿态")
        input_layout = QGridLayout()
        
        # 位置输入
        input_layout.addWidget(QLabel("X 位置"), 0, 0)
        input_layout.addWidget(QLabel("Y 位置"), 0, 1)
        input_layout.addWidget(QLabel("Z 位置"), 0, 2)
        
        self.position_inputs = []
        for i in range(3):
            input_edit = QLineEdit("0.0")
            input_edit.setValidator(QDoubleValidator())
            input_edit.setFont(default_font)
            self.position_inputs.append(input_edit)
            input_layout.addWidget(input_edit, 1, i)
        
        # 姿态输入
        input_layout.addWidget(QLabel("A 角度"), 2, 0)
        input_layout.addWidget(QLabel("B 角度"), 2, 1)
        input_layout.addWidget(QLabel("C 角度"), 2, 2)
        
        self.euler_inputs = []
        for i in range(3):
            input_edit = QLineEdit("0.0")
            input_edit.setValidator(QDoubleValidator())
            input_edit.setFont(default_font)
            self.euler_inputs.append(input_edit)
            input_layout.addWidget(input_edit, 3, i)
        
        # 添加计算按钮
        self.calculate_button = QPushButton("计算逆运动学")
        self.calculate_button.setFont(default_font)
        self.calculate_button.clicked.connect(self.calculate_inverse_kinematics)
        input_layout.addWidget(self.calculate_button, 4, 0, 1, 3)
        
        input_group.setLayout(input_layout)
        layout.addWidget(input_group)
        
        # 创建结果显示组
        result_group = QGroupBox("计算结果")
        result_layout = QVBoxLayout()
        
        self.result_label = QLabel("请输入位置和姿态，然后点击计算")
        self.result_label.setFont(default_font)
        result_layout.addWidget(self.result_label)
        
        # 添加结果显示区域
        self.result_layout = QGridLayout()
        for i in range(6):
            col = i % 3
            row = i // 3
            self.result_layout.addWidget(QLabel(f"关节{i+1}:"), row, col*2)
            
            result_label = QLabel("0.0000")
            result_label.setFont(default_font)
            self.result_layout.addWidget(result_label, row, col*2+1)
            
        result_layout.addLayout(self.result_layout)
        
        # 添加应用按钮
        self.apply_button = QPushButton("应用到角度控制")
        self.apply_button.setFont(default_font)
        self.apply_button.setEnabled(False)
        self.apply_button.clicked.connect(self.apply_result)
        result_layout.addWidget(self.apply_button)
        
        result_group.setLayout(result_layout)
        layout.addWidget(result_group)
        
        # 设置布局
        self.setLayout(layout)
    
    def calculate_inverse_kinematics(self):
        """
        计算逆运动学
        """
        try:
            # 获取输入值
            position = [float(self.position_inputs[i].text()) for i in range(3)]
            euler = [float(self.euler_inputs[i].text()) for i in range(3)]
            
            # 调用回调函数计算逆运动学
            if self.inverse_callback:
                self.calculated_angles = self.inverse_callback(
                    position[0], position[1], position[2], 
                    euler[0], euler[1], euler[2]
                )
                
                if self.calculated_angles:
                    # 更新结果显示
                    self.update_result(self.calculated_angles)
                    self.apply_button.setEnabled(True)
                else:
                    self.apply_button.setEnabled(False)
                    self.result_label.setText("计算失败，请检查输入值")
            else:
                self.apply_button.setEnabled(False)
                self.result_label.setText("未配置逆运动学计算函数")
        except ValueError:
            self.apply_button.setEnabled(False)
            self.result_label.setText("请输入有效的数值")
    
    def update_result(self, angles):
        """
        更新计算结果显示
        
        参数:
            angles: 关节角度列表
        """
        # 更新结果标签
        self.result_label.setText("计算完成，得到的关节角度 (弧度)：")
        
        # 更新各关节角度值显示
        for i, angle in enumerate(angles):
            if i < 6:
                col = i % 3
                row = i // 3
                widget = self.result_layout.itemAtPosition(row, col*2+1).widget()
                if widget:
                    widget.setText(f"{angle:.6f}")
    
    def set_apply_callback(self, callback):
        """
        设置应用结果的回调函数
        
        参数:
            callback: 应用结果的回调函数
        """
        self.apply_callback = callback
    
    def apply_result(self):
        """应用计算结果"""
        if self.calculated_angles and self.apply_callback:
            self.apply_callback(self.calculated_angles)
    
    def get_result_angles(self):
        """
        获取计算结果
        
        返回:
            angles: 关节角度列表
        """
        return self.calculated_angles


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
        
        for widget in [self.theoretical_x, self.theoretical_y, self.theoretical_z]:
            widget.setReadOnly(True)
            
        layout.addWidget(QLabel("X:"), 0, 1)
        layout.addWidget(self.theoretical_x, 0, 2)
        layout.addWidget(QLabel("Y:"), 0, 3)
        layout.addWidget(self.theoretical_y, 0, 4)
        layout.addWidget(QLabel("Z:"), 0, 5)
        layout.addWidget(self.theoretical_z, 0, 6)
        
        # 实际位置
        layout.addWidget(QLabel("实际位置:"), 1, 0)
        self.actual_x = QLineEdit()
        self.actual_y = QLineEdit()
        self.actual_z = QLineEdit()
        
        for widget in [self.actual_x, self.actual_y, self.actual_z]:
            widget.setReadOnly(True)
            
        layout.addWidget(QLabel("X:"), 1, 1)
        layout.addWidget(self.actual_x, 1, 2)
        layout.addWidget(QLabel("Y:"), 1, 3)
        layout.addWidget(self.actual_y, 1, 4)
        layout.addWidget(QLabel("Z:"), 1, 5)
        layout.addWidget(self.actual_z, 1, 6)
        
        # 理论姿态
        layout.addWidget(QLabel("理论姿态:"), 2, 0)
        self.theoretical_roll = QLineEdit()
        self.theoretical_pitch = QLineEdit()
        self.theoretical_yaw = QLineEdit()
        
        for widget in [self.theoretical_roll, self.theoretical_pitch, self.theoretical_yaw]:
            widget.setReadOnly(True)
            
        layout.addWidget(QLabel("Roll:"), 2, 1)
        layout.addWidget(self.theoretical_roll, 2, 2)
        layout.addWidget(QLabel("Pitch:"), 2, 3)
        layout.addWidget(self.theoretical_pitch, 2, 4)
        layout.addWidget(QLabel("Yaw:"), 2, 5)
        layout.addWidget(self.theoretical_yaw, 2, 6)
        
        # 实际姿态
        layout.addWidget(QLabel("实际姿态:"), 3, 0)
        self.actual_roll = QLineEdit()
        self.actual_pitch = QLineEdit()
        self.actual_yaw = QLineEdit()
        
        for widget in [self.actual_roll, self.actual_pitch, self.actual_yaw]:
            widget.setReadOnly(True)
            
        layout.addWidget(QLabel("Roll:"), 3, 1)
        layout.addWidget(self.actual_roll, 3, 2)
        layout.addWidget(QLabel("Pitch:"), 3, 3)
        layout.addWidget(self.actual_pitch, 3, 4)
        layout.addWidget(QLabel("Yaw:"), 3, 5)
        layout.addWidget(self.actual_yaw, 3, 6)
        
        self.setLayout(layout)
    
    def update_theoretical_position(self, x, y, z):
        """更新理论位置"""
        self.theoretical_x.setText(f"{x:.6f}")
        self.theoretical_y.setText(f"{y:.6f}")
        self.theoretical_z.setText(f"{z:.6f}")
    
    def update_actual_position(self, x, y, z):
        """更新实际位置"""
        self.actual_x.setText(f"{x:.6f}")
        self.actual_y.setText(f"{y:.6f}")
        self.actual_z.setText(f"{z:.6f}")
    
    def update_theoretical_attitude(self, roll, pitch, yaw):
        """更新理论姿态"""
        self.theoretical_roll.setText(f"{roll:.6f}")
        self.theoretical_pitch.setText(f"{pitch:.6f}")
        self.theoretical_yaw.setText(f"{yaw:.6f}")
    
    def update_actual_attitude(self, roll, pitch, yaw):
        """更新实际姿态"""
        self.actual_roll.setText(f"{roll:.6f}")
        self.actual_pitch.setText(f"{pitch:.6f}")
        self.actual_yaw.setText(f"{yaw:.6f}")
    
    def update_position(self, x, y, z, roll, pitch, yaw):
        """更新所有位置和姿态"""
        self.update_theoretical_position(x, y, z)
        self.update_theoretical_attitude(roll, pitch, yaw) 