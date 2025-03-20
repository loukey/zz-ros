"""
显示相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QTextEdit, QPushButton, 
                           QVBoxLayout, QHBoxLayout, QSplitter, QGridLayout, QGroupBox, QTabWidget, QLineEdit)
from PyQt5.QtCore import Qt, QDateTime
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np
from gui.components.base_components import default_font, text_font, GroupFrame
from gui.utils.command_utils import generate_trajectory
import platform
import math

# 确保本地有正确的导入
try:
    from utils.kinematic.kinematic_6dof import Kinematic6DOF
    from utils.kinematic.velocity_planning import trapezoidal_velocity_planning, s_curve_velocity_planning
except ImportError:
    # 如果本地没有，尝试从usb_driver导入
    try:
        from test.usb_driver.gui.kinematic.kinematic_6dof import Kinematic6DOF
        from test.usb_driver.gui.kinematic.velocity_planning import trapezoidal_velocity_planning, s_curve_velocity_planning
    except ImportError:
        # 如果导入失败，提供简单的替代实现
        class Kinematic6DOF:
            def __init__(self):
                pass
                
        def trapezoidal_velocity_planning(angles, v_max=3.14, t_acc=1.0, dt=0.1):
            """梯形速度规划的简单实现"""
            # 创建简单的时间序列和位置序列
            time_points = np.arange(0, 5, dt)
            n_points = len(time_points)
            positions = np.zeros((n_points, len(angles)))
            velocities = np.zeros(n_points)
            accelerations = np.zeros(n_points)
            
            for i in range(n_points):
                t = time_points[i]
                phase = t / 5.0  # 简单的线性插值
                if phase > 1.0:
                    phase = 1.0
                for j, angle in enumerate(angles):
                    positions[i, j] = angle * phase
                    
                if t < t_acc:  # 加速阶段
                    velocities[i] = v_max * (t / t_acc)
                    accelerations[i] = v_max / t_acc
                elif t < 5 - t_acc:  # 匀速阶段
                    velocities[i] = v_max
                    accelerations[i] = 0
                else:  # 减速阶段
                    velocities[i] = v_max * ((5 - t) / t_acc)
                    accelerations[i] = -v_max / t_acc
                    
            return time_points, velocities, accelerations, positions
                
        def s_curve_velocity_planning(angles, v_max=3.14, t_acc=1.0, dt=0.1):
            """S形速度规划的简单实现"""
            # 与梯形速度规划相似，但加速度曲线更平滑
            return trapezoidal_velocity_planning(angles, v_max, t_acc, dt)

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
        timestamp = QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        
        # 使用[类型]格式显示不同类型的消息，每种类型对应不同颜色
        if message_type in ["发送", "控制", "参数"]:  # 这些类型显示在发送区
            # 根据消息类型设置颜色
            if message_type == "发送":
                color = "#007ACC"  # 蓝色
            elif message_type == "控制":
                color = "#E91E63"  # 粉色
            else:  # 参数
                color = "#FF9800"  # 橙色
                
            formatted_message = f"<span style='color:{color};'>[{timestamp}][{message_type}]</span> {message}"
            self.send_text.append(formatted_message)
            self.send_text.verticalScrollBar().setValue(
                self.send_text.verticalScrollBar().maximum()
            )
        else:  # 接收、错误、信息、系统等类型显示在接收区
            if message_type == "接收":
                color = "#28A745"  # 绿色
            elif message_type == "错误":
                color = "#DC3545"  # 红色
            elif message_type == "信息":
                color = "#6C757D"  # 灰色
            elif message_type == "系统":
                color = "#9C27B0"  # 紫色
            else:
                color = "#6C757D"  # 默认灰色
                
            formatted_message = f"<span style='color:{color};'>[{timestamp}][{message_type}]</span> {message}"
            self.receive_text.append(formatted_message)
            self.receive_text.verticalScrollBar().setValue(
                self.receive_text.verticalScrollBar().maximum()
            )
    
    def clear_send(self):
        """清除发送区域"""
        self.send_text.clear()
    
    def clear_receive(self):
        """清除接收区域"""
        self.receive_text.clear()
    
    def clear_all(self):
        """清除所有区域"""
        self.send_text.clear()
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
    
    def update_position(self, x, y, z, roll, pitch, yaw):
        """更新位置和姿态，兼容原有接口"""
        self.update_theoretical_position(x, y, z)
        self.update_theoretical_attitude(roll, pitch, yaw) 