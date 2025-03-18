"""
PyQt版本的主窗口模块
"""

from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QMessageBox
from PyQt5.QtCore import Qt, QThread, QTimer
from .qt_components import (PortSelectionFrame, SerialConfigFrame, ControlButtonsFrame,
                          AngleControlFrame, DataDisplayFrame, CurvePlotFrame,
                          InverseKinematicFrame, EndPositionFrame)
from .serial_comm import SerialComm
from .utils import format_command, generate_trajectory
import math


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("机器人控制界面")
        self.setMinimumSize(1200, 800)
        
        # 创建串口通信对象
        self.serial_comm = SerialComm()
        self.serial_comm.data_received.connect(self.handle_received_data)
        self.serial_comm.connection_changed.connect(self.handle_connection_changed)
        self.serial_comm.error_occurred.connect(self.handle_error)
        
        # 创建读取线程
        self.read_thread = QThread()
        self.serial_comm.moveToThread(self.read_thread)
        self.read_thread.started.connect(self.serial_comm.read_data)
        
        self.current_angles = [0.0] * 6  # 当前角度
        self.trajectory_data = None  # 轨迹数据
        self.trajectory_index = 0    # 当前轨迹点索引
        self.trajectory_timer = None # 轨迹发送定时器
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        # 创建中央窗口部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QVBoxLayout()
        
        # 创建串口选择区域
        self.port_frame = PortSelectionFrame(self, self.refresh_ports)
        main_layout.addWidget(self.port_frame)
        
        # 创建串口配置区域
        self.serial_config = SerialConfigFrame(self, self.toggle_connection)
        main_layout.addWidget(self.serial_config)
        
        # 创建控制按钮区域
        self.control_buttons = ControlButtonsFrame(self, self.send_command)
        main_layout.addWidget(self.control_buttons)
        
        # 创建角度控制区域
        self.angle_control = AngleControlFrame(self, self.send_angles, self.convert_angles, self.zero_angles)
        main_layout.addWidget(self.angle_control)
        
        # 创建标签页控件
        tab_widget = QTabWidget()
        
        # 创建数据显示标签页
        data_tab = QWidget()
        data_layout = QVBoxLayout()
        self.data_display = DataDisplayFrame(self)
        # 连接清除按钮的信号
        self.data_display.clear_send_btn.clicked.connect(self.clear_send)
        self.data_display.clear_receive_btn.clicked.connect(self.clear_receive)
        self.data_display.clear_all_btn.clicked.connect(self.clear_all)
        data_layout.addWidget(self.data_display)
        data_tab.setLayout(data_layout)
        tab_widget.addTab(data_tab, "数据显示")
        
        # 创建曲线显示标签页
        curve_tab = QWidget()
        curve_layout = QVBoxLayout()
        self.curve_plot = CurvePlotFrame(self)
        curve_layout.addWidget(self.curve_plot)
        curve_tab.setLayout(curve_layout)
        tab_widget.addTab(curve_tab, "曲线显示")
        
        # 创建末端姿态显示标签页
        end_pos_tab = QWidget()
        end_pos_layout = QVBoxLayout()
        self.end_position = EndPositionFrame(self)
        end_pos_layout.addWidget(self.end_position)
        end_pos_tab.setLayout(end_pos_layout)
        tab_widget.addTab(end_pos_tab, "末端姿态")
        
        # 创建逆运动学标签页
        ik_tab = QWidget()
        ik_layout = QVBoxLayout()
        self.inverse_kinematic = InverseKinematicFrame(self, self.calculate_inverse_kinematics)
        ik_layout.addWidget(self.inverse_kinematic)
        ik_tab.setLayout(ik_layout)
        tab_widget.addTab(ik_tab, "逆运动学")
        
        main_layout.addWidget(tab_widget)
        
        # 设置主布局
        central_widget.setLayout(main_layout)
    
    def refresh_ports(self):
        """刷新串口列表"""
        ports = self.serial_comm.get_available_ports()
        self.port_frame.set_ports(ports)
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.serial_comm.is_connected:
            # 获取选中的串口
            port = self.port_frame.get_selected_port()
            if not port:
                QMessageBox.warning(self, "警告", "请选择串口")
                return
            
            # 获取串口配置
            config = self.serial_config.get_config()
            
            # 连接串口
            if self.serial_comm.connect(
                port=port,
                baud_rate=config['baud_rate'],
                data_bits=config['data_bits'],
                parity=config['parity'],
                stop_bits=config['stop_bits'],
                flow_control=config['flow_control']
            ):
                # 启动读取线程
                self.read_thread.start()
                self.serial_config.set_connect_button_state(True)
                self.control_buttons.set_buttons_state(True)
                self.angle_control.set_send_button_state(True)
                self.data_display.append_message(f"已连接到串口 {port}", "信息")
        else:
            # 断开连接
            self.serial_comm.disconnect()
            self.read_thread.quit()
            self.read_thread.wait()
            self.serial_config.set_connect_button_state(False)
            self.control_buttons.set_buttons_state(False)
            self.angle_control.set_send_button_state(False)
            self.data_display.append_message("已断开串口连接", "信息")
    
    def send_command(self, command):
        """发送控制命令"""
        if self.serial_comm.send_command(command):
            self.data_display.append_message(f"发送命令: {command}", "发送")
        else:
            self.data_display.append_message(f"发送命令失败: {command}", "错误")
    
    def send_angles(self):
        """发送角度值"""
        # 获取角度值
        angles = self.angle_control.get_angles()
        
        # 获取曲线参数
        curve_type, duration, frequency = self.angle_control.get_curve_type()
        
        # 发送角度值
        if self.serial_comm.send_angles(angles, duration, frequency, curve_type):
            self.data_display.append_message(
                f"发送角度: {angles}, 时长: {duration}s, 频率: {frequency}s, 曲线类型: {curve_type}",
                "发送"
            )
        else:
            self.data_display.append_message("发送角度失败", "错误")
    
    def convert_angles(self):
        """转换角度值（度数转弧度）"""
        angles = self.angle_control.get_angles()
        converted_angles = [math.radians(angle) for angle in angles]
        self.angle_control.set_angles(converted_angles)
        self.data_display.append_message(f"角度转换: {angles}° -> {converted_angles}rad", "参数")
    
    def zero_angles(self):
        """将所有角度值归零"""
        self.angle_control.set_angles([0.0] * 6)
        self.data_display.append_message("所有角度值已归零", "参数")
    
    def calculate_inverse_kinematics(self):
        """计算逆运动学"""
        # TODO: 实现逆运动学计算功能
        pass
    
    def clear_send(self):
        """清除发送区显示"""
        self.data_display.clear_display("send")
    
    def clear_receive(self):
        """清除接收区显示"""
        self.data_display.clear_display("receive")
    
    def clear_all(self):
        """清除所有显示"""
        self.data_display.clear_display("all")
    
    def handle_received_data(self, data):
        """处理接收到的数据"""
        self.data_display.append_message(data, "接收")
        # 更新末端姿态显示
        self.end_position.parse_and_update_actual_position(data, None)  # TODO: 添加运动学求解器
    
    def handle_connection_changed(self, is_connected):
        """处理连接状态改变"""
        self.serial_config.set_connect_button_state(is_connected)
        self.control_buttons.set_buttons_state(is_connected)
        self.angle_control.set_send_button_state(is_connected)
    
    def handle_error(self, error_msg):
        """处理错误"""
        self.data_display.append_message(error_msg, "错误")
        QMessageBox.warning(self, "错误", error_msg)
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        if self.serial_comm.is_connected:
            self.serial_comm.disconnect()
            self.read_thread.quit()
            self.read_thread.wait()
        event.accept() 