"""
主窗口视图
"""
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QTabWidget, QMessageBox, QHBoxLayout
from PyQt5.QtCore import QTimer
from components.serial_components import PortSelectionFrame, SerialConfigFrame
from components.control_components import ControlButtonsFrame, AngleControlFrame
from components.display_components import DataDisplayFrame, CurvePlotFrame, EndPositionFrame
from components.kinematic_components import InverseKinematicFrame
from controllers.robot_controller import RobotController
from controllers.serial_controller import SerialController
from math import pi
import math


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("机器人控制界面")
        self.setMinimumSize(1200, 800)
        
        # 创建串口控制器
        self.serial_controller = SerialController()
        
        # 创建机器人控制器，传入串口控制器
        self.robot_controller = RobotController(self.serial_controller)
        
        # 注册回调函数
        self.serial_controller.serial_model.data_received.connect(self.handle_data_received)
        self.serial_controller.serial_model.connection_changed.connect(self.handle_connection_changed)
        self.serial_controller.serial_model.error_occurred.connect(self.handle_error_occurred)
        
        # 创建定时器以定期更新末端位置显示
        self.position_timer = QTimer()
        self.position_timer.timeout.connect(self.update_end_position)
        self.position_timer.start(500)  # 每500毫秒更新一次
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 创建串口选择区域
        self.port_frame = PortSelectionFrame(
            parent=self,
            refresh_callback=self.refresh_ports
        )
        main_layout.addWidget(self.port_frame)
        
        # 创建串口配置区域
        self.serial_config = SerialConfigFrame(
            parent=self,
            connect_callback=self.toggle_connection
        )
        main_layout.addWidget(self.serial_config)
        
        # 创建控制按钮区域
        self.control_buttons = ControlButtonsFrame(
            parent=self,
            send_control_command_callback=self.send_control_command
        )
        main_layout.addWidget(self.control_buttons)
        
        # 创建角度控制区域
        self.angle_control = AngleControlFrame(
            parent=self,
            send_callback=self.send_angles, 
            convert_callback=self.convert_angles, 
            zero_callback=self.zero_angles
        )
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
        self.inverse_kinematic = InverseKinematicFrame(
            parent=self,
            inverse_callback=self.calculate_inverse_kinematics
        )
        self.inverse_kinematic.set_apply_callback(self.apply_inverse_kinematics_result)
        ik_layout.addWidget(self.inverse_kinematic)
        ik_tab.setLayout(ik_layout)
        tab_widget.addTab(ik_tab, "逆运动学")
        
        main_layout.addWidget(tab_widget)
    
    def refresh_ports(self):
        """刷新串口列表"""
        available_ports = self.serial_controller.get_available_ports()
        self.port_frame.set_ports(available_ports)
    
    def toggle_connection(self):
        """切换连接状态"""
        # 获取按钮当前文本，判断用户意图
        button_text = self.serial_config.connect_button.text()
        
        if button_text == "断开连接":
            # 用户想要断开连接
            self.data_display.append_message("断开串口连接", "控制")
            self.serial_controller.disconnect()
            self.port_frame.update_connection_status(False)
            self.serial_config.update_connection_status(False)
            self.control_buttons.update_connection_status(False)
            self.angle_control.set_send_button_state(False)
        else:
            # 用户想要连接
            # 获取选择的串口
            port = self.port_frame.get_selected_port()
            if not port:
                QMessageBox.warning(self, "警告", "请选择一个串口")
                return
            
            # 获取串口配置
            config = self.serial_config.get_config()
            
            # 尝试连接前先记录操作信息
            self.data_display.append_message(f"连接到串口 {port}", "控制")
            config_str = f"波特率:{config['baud_rate']}, 数据位:{config['data_bits']}, 校验位:{config['parity']}, 停止位:{config['stop_bits']}"
            self.data_display.append_message(config_str, "参数")
            
            # 尝试连接
            try:
                success = self.serial_controller.connect(
                    port=port,
                    baud_rate=config['baud_rate'],
                    data_bits=config['data_bits'],
                    parity=config['parity'],
                    stop_bits=config['stop_bits'],
                    flow_control=config['flow_control']
                )
                
                if success:
                    self.port_frame.update_connection_status(True)
                    self.serial_config.update_connection_status(True)
                    self.control_buttons.update_connection_status(True)
                    self.angle_control.set_send_button_state(True)
                else:
                    QMessageBox.critical(self, "错误", "连接失败，请检查串口配置")
                    self.data_display.append_message("连接失败", "错误")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"连接失败: {str(e)}")
                self.data_display.append_message(f"连接异常: {str(e)}", "错误")
                return
    
    def send_control_command(self, command_type):
        """发送控制命令"""
        if not (hasattr(self.serial_controller.serial_model, 'is_connected') and 
                self.serial_controller.serial_model.is_connected):
            return
        
        # 获取编码格式和运行模式
        encoding_type = self.control_buttons.get_encoding_type()
        run_mode = self.control_buttons.get_run_mode()
        
        try:
            # 记录控制命令的高级描述
            command_desc = {
                "ENABLE": "使能",
                "DISABLE": "取消使能",
                "RELEASE": "释放刹车",
                "LOCK": "锁止刹车",
                "STOP": "立刻停止",
                "MOTION": "运动状态"
            }.get(command_type, f"执行 {command_type} 命令")
            
            # 显示控制命令描述
            self.data_display.append_message(command_desc, "控制")
            
            # 直接通过串口控制器发送命令并获取命令字符串
            success, cmd_str = self.serial_controller.send_control_command(
                command_type=command_type,
                encoding=encoding_type,
                mode=run_mode,
                return_cmd=True
            )
            
            if success:
                # 显示实际命令的内容
                self.data_display.append_message(f"{cmd_str}", "发送")
            else:
                # 记录错误
                self.data_display.append_message(f"发送控制命令失败: {command_type}", "错误")
            
        except Exception as e:
            # 记录异常
            self.data_display.append_message(f"发送控制命令异常: {str(e)}", "错误")
    
    def send_angles(self):
        """发送角度命令"""
        if not (hasattr(self.serial_controller.serial_model, 'is_connected') and 
                self.serial_controller.serial_model.is_connected):
            QMessageBox.warning(self, "警告", "请先连接串口")
            return
        
        try:
            # 获取角度值
            angles = self.angle_control.get_angles()
            
            # 获取曲线类型和时间参数
            curve_type, duration, frequency = self.angle_control.get_curve_type()
            
            # 获取编码类型
            encoding_type = self.control_buttons.get_encoding_type()
            
            # 获取运行模式
            run_mode = self.control_buttons.get_run_mode()
            
            # 使用控制类型消息显示高级操作描述
            angles_str = ", ".join([f"{angle:.4f}" for angle in angles])
            self.data_display.append_message(f"移动到角度: [{angles_str}]", "控制")
            self.data_display.append_message(f"曲线类型: {curve_type}, 时长: {duration}秒, 频率: {frequency}秒", "参数")
            
            if duration > 0 and frequency > 0:
                # 轨迹模式
                # 注：这里将关键的串口控制器设置为创建轨迹线程但不立即启动
                # 获取串口控制器准备好的轨迹线程
                self.serial_controller.prepare_trajectory(
                    angles=angles,
                    curve_type=curve_type,
                    duration=duration,
                    frequency=frequency,
                    encoding=encoding_type,
                    mode=run_mode
                )
                
                # 获取创建好的轨迹线程
                if hasattr(self.serial_controller, 'trajectory_thread') and self.serial_controller.trajectory_thread:
                    # 清除之前的连接
                    try:
                        self.serial_controller.trajectory_thread.point_sent.disconnect()
                        self.serial_controller.trajectory_thread.finished.disconnect()
                    except:
                        pass  # 如果没有连接过，会抛出异常，忽略
                    
                    # 连接轨迹点发送和完成信号
                    self.serial_controller.trajectory_thread.point_sent.connect(self.handle_trajectory_point_sent)
                    self.serial_controller.trajectory_thread.finished.connect(self.handle_trajectory_finished)
                    
                    # 启动轨迹线程
                    self.serial_controller.trajectory_thread.start()
                else:
                    self.data_display.append_message("创建轨迹线程失败", "错误")
            else:
                # 单点模式，直接获取完整命令字符串
                success, cmd_str = self.serial_controller.send_angles(
                    angles=angles,
                    curve_type=curve_type,
                    duration=0,
                    frequency=0,
                    encoding=encoding_type,
                    mode=run_mode,
                    return_cmd=True
                )
                
                if success:
                    # 显示实际发送的命令
                    self.data_display.append_message(f"{cmd_str}", "发送")
                    
                    # 更新曲线绘图
                    self.curve_plot.update_angles(angles)
                else:
                    self.data_display.append_message(f"发送角度命令失败", "错误")
                
        except Exception as e:
            self.data_display.append_message(f"发送角度命令异常: {str(e)}", "错误")
    
    def calculate_inverse_kinematics(self, x, y, z, A, B, C):
        """计算逆运动学"""
        try:
            # 调用机器人控制器计算逆运动学
            joint_angles = self.robot_controller.calculate_inverse_kinematics(x, y, z, A, B, C)
            
            if joint_angles:
                return joint_angles
            else:
                return None
        except Exception as e:
            QMessageBox.critical(self, "错误", f"逆运动学计算失败: {str(e)}")
            return None
    
    def apply_inverse_kinematics_result(self, angles):
        """应用逆运动学计算结果"""
        if not angles:
            return
        
        # 将计算结果更新到角度控制框
        for i, angle_var in enumerate(self.angle_control.angle_vars):
            if i < len(angles):
                angle_var.setText(f"{angles[i]:.6f}")
        
        # 显示应用信息
        angles_str = ", ".join([f"{a:.4f}" for a in angles])
        self.data_display.append_message(f"应用逆运动学结果: [{angles_str}]", "控制")
    
    def zero_angles(self):
        """归零处理"""
        # 将所有角度值设置为0
        for angle_var in self.angle_control.angle_vars:
            angle_var.setText("0.0")
        
        # 显示归零信息
        self.data_display.append_message("角度值归零", "控制")
    
    def update_end_position(self):
        """更新末端位置显示"""
        if hasattr(self.end_position, 'update_position'):
            # 获取当前末端位置
            position = self.robot_controller.get_end_position()
            if position:
                self.end_position.update_position(*position)
    
    def handle_data_received(self, data):
        """处理接收到的数据"""
        self.data_display.append_message(data, "接收")
    
    def handle_connection_changed(self, connected):
        """处理连接状态变化"""
        if not connected:
            self.port_frame.update_connection_status(False)
            self.serial_config.update_connection_status(False)
            self.control_buttons.update_connection_status(False)
            self.angle_control.set_send_button_state(False)
    
    def handle_error_occurred(self, error_message):
        """处理错误"""
        self.data_display.append_message(f"错误: {error_message}", "错误")
        QMessageBox.critical(self, "错误", error_message)
    
    def clear_send(self):
        """清除发送数据区域"""
        self.data_display.clear_send()
    
    def clear_receive(self):
        """清除接收数据区域"""
        self.data_display.clear_receive()
    
    def clear_all(self):
        """清除所有数据区域"""
        self.data_display.clear_all()
    
    def convert_angles(self):
        """度数转弧度处理"""
        # 获取当前角度值
        angles = self.angle_control.get_angles()
        # 将角度值乘以π/180转换为弧度值
        radian_angles = [angle * math.pi / 180 for angle in angles]
        
        # 更新角度控制框中的值
        for i, angle_var in enumerate(self.angle_control.angle_vars):
            angle_var.setText(f"{radian_angles[i]:.6f}")
        
        # 显示转换信息
        self.data_display.append_message("度数转换为弧度", "控制")
    
    def handle_trajectory_point_sent(self, index, total, cmd_str, success):
        """处理轨迹点发送信号"""
        if success:
            self.data_display.append_message(f"轨迹点[{index}/{total}]已发送", "控制")
            self.data_display.append_message(f"{cmd_str}", "发送")
        else:
            self.data_display.append_message(f"轨迹点[{index}/{total}]发送失败", "错误")
    
    def handle_trajectory_finished(self, success, cmd_strs):
        """处理轨迹发送完成信号"""
        if success:
            self.data_display.append_message(f"轨迹执行完成，共{len(cmd_strs)}个点", "控制")
        else:
            self.data_display.append_message("轨迹执行失败", "错误") 