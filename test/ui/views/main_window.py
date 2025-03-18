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
from math import pi
import math


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("机器人控制界面")
        self.setMinimumSize(1200, 800)
        
        # 创建机器人控制器
        self.robot_controller = RobotController()
        
        # 注册回调函数
        self.robot_controller.serial_controller.serial_model.data_received.connect(self.handle_data_received)
        self.robot_controller.serial_controller.serial_model.connection_changed.connect(self.handle_connection_changed)
        self.robot_controller.serial_controller.serial_model.error_occurred.connect(self.handle_error_occurred)
        
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
        available_ports = self.robot_controller.serial_controller.get_available_ports()
        self.port_frame.set_ports(available_ports)
    
    def toggle_connection(self):
        """切换连接状态"""
        if hasattr(self.robot_controller.serial_controller.serial_model, 'is_connected') and self.robot_controller.serial_controller.serial_model.is_connected:
            # 断开连接
            self.robot_controller.disconnect_serial()
            self.port_frame.update_connection_status(False)
            self.serial_config.update_connection_status(False)
            self.control_buttons.update_connection_status(False)
            self.angle_control.set_send_button_state(False)
            self.data_display.append_message("已断开串口连接", "信息")
        else:
            # 获取选择的串口
            port = self.port_frame.get_selected_port()
            if not port:
                QMessageBox.warning(self, "警告", "请选择一个串口")
                return
            
            # 获取串口配置
            config = self.serial_config.get_config()
            
            # 尝试连接
            try:
                self.robot_controller.connect_serial(
                    port=port,
                    baud_rate=config['baud_rate'],
                    data_bits=config['data_bits'],
                    parity=config['parity'],
                    stop_bits=config['stop_bits'],
                    flow_control=config['flow_control']
                )
                self.port_frame.update_connection_status(True)
                self.serial_config.update_connection_status(True)
                self.control_buttons.update_connection_status(True)
                self.angle_control.set_send_button_state(True)
                self.data_display.append_message(f"已连接到串口 {port}", "信息")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"连接失败: {str(e)}")
                return
    
    def send_control_command(self, command_type):
        """发送控制命令"""
        if not (hasattr(self.robot_controller.serial_controller.serial_model, 'is_connected') and 
                self.robot_controller.serial_controller.serial_model.is_connected):
            return
        
        # 获取编码格式和运行模式
        encoding_type = self.control_buttons.get_encoding_type()
        run_mode = self.control_buttons.get_run_mode()
        
        try:
            # 调用SerialController的方法发送控制命令
            success, cmd_str = self.robot_controller.serial_controller.send_control_command(
                command_type=command_type,
                encoding=encoding_type,
                mode=run_mode,
                return_cmd=True  # 要求返回命令字符串
            )
            
            if success:
                # 记录完整的消息格式
                self.data_display.append_message(f"发送控制命令: {command_type}", "发送")
                self.data_display.append_message(f"完整命令: {cmd_str}", "发送")
            else:
                # 记录错误
                self.data_display.append_message(f"发送控制命令失败: {command_type}", "错误")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"发送命令失败: {str(e)}")
    
    def send_angles(self):
        """发送角度命令"""
        if not (hasattr(self.robot_controller.serial_controller.serial_model, 'is_connected') and self.robot_controller.serial_controller.serial_model.is_connected):
            return
        
        # 获取角度值和曲线参数
        angles = self.angle_control.get_angles()
        curve_type, duration, frequency = self.angle_control.get_curve_type()
        # 获取编码格式和运行模式
        encoding_type = self.control_buttons.get_encoding_type()
        run_mode = self.control_buttons.get_run_mode()
        
        try:
            # 绘制曲线
            self.curve_plot.plot_curves(angles, curve_type, duration, frequency)
            
            # 记录基本发送信息
            angles_str = ", ".join([f"{angle:.4f}" for angle in angles])
            self.data_display.append_message(
                f"发送角度: [{angles_str}], 曲线类型: {curve_type}, 时长: {duration}秒, 频率: {frequency}秒",
                "发送"
            )
            
            # 如果是轨迹发送模式，连接信号
            if duration > 0 and frequency > 0:
                # 先创建轨迹线程（会自动清除旧线程）
                success = self.robot_controller.serial_controller.send_angles(
                    angles=angles,
                    curve_type=curve_type,
                    duration=duration,
                    frequency=frequency,
                    encoding=encoding_type,
                    mode=run_mode,
                    return_cmd=False  # 非阻塞方式
                )
                
                if not success:
                    # 记录错误
                    self.data_display.append_message("发送角度命令失败", "错误")
                    return
                
                # 获取创建好的轨迹线程
                traj_thread = self.robot_controller.serial_controller.trajectory_thread
                if traj_thread:
                    # 清除之前的连接
                    try:
                        traj_thread.point_sent.disconnect(self.handle_trajectory_point_sent)
                        traj_thread.finished.disconnect(self.handle_trajectory_finished)
                    except:
                        pass  # 如果没有连接过，会抛出异常，忽略
                    
                    # 连接轨迹点发送和完成信号
                    traj_thread.point_sent.connect(self.handle_trajectory_point_sent)
                    traj_thread.finished.connect(self.handle_trajectory_finished)
                else:
                    self.data_display.append_message("创建轨迹线程失败", "错误")
            else:
                # 单点发送模式，使用阻塞方式
                success, cmd_str = self.robot_controller.serial_controller.send_angles(
                    angles=angles,
                    curve_type=curve_type,
                    duration=0,
                    frequency=0,
                    encoding=encoding_type,
                    mode=run_mode,
                    return_cmd=True  # 要求返回命令字符串
                )
                
                if success:
                    # 记录完整命令
                    self.data_display.append_message(f"完整命令: {cmd_str}", "发送")
                else:
                    # 记录错误
                    self.data_display.append_message("发送角度命令失败", "错误")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"发送角度命令失败: {str(e)}")
    
    def handle_trajectory_point_sent(self, index, total, cmd_str, success):
        """处理轨迹点发送信号"""
        if success:
            self.data_display.append_message(f"轨迹点[{index}/{total}]: {cmd_str}", "发送")
        else:
            self.data_display.append_message(f"轨迹点[{index}/{total}]发送失败", "错误")
    
    def handle_trajectory_finished(self, success, cmd_strs):
        """处理轨迹发送完成信号"""
        if success:
            self.data_display.append_message(f"轨迹发送完成，共{len(cmd_strs)}个点", "信息")
        else:
            self.data_display.append_message("轨迹发送失败", "错误")
    
    def calculate_inverse_kinematics(self, x, y, z, A, B, C):
        """计算逆运动学"""
        try:
            # 转化为浮点数
            x = float(x)
            y = float(y)
            z = float(z)
            A = float(A)
            B = float(B)
            C = float(C)
            angles = self.robot_controller.calculate_inverse_kinematics(x, y, z, A, B, C)
            return angles
        except Exception as e:
            QMessageBox.warning(self, "错误", f"计算逆运动学失败: {str(e)}")
            return None
    
    def apply_inverse_kinematics_result(self, angles):
        """应用逆运动学计算结果"""
        if angles:
            self.angle_control.set_angles(angles)
    
    def zero_angles(self):
        """所有角度清零"""
        self.angle_control.set_angles([0.0] * 6)
    
    def update_end_position(self):
        """更新末端位置显示"""
        if not (hasattr(self.robot_controller.serial_controller.serial_model, 'is_connected') and 
                self.robot_controller.serial_controller.serial_model.is_connected):
            return
        
        try:
            # 获取当前末端位置
            x, y, z, A, B, C = self.robot_controller.get_end_position()
            # 更新末端位置和姿态显示
            self.end_position.update_theoretical_position(x, y, z)
            self.end_position.update_theoretical_attitude(A, B, C)
        except Exception as e:
            pass
    
    def handle_data_received(self, data, is_hex=False):
        """处理接收到的数据"""
        if is_hex:
            # 如果是十六进制数据，则转换为十六进制字符串
            hex_string = ' '.join([f"{byte:02X}" for byte in data])
            self.data_display.append_message(hex_string, "接收(HEX)")
        else:
            # 如果是字符串数据，直接显示
            self.data_display.append_message(data.decode('utf-8', errors='replace'), "接收")
    
    def handle_connection_changed(self, connected):
        """处理连接状态变化"""
        status = "已连接" if connected else "已断开"
        self.data_display.append_message(f"串口状态: {status}", "系统")
        
        # 更新UI状态
        self.port_frame.update_connection_status(connected)
        self.control_buttons.update_connection_status(connected)
    
    def handle_error_occurred(self, error_message):
        """处理错误消息"""
        self.data_display.append_message(f"错误: {error_message}", "错误")
    
    def clear_send(self):
        """清除发送区数据"""
        self.data_display.clear_display("send")
    
    def clear_receive(self):
        """清除接收区数据"""
        self.data_display.clear_display("receive")
    
    def clear_all(self):
        """清除所有数据显示"""
        self.data_display.clear_display("all")
    
    def convert_angles(self):
        """转换角度值（度数转弧度）"""
        angles = self.angle_control.get_angles()
        converted_angles = [math.radians(angle) for angle in angles]
        self.angle_control.set_angles(converted_angles)
        self.data_display.append_message(f"角度转换: {angles}° -> {converted_angles}rad", "参数") 