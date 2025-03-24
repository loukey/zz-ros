"""
主窗口视图
"""
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QTabWidget, QMessageBox, QHBoxLayout
from PyQt5.QtCore import QTimer
from gui.components.serial_components import PortSelectionFrame, SerialConfigFrame
from gui.components.control_components import ControlButtonsFrame, AngleControlFrame
from gui.components.display_components import DataDisplayFrame
from gui.components.kinematic_components import InverseKinematicFrame, CurvePlotFrame, EndPositionFrame
from gui.controllers.robot_controller import RobotController
from gui.controllers.serial_controller import SerialController
from gui.controllers.ros_controller import ROSController
from gui.controllers.trajectory_controller import TrajectoryController
from math import pi
import math
from gui.utils.command_utils import calculate_crc16
from gui.views.simulation_window import SimulationWindow  # 导入仿真窗口类
from gui.views.camera_window import CameraWindow  # 导入相机窗口类

class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("镇中科技机械臂控制工具v0.1.1")
        self.setMinimumSize(1200, 800)
        
        # 创建串口控制器
        self.serial_controller = SerialController()
        
        # 创建机器人控制器，传入串口控制器
        self.robot_controller = RobotController(self.serial_controller)
        
        # 创建ROS控制器
        self.ros_controller = ROSController()
        # 初始化ROS控制器
        self.ros_initialized = False
        try:
            self.ros_initialized = self.ros_controller.initialize()
            if self.ros_initialized:
                print("ROS控制器初始化成功")
            else:
                print("ROS控制器初始化失败")
        except Exception as e:
            print(f"ROS控制器初始化异常: {str(e)}")
        
        # 创建轨迹控制器
        self.trajectory_controller = TrajectoryController(self.serial_controller, self.ros_controller)
        
        # 注册回调函数
        self.serial_controller.serial_model.data_received.connect(self.handle_data_received)
        self.serial_controller.serial_model.connection_changed.connect(self.handle_connection_changed)
        self.serial_controller.serial_model.error_occurred.connect(self.handle_error_occurred)
        
        # 注册轨迹控制器的信号
        self.trajectory_controller.trajectory_started.connect(self.handle_trajectory_started)
        self.trajectory_controller.trajectory_paused.connect(self.handle_trajectory_paused)
        self.trajectory_controller.trajectory_resumed.connect(self.handle_trajectory_resumed)
        self.trajectory_controller.trajectory_stopped.connect(self.handle_trajectory_stopped)
        self.trajectory_controller.trajectory_finished.connect(self.handle_trajectory_finished)
        self.trajectory_controller.trajectory_error.connect(self.handle_trajectory_error)
        self.trajectory_controller.progress_updated.connect(self.handle_trajectory_progress)
        
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
        
        # 创建顶层标签页控件（位于左上角）
        self.main_tab_widget = QTabWidget()
        main_layout.addWidget(self.main_tab_widget)
        
        # 创建第一个标签页（主控制页面）
        self.control_tab = QWidget()
        self.main_tab_widget.addTab(self.control_tab, "机械臂控制")
        
        # 第二个标签页（仿真页面）
        self.simulation_window = SimulationWindow(self)
        self.main_tab_widget.addTab(self.simulation_window, "仿真控制")
        
        # 第三个标签页（相机页面）
        self.camera_window = CameraWindow(self)
        self.main_tab_widget.addTab(self.camera_window, "相机控制")
        
        # 为第一个标签页设置布局
        control_layout = QVBoxLayout(self.control_tab)
        
        # 创建串口选择区域
        self.port_frame = PortSelectionFrame(
            parent=self,
            refresh_callback=self.refresh_ports
        )
        control_layout.addWidget(self.port_frame)
        
        # 创建串口配置区域
        self.serial_config = SerialConfigFrame(
            parent=self,
            connect_callback=self.toggle_connection
        )
        control_layout.addWidget(self.serial_config)
        
        # 创建控制按钮区域
        self.control_buttons = ControlButtonsFrame(
            parent=self,
            send_control_command_callback=self.send_control_command
        )
        control_layout.addWidget(self.control_buttons)
        
        # 创建角度控制区域
        self.angle_control = AngleControlFrame(
            parent=self,
            send_callback=self.send_angles, 
            convert_callback=self.convert_angles, 
            zero_callback=self.zero_angles
        )
        control_layout.addWidget(self.angle_control)
        
        # 创建功能标签页控件（嵌套在主控制标签页内）
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
        
        control_layout.addWidget(tab_widget)
    
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
            QMessageBox.warning(self, "警告", "请先连接串口")
            return
        
        # 获取编码格式和运行模式
        encoding_type = self.control_buttons.get_encoding_type()
        run_mode = self.control_buttons.get_run_mode()
        # 记录控制命令的高级描述
        command_desc = {
            "ENABLE": "使能",
            "DISABLE": "取消使能",
            "RELEASE": "释放刹车",
            "LOCK": "锁止刹车",
            "STOP": "立刻停止",
            "PAUSE": "暂停"
        }.get(command_type, f"执行 {command_type} 命令")
        
        # 如果是暂停命令，停止轨迹执行
        if command_type == "PAUSE":
            self.trajectory_controller.pause_trajectory()
            return
        
        # 如果是停止命令，停止轨迹执行
        if command_type == "STOP":
            self.trajectory_controller.stop_trajectory()
            return
        
        # 发送控制命令并获取命令字符串
        success, cmd_str = self.serial_controller.send_control_command(
            command_type=command_type,
            encoding=encoding_type,
            mode=run_mode,
            return_cmd=True  # 确保返回命令字符串
        )
        
        # 显示控制命令描述
        self.data_display.append_message(cmd_str, "控制")
        
        if success:
            # 显示实际发送的命令
            self.data_display.append_message(f"{cmd_str}", "发送")
        else:
            self.data_display.append_message(f"发送控制命令失败: {command_type}", "错误")
            QMessageBox.warning(self, "错误", f"发送{command_desc}命令失败")
    
    def send_angles(self):
        """发送角度命令"""
        if not (hasattr(self.serial_controller.serial_model, 'is_connected') and 
                self.serial_controller.serial_model.is_connected):
            QMessageBox.warning(self, "警告", "请先连接串口")
            return
        
        try:
            # 获取目标角度值（从界面输入）
            target_angles = self.angle_control.get_angles()
            
            # 获取曲线类型和时间参数
            curve_type, duration, frequency = self.angle_control.get_curve_type()
            
            # 获取编码类型
            encoding_type = self.control_buttons.get_encoding_type()
            
            # 获取运行模式
            run_mode = self.control_buttons.get_run_mode()
            
            # 记录操作信息
            angles_str = ", ".join([f"{angle:.4f}" for angle in target_angles])
            self.data_display.append_message(f"目标角度: [{angles_str}]", "控制")
            self.data_display.append_message(f"曲线类型: {curve_type}, 时长: {duration}秒, 频率: {frequency}秒", "参数")
            
            # 先发送cmd 07获取当前位置
            self.data_display.append_message("正在获取当前位置...", "控制")
            success, cmd_str = self.serial_controller.send_control_command(
                command_type="POSITION",  # 需要确保serial_controller中有POSITION对应的控制字节07
                encoding=encoding_type,
                mode=run_mode,
                return_cmd=True
            )
            
            if not success:
                self.data_display.append_message("获取当前位置失败，无法执行差分运动", "错误")
                return
                
            self.data_display.append_message(f"{cmd_str}", "发送")
            
            # 等待接收当前位置响应
            self.data_display.append_message("等待位置数据响应...", "控制")
            
            # 设置标志和变量，将在handle_data_received中使用
            self.waiting_for_position = True
            self.current_position = None
            self.target_angles_pending = target_angles
            self.curve_params_pending = (curve_type, duration, frequency)
            self.encoding_type_pending = encoding_type
            self.run_mode_pending = run_mode
            
            # 创建定时器等待响应，超时处理
            self.position_response_timer = QTimer()
            self.position_response_timer.setSingleShot(True)
            self.position_response_timer.timeout.connect(self.on_position_response_timeout)
            self.position_response_timer.start(2000)  # 2秒超时
            
            # 注意：实际的差分处理会在handle_data_received方法中完成
            # 当接收到msg 07响应时，会调用process_differential_motion方法
                
        except Exception as e:
            self.data_display.append_message(f"发送角度命令异常: {str(e)}", "错误")
    
    def on_position_response_timeout(self):
        """位置响应超时处理"""
        if self.waiting_for_position:
            self.waiting_for_position = False
            self.data_display.append_message("获取当前位置超时", "错误")
            self.current_position = None
    
    def process_differential_motion(self):
        """处理差分运动"""
        if not self.current_position or not self.target_angles_pending:
            self.data_display.append_message("缺少当前位置或目标位置数据", "错误")
            return
        
        try:
            # 获取存储的目标角度和参数
            target_angles = self.target_angles_pending
            curve_type, duration, frequency = self.curve_params_pending
            encoding_type = self.encoding_type_pending
            run_mode = self.run_mode_pending
            
            # 偏移值
            OFFSETS = [78623, 369707, 83986, 391414, 508006, 455123]
            
            # 转换系数 2π / 2^19
            SCALE_FACTOR = (2 * math.pi) / (2**19)
            
            # 记录接收到的位置类型和值，方便调试
            pos_types = [f"{type(pos).__name__}" for pos in self.current_position]
            self.data_display.append_message(f"位置数据类型: {pos_types}", "调试")
            self.data_display.append_message(f"位置数据值: {self.current_position}", "调试")
            
            # 将当前位置从整数值转换为弧度
            current_angles_rad = []
            for i, pos in enumerate(self.current_position):
                try:
                    # 先将字符串转换为整数
                    pos_value = int(pos)
                    # 原始整数转换成弧度: (整数值 - 偏移值) * (2π / 2^19)
                    angle_rad = (pos_value - OFFSETS[i]) * SCALE_FACTOR
                    current_angles_rad.append(angle_rad)
                except ValueError as ve:
                    # 详细的转换错误信息
                    self.data_display.append_message(f"关节{i+1}位置值'{pos}'无法转换为整数: {str(ve)}", "错误")
                    raise  # 重新抛出异常
                except Exception as e:
                    self.data_display.append_message(f"处理关节{i+1}位置时发生错误: {str(e)}", "错误")
                    raise  # 重新抛出异常
            
            self.data_display.append_message(f"当前角度(rad): {[round(a, 4) for a in current_angles_rad]}", "控制")
            self.data_display.append_message(f"目标角度(rad): {[round(a, 4) for a in target_angles]}", "控制")
            
            # 发送到ROS系统
            if self.ros_initialized:
                # 对于轨迹发送，使用同步的轨迹发布
                if duration > 0 and frequency > 0:
                    # 将轨迹类型从字符串转换为ROS控制器需要的格式
                    ros_curve_type = "Trapezoid" if curve_type.lower() == "trapezoid" else "S-Curve"
                    
                    # 启动ROS轨迹发布
                    if self.ros_controller.publish_trajectory(
                        angles=target_angles,
                        start_angles=current_angles_rad,
                        duration=duration,
                        frequency=frequency,
                        curve_type=ros_curve_type
                    ):
                        self.data_display.append_message(f"已启动ROS轨迹发布，持续{duration}秒，频率{frequency}秒", "ROS")
                    else:
                        self.data_display.append_message("启动ROS轨迹发布失败", "错误")
                else:
                    # 单点模式，直接发送目标角度
                    if self.ros_controller.publish_angles(target_angles):
                        self.data_display.append_message("已将目标角度发送至ROS系统", "ROS")
                    else:
                        self.data_display.append_message("发送角度到ROS系统失败", "错误")
            
            # 计算差分轨迹
            self.data_display.append_message("计算差分轨迹...", "控制")
            
            # 差分轨迹使用的是目标位置和当前位置之间的运动
            if duration > 0 and frequency > 0:
                # 轨迹模式 - 使用当前位置作为起点，目标位置作为终点
                self.serial_controller.prepare_trajectory(
                    angles=target_angles,
                    start_angles=current_angles_rad,  # 传入当前位置作为起点
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
                # 单点模式，直接发送目标角度
                success, cmd_str = self.serial_controller.send_angles(
                    angles=target_angles,
                    start_angles=current_angles_rad,  # 传入当前位置作为起点
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
                    if hasattr(self, 'curve_plot') and hasattr(self.curve_plot, 'plot_angles'):
                        self.curve_plot.plot_angles(target_angles)
                else:
                    self.data_display.append_message("发送角度命令失败", "错误")
            
            # 清除等待标志和临时变量
            self.waiting_for_position = False
            self.target_angles_pending = None
            self.curve_params_pending = None
            self.encoding_type_pending = None
            self.run_mode_pending = None
            
        except Exception as e:
            self.data_display.append_message(f"差分运动处理异常: {str(e)}", "错误")
            import traceback
            self.data_display.append_message(f"错误详情: {traceback.format_exc()}", "错误")
            # 重置等待位置的标志
            self.waiting_for_position = False
    
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
        """应用逆运动学结果
        
        Args:
            angles: 关节角度列表
        """
        self.angle_control.set_angles(angles)
        
        # 发送到ROS系统
        if self.ros_initialized:
            if self.ros_controller.publish_angles(angles):
                self.data_display.append_message("已将逆运动学结果发送至ROS系统", "ROS")
            else:
                self.data_display.append_message("发送角度到ROS系统失败", "错误")
    
    def zero_angles(self):
        """归零处理"""
        # 将所有角度值设置为0
        for angle_var in self.angle_control.angle_vars:
            angle_var.setText("0.0")
        
        # 显示归零信息
        self.data_display.append_message("角度值归零", "控制")
    
    def update_end_position(self):
        """更新末端位置显示"""
        pass
        # if hasattr(self.end_position, 'update_position'):
        #     # 获取当前末端位置
        #     position = self.robot_controller.get_end_position()
        #     if position:
        #         self.end_position.update_position(*position)
    
    def handle_data_received(self, data):
        """处理接收到的数据"""
        # 去除首尾空白和换行符后处理
        clean_data = data.strip()
        self.data_display.append_message(f"接收数据: '{clean_data}'", "接收")
        
        # 如果数据以"msg"开头，尝试解析更详细的内容
        if clean_data.lower().startswith('msg'):
            try:
                # 按空格分割
                parts = clean_data.split()
                if len(parts) >= 22:  # msg命令完整格式 (msg + 控制 + 模式 + 6个位置 + 6个状态 + 6个异常 + CRC)
                    control = parts[1]
                    mode = parts[2]
                    positions = parts[3:9]
                    status = parts[9:15]
                    errors = parts[15:21]
                    crc = parts[21]
                    
                    # 记录解析后的详细信息
                    self.data_display.append_message(f"控制字节: {control}, 模式: {mode}", "接收")
                    self.data_display.append_message(f"位置数据: {positions}", "接收")
                    self.data_display.append_message(f"状态字: {status}", "接收")
                    self.data_display.append_message(f"异常值: {errors}", "接收")
                    
                    # 验证CRC16校验
                    # 构造用于计算CRC的消息（去除CRC部分）
                    crc_message = ' '.join(parts[:-1])
                    calculated_crc = calculate_crc16(crc_message)
                    calculated_crc_hex = f"{calculated_crc:04X}"
                    
                    # 验证校验结果
                    if calculated_crc_hex == crc:
                        self.data_display.append_message(f"CRC校验: 正确 (接收: {crc}, 计算: {calculated_crc_hex})", "接收")
                    else:
                        self.data_display.append_message(f"CRC校验: 错误 (接收: {crc}, 计算: {calculated_crc_hex})", "错误")
                    
                    # 检查是否是位置响应 (msg 07)
                    if control == '07' and hasattr(self, 'waiting_for_position') and self.waiting_for_position:
                        self.data_display.append_message("收到当前位置响应", "控制")
                        # 停止计时器
                        if hasattr(self, 'position_response_timer') and self.position_response_timer.isActive():
                            self.position_response_timer.stop()
                        
                        # 存储当前位置
                        self.current_position = positions
                        
                        # 处理差分运动
                        self.process_differential_motion()
                        return
                    
                    # 更新曲线 - 使用plot_curves方法
                    try:
                        # 偏移值
                        OFFSETS = [78623, 369707, 83986, 391414, 508006, 455123]
                        
                        # 转换系数 2π / 2^19
                        SCALE_FACTOR = (2 * math.pi) / (2**19)
                        
                        # 转换位置数据到弧度角度
                        angles = []
                        for pos_str, offset in zip(positions, OFFSETS):
                            pos_value = int(pos_str)
                            # 减去偏移值
                            adjusted_value = (pos_value - offset) & 0xFFFFFF
                            # 如果是负数(高位为1)，则转换为有符号数
                            if adjusted_value & 0x800000:
                                adjusted_value = adjusted_value - 0x1000000
                            # 转换为弧度
                            angle = adjusted_value * SCALE_FACTOR
                            angles.append(angle)
                        
                        # 使用plot_curves方法绘制曲线
                        self.curve_plot.plot_curves(angles, "trapezoidal", 4.0, 0.1)
                    except ValueError as e:
                        self.data_display.append_message(f"角度转换失败: {str(e)}", "错误")
                    except Exception as e:
                        self.data_display.append_message(f"曲线绘制失败: {str(e)}", "错误")
            except Exception as e:
                self.data_display.append_message(f"解析消息失败: {str(e)}", "错误")
    
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
    
    def handle_trajectory_finished(self, success, points):
        """处理轨迹完成事件
        
        Args:
            success: 是否成功完成
            points: 轨迹点列表
        """
        if success:
            self.data_display.append_message(f"轨迹执行完成，共发送 {len(points)} 个点", "控制")
        else:
            self.data_display.append_message(f"轨迹执行失败，已发送 {len(points)} 个点", "错误")
        self.control_buttons.set_trajectory_running(False)
    
    def handle_trajectory_started(self):
        """处理轨迹开始事件"""
        self.data_display.append_message("开始执行轨迹", "控制")
        self.control_buttons.set_trajectory_running(True)
    
    def handle_trajectory_paused(self):
        """处理轨迹暂停事件"""
        self.data_display.append_message("轨迹已暂停", "控制")
        self.control_buttons.set_trajectory_running(False)
    
    def handle_trajectory_resumed(self):
        """处理轨迹恢复事件"""
        self.data_display.append_message("轨迹已恢复", "控制")
        self.control_buttons.set_trajectory_running(True)
    
    def handle_trajectory_stopped(self):
        """处理轨迹停止事件"""
        self.data_display.append_message("轨迹已停止", "控制")
        self.control_buttons.set_trajectory_running(False)
    
    def handle_trajectory_error(self, error_msg):
        """处理轨迹错误事件
        
        Args:
            error_msg: 错误信息
        """
        self.data_display.append_message(error_msg, "错误")
        self.control_buttons.set_trajectory_running(False)
    
    def handle_trajectory_progress(self, current_index, total_points):
        """处理轨迹进度更新事件
        
        Args:
            current_index: 当前点索引
            total_points: 总点数
        """
        # 更新进度显示
        progress = (current_index / total_points) * 100
        self.control_buttons.update_progress(progress)
    
    def closeEvent(self, event):
        """窗口关闭事件处理"""
        # 停止轨迹执行
        self.trajectory_controller.stop_trajectory()
        
        # 断开串口连接
        if hasattr(self.serial_controller.serial_model, 'is_connected') and self.serial_controller.serial_model.is_connected:
            self.serial_controller.disconnect()
        
        # 关闭ROS控制器
        if self.ros_initialized:
            self.ros_controller.shutdown()
        
        # 关闭仿真窗口
        if hasattr(self, 'simulation_window'):
            self.simulation_window.close()
        
        # 关闭相机窗口
        if hasattr(self, 'camera_window'):
            self.camera_window.close()
        
        event.accept() 