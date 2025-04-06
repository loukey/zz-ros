"""
串口处理模块
"""
from PyQt5.QtWidgets import QMessageBox
from controllers.serial_controller import SerialController
from utils.command_utils import calculate_crc16, position_to_radian


class SerialHandler:
    """串口处理类，处理串口通信相关功能"""
    
    def __init__(self, main_window):
        self.main_window = main_window
        self.serial_controller = SerialController()
        
        # 注册回调函数
        self.serial_controller.serial_model.data_received.connect(self.handle_data_received)
        self.serial_controller.serial_model.connection_changed.connect(self.handle_connection_changed)
        self.serial_controller.serial_model.error_occurred.connect(self.handle_error_occurred)

        self.buffer = ""
    
    def refresh_ports(self):
        """刷新串口列表"""
        available_ports = self.serial_controller.get_available_ports()
        self.main_window.port_frame.set_ports(available_ports)
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.serial_controller:
            return
            
        button_text = self.main_window.port_frame.connect_button.text()
        if button_text == "连接串口":
            # 获取串口配置
            port = self.main_window.port_frame.get_selected_port()
            if not port:
                self.main_window.data_display.append_message("请选择串口", "错误")
                return
                
            config = self.main_window.serial_config.get_config()
            
            # 显示配置参数
            config_str = f"波特率:{config['baud_rate']}, 数据位:{config['data_bits']}, 校验位:{config['parity']}, 停止位:{config['stop_bits']}"
            self.main_window.data_display.append_message(config_str, "参数")
            
            try:
                # 连接串口
                if self.serial_controller.connect(
                    port=port,
                    baud_rate=config['baud_rate'],
                    data_bits=config['data_bits'],
                    parity=config['parity'],
                    stop_bits=config['stop_bits'],
                    flow_control=config['flow_control']
                ):
                    self.main_window.data_display.append_message(f"已连接到串口 {port}", "串口")
                    self.handle_connection_changed(True)
                else:
                    self.main_window.data_display.append_message("连接串口失败", "错误")
            except Exception as e:
                self.main_window.data_display.append_message(f"连接异常: {str(e)}", "错误")
        else:
            # 断开连接
            self.serial_controller.disconnect()
            self.main_window.data_display.append_message("已断开串口连接", "串口")
            self.handle_connection_changed(False)
    
    def handle_data_received(self, data):
        """处理接收到的数据"""
        clean_data = data.strip()        
        encoding = self.main_window.control_buttons.get_encoding_type()    
        if encoding == 'hex':
            self.buffer += clean_data
            if "0A0D" in self.buffer:
                self.main_window.data_display.append_message(f"接收数据: '{self.buffer}'", "接收")
                lines = self.buffer.split("0A0D")
                self.buffer = lines[-1]
                command_line = lines[0]
                
                if command_line.startswith("AA55"):
                    try:
                        # 帧头 (2字节)
                        header = command_line[0:4]  # AA55
                        
                        # 初始化状态 (1字节)
                        init_status = command_line[4:6]
                        
                        # 当前命令 (1字节)
                        current_command = command_line[6:8]
                        
                        # 运行模式 (1字节)
                        run_mode = command_line[8:10]
                        
                        # 位置1-6 (每个3字节，共18字节)
                        positions = []
                        start = 10
                        for i in range(6):
                            pos = int(command_line[start:start+6], 16)
                            positions.append(pos)
                            start += 6
                        
                        # 状态字1-6 (每个2字节，共12字节)
                        status = []
                        for i in range(6):
                            stat = command_line[start:start+4]
                            status.append(stat)
                            start += 4
                        
                        # 实际速度1-6 (每个3字节，共18字节)
                        speeds = []
                        for i in range(6):
                            speed = int(command_line[start:start+6], 16)
                            speeds.append(speed)
                            start += 6
                        
                        # 错误码1-6 (每个2字节，共12字节)
                        errors = []
                        for i in range(6):
                            error = command_line[start:start+4]
                            errors.append(error)
                            start += 4
                        
                        # 夹爪数据 (4字节)
                        effector_data_1 = int(command_line[start:start+4], 16)
                        effector_data_2 = int(command_line[start+4:start+8], 16)
                        effector_data = "{}.{}".format(effector_data_1, effector_data_2)
                        start += 8
                        
                        # CRC16 (2字节)
                        crc = command_line[start:start+4]
                        crc_message = command_line[:-4]
                        calculated_crc = calculate_crc16(crc_message)
                        calculated_crc_hex = f"{calculated_crc:04X}"
                        if calculated_crc_hex == crc:
                            self.main_window.data_display.append_message(f"CRC校验: 正确 (接收: {crc}, 计算: {calculated_crc_hex})", "接收")
                        else:
                            self.main_window.data_display.append_message(f"CRC校验: 错误 (接收: {crc}, 计算: {calculated_crc_hex})", "错误")
                        
                        # 记录解析后的详细信息
                        return_msg = f"帧头: {header}, 初始状态: {init_status}, 当前命令: {current_command}, 运行模式: {run_mode}, 位置数据: {positions}, 状态字: {status}, 实际速度: {speeds}, 错误码: {errors}, 夹爪数据: {effector_data}, CRC16: {crc}"
                        self.main_window.data_display.append_message(return_msg, "接收")
                        if run_mode == "07":
                            self.main_window.data_display.append_message("收到当前位置响应", "控制")
                            self.main_window.motion_handler.process_differential_motion(positions)
                            return

                    except Exception as e:
                        self.main_window.data_display.append_message(f"解析AA55数据帧失败: {str(e)}", "错误")
        else:
            if clean_data.lower().startswith('msg'):
                try:
                    # 按空格分割
                    parts = clean_data.split()
                    if len(parts) >= 22:  # msg命令完整格式 (msg + 控制 + 模式 + 6个位置 + 6个状态 + 6个速度 + 6个异常 + 夹爪数据 + CRC)
                        init_status = parts[1]
                        control = parts[2]
                        mode = parts[3]
                        positions = parts[4:10]
                        status = parts[10:16]
                        speeds = parts[16:22]
                        errors = parts[22:28]
                        effector_data = parts[28]
                        crc = parts[29]
                        
                        # 记录解析后的详细信息
                        self.main_window.data_display.append_message(f"初始状态: {init_status}, 控制字节: {control}, 模式: {mode}", "接收")
                        self.main_window.data_display.append_message(f"位置数据: {positions}", "接收")
                        self.main_window.data_display.append_message(f"状态字: {status}", "接收")
                        self.main_window.data_display.append_message(f"速度: {speeds}", "接收")
                        self.main_window.data_display.append_message(f"异常值: {errors}", "接收")
                        self.main_window.data_display.append_message(f"夹爪数据: {effector_data}", "接收")

                        # 验证CRC16校验
                        crc_message = ' '.join(parts[:-1])
                        calculated_crc = calculate_crc16(crc_message)
                        calculated_crc_hex = f"{calculated_crc:04X}"
                        
                        if calculated_crc_hex == crc:
                            self.main_window.data_display.append_message(f"CRC校验: 正确 (接收: {crc}, 计算: {calculated_crc_hex})", "接收")
                        else:
                            self.main_window.data_display.append_message(f"CRC校验: 错误 (接收: {crc}, 计算: {calculated_crc_hex})", "错误")
                        
                        # 检查是否是位置响应 (msg 07)
                        if control == '07':
                            self.main_window.data_display.append_message("收到当前位置响应", "控制")
                            self.main_window.motion_handler.process_differential_motion(positions)
                            return
                        
                        # # 更新曲线 - 使用plot_curves方法
                        # try:
                        #     # 使用position_to_radian函数将位置值转换为弧度
                        #     angles = position_to_radian(positions)
                            
                        #     # 使用plot_curves方法绘制曲线
                        #     self.main_window.curve_plot.plot_curves(angles, "trapezoidal", 4.0, 0.1)
                        # except ValueError as e:
                        #     self.main_window.data_display.append_message(f"角度转换失败: {str(e)}", "错误")
                        # except Exception as e:
                        #     self.main_window.data_display.append_message(f"曲线绘制失败: {str(e)}", "错误")
                except Exception as e:
                    self.main_window.data_display.append_message(f"解析消息失败: {str(e)}", "错误")
    
    def handle_connection_changed(self, connected):
        self.main_window.serial_config.update_connection_status(connected)
        self.main_window.control_buttons.update_connection_status(connected)
        self.main_window.effector_settings.update_connection_status(connected)
        self.main_window.port_frame.update_connection_status(connected)
        self.main_window.angle_control.update_connection_status(connected)
    
    def handle_error_occurred(self, error_message):
        """处理错误"""
        self.main_window.data_display.append_message(f"错误: {error_message}", "错误")
        QMessageBox.critical(self.main_window, "错误", error_message)
    
    def handle_calculation_error(self, error_message):
        """处理轨迹计算错误
        
        Args:
            error_message: 错误信息
        """
        self.main_window.data_display.append_message(f"轨迹计算错误: {error_message}", "错误")
        
    def is_connected(self):
        """检查串口是否已连接"""
        return (hasattr(self.serial_controller.serial_model, 'is_connected') and 
                self.serial_controller.serial_model.is_connected) 
    