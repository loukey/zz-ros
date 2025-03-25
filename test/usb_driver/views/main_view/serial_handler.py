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
    
    def refresh_ports(self):
        """刷新串口列表"""
        available_ports = self.serial_controller.get_available_ports()
        self.main_window.port_frame.set_ports(available_ports)
    
    def toggle_connection(self):
        """切换连接状态"""
        button_text = self.main_window.serial_config.connect_button.text()
        
        if button_text == "断开连接":
            self.main_window.data_display.append_message("断开串口连接", "控制")
            self.serial_controller.disconnect()
            self.main_window.port_frame.update_connection_status(False)
            self.main_window.serial_config.update_connection_status(False)
            self.main_window.control_buttons.update_connection_status(False)
            self.main_window.angle_control.set_send_button_state(False)
        else:
            port = self.main_window.port_frame.get_selected_port()
            if not port:
                QMessageBox.warning(self.main_window, "警告", "请选择一个串口")
                return
            
            config = self.main_window.serial_config.get_config()
            
            self.main_window.data_display.append_message(f"连接到串口 {port}", "控制")
            config_str = f"波特率:{config['baud_rate']}, 数据位:{config['data_bits']}, 校验位:{config['parity']}, 停止位:{config['stop_bits']}"
            self.main_window.data_display.append_message(config_str, "参数")
            
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
                    self.main_window.port_frame.update_connection_status(True)
                    self.main_window.serial_config.update_connection_status(True)
                    self.main_window.control_buttons.update_connection_status(True)
                    self.main_window.angle_control.set_send_button_state(True)
                else:
                    QMessageBox.critical(self.main_window, "错误", "连接失败，请检查串口配置")
                    self.main_window.data_display.append_message("连接失败", "错误")
            except Exception as e:
                QMessageBox.critical(self.main_window, "错误", f"连接失败: {str(e)}")
                self.main_window.data_display.append_message(f"连接异常: {str(e)}", "错误")
    
    def handle_data_received(self, data):
        """处理接收到的数据"""
        # 去除首尾空白和换行符后处理
        clean_data = data.strip()
        self.main_window.data_display.append_message(f"接收数据: '{clean_data}'", "接收")
        
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
                    self.main_window.data_display.append_message(f"控制字节: {control}, 模式: {mode}", "接收")
                    self.main_window.data_display.append_message(f"位置数据: {positions}", "接收")
                    self.main_window.data_display.append_message(f"状态字: {status}", "接收")
                    self.main_window.data_display.append_message(f"异常值: {errors}", "接收")
                    
                    # 验证CRC16校验
                    crc_message = ' '.join(parts[:-1])
                    calculated_crc = calculate_crc16(crc_message)
                    calculated_crc_hex = f"{calculated_crc:04X}"
                    
                    if calculated_crc_hex == crc:
                        self.main_window.data_display.append_message(f"CRC校验: 正确 (接收: {crc}, 计算: {calculated_crc_hex})", "接收")
                    else:
                        self.main_window.data_display.append_message(f"CRC校验: 错误 (接收: {crc}, 计算: {calculated_crc_hex})", "错误")
                    
                    # 检查是否是位置响应 (msg 07)
                    if control == '07' and hasattr(self.main_window, 'waiting_for_position') and self.main_window.waiting_for_position:
                        self.main_window.data_display.append_message("收到当前位置响应", "控制")
                        # 停止计时器
                        if hasattr(self.main_window, 'position_response_timer') and self.main_window.position_response_timer.isActive():
                            self.main_window.position_response_timer.stop()
                        
                        # 存储当前位置
                        self.main_window.current_position = positions
                        
                        # 处理差分运动
                        self.main_window.motion_handler.process_differential_motion()
                        return
                    
                    # 更新曲线 - 使用plot_curves方法
                    try:
                        # 使用position_to_radian函数将位置值转换为弧度
                        angles = position_to_radian(positions)
                        
                        # 使用plot_curves方法绘制曲线
                        self.main_window.curve_plot.plot_curves(angles, "trapezoidal", 4.0, 0.1)
                    except ValueError as e:
                        self.main_window.data_display.append_message(f"角度转换失败: {str(e)}", "错误")
                    except Exception as e:
                        self.main_window.data_display.append_message(f"曲线绘制失败: {str(e)}", "错误")
            except Exception as e:
                self.main_window.data_display.append_message(f"解析消息失败: {str(e)}", "错误")
    
    def handle_connection_changed(self, connected, port=None):
        """处理连接状态变化"""
        if connected:
            self.main_window.status_bar.showMessage(f"已连接到端口: {port}")
            
            # 更新串口配置区域
            self.main_window.serial_config.update_connection_status(True)
            
            # 启用控制按钮
            if hasattr(self.main_window, 'control_buttons'):
                self.main_window.control_buttons.update_connection_status(True)
        else:
            self.main_window.status_bar.showMessage("已断开连接")
            
            # 更新串口配置区域
            self.main_window.serial_config.update_connection_status(False)
            
            # 禁用控制按钮
            if hasattr(self.main_window, 'control_buttons'):
                self.main_window.control_buttons.update_connection_status(False)
    
    def handle_error_occurred(self, error_message):
        """处理错误"""
        self.main_window.data_display.append_message(f"错误: {error_message}", "错误")
        QMessageBox.critical(self.main_window, "错误", error_message)
    
    def is_connected(self):
        """检查串口是否已连接"""
        return (hasattr(self.serial_controller.serial_model, 'is_connected') and 
                self.serial_controller.serial_model.is_connected) 
    