"""
串口通信控制器
"""
from PyQt5.QtCore import QThread
from models import *
from .utils import *


class SerialController:
    def __init__(self, display_callback, update_connection_status_callback):
        self.display_callback = display_callback
        self.update_connection_status_callback = update_connection_status_callback
        self.serial_model = SerialModel()
        self.motion_model = MotionModel()
        self.read_thread = QThread()
        
        # 将串口模型移动到读取线程中
        self.serial_model.moveToThread(self.read_thread)
        self.read_thread.started.connect(self.serial_model.read_data)
        self.serial_model.data_received.connect(self.handle_data_received)
        self.serial_model.connection_changed.connect(self.update_connection_status_callback)
        self.serial_model.error_occurred.connect(self.handle_error_occurred)
    
    def refresh_ports(self):
        return self.serial_model.get_available_ports()

    def connect(self, port, baud_rate=115200, data_bits=8, parity='N', stop_bits=1, flow_control=None):
        """
        连接到串口
        
        参数:
            port: 串口设备名称
            baud_rate: 波特率
            data_bits: 数据位
            parity: 校验位
            stop_bits: 停止位
            flow_control: 流控制
        
        返回:
            bool: 是否连接成功
        """
        config_str = f"波特率:{baud_rate}, 数据位:{data_bits}, 校验位:{parity}, 停止位:{stop_bits}, 流控制:{flow_control}"
        self.display_callback(config_str, "参数")
        success = self.serial_model.connect(
            port=port,
            baud_rate=baud_rate,
            data_bits=data_bits,
            parity=parity,
            stop_bits=stop_bits,
            flow_control=flow_control
        )
        
        if success:
            # 启动读取线程
            self.read_thread.start()
            self.display_callback(f"已连接到串口 {port}", "串口")
            self.update_connection_status_callback(True)
        else:
            self.display_callback("连接串口失败", "错误")
        
        return success
    
    def disconnect(self):
        """断开串口连接"""
        if self.read_thread.isRunning():
            self.serial_model.stop()
            self.read_thread.quit()
            self.read_thread.wait()
        
        self.serial_model.disconnect()
        self.update_connection_status_callback(False)
        self.display_callback("已断开串口连接", "串口")

    def send_data(self, data, encoding='string'):
        """
        发送数据
        
        参数:
            data: 要发送的数据
            encoding: 编码格式，'string' 或 'hex'
        
        返回:
            bool: 是否发送成功
        """
        return self.serial_model.send_data(data, encoding)
    
    def send_control_command(self, 
                             joint_angles=[0.0] * 6, 
                             control=0x00, 
                             mode=0x08, 
                             contour_speed=[0.0] * 6, 
                             contour_acceleration=[0.0] * 6, 
                             contour_deceleration=[0.0] * 6, 
                             effector_mode=0x00, 
                             effector_data=0.0, 
                             encoding='string', 
                             return_cmd=False):

        try:
            cmd = format_command(joint_angles=joint_angles, 
                                 control=control, 
                                 mode=mode, 
                                 contour_speed=contour_speed, 
                                 contour_acceleration=contour_acceleration, 
                                 contour_deceleration=contour_deceleration, 
                                 effector_mode=effector_mode, 
                                 effector_data=effector_data, 
                                 encoding=encoding)
            
            # 发送命令
            success = self.send_data(cmd, encoding=encoding)
            
            if not success:
                self.display_callback(f"发送命令失败: control: {control}, mode: {mode}", "错误")
                return (False, "") if return_cmd else False
            if return_cmd:
                return True, cmd
            return True
            
        except Exception as e:
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit(f"发送控制命令失败: {str(e)}")
            return (False, "") if return_cmd else False
    
    def handle_data_received(self, data):
        """处理接收到的数据"""
        clean_data = data.strip()        
        self.buffer += clean_data
        if "0D0A" in self.buffer:
            lines = self.buffer.split("0D0A")
            self.buffer = lines[-1]
            command_line = lines[0]
            self.display_callback(f"接收数据: '{command_line}'", "接收")
            
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
                    for _ in range(6):
                        pos = int(command_line[start:start+6], 16)
                        positions.append(pos)
                        start += 6
                    
                    # 状态字1-6 (每个2字节，共12字节)
                    status = []
                    for _ in range(6):
                        stat = command_line[start:start+4]
                        status.append(stat)
                        start += 4
                    
                    # 实际速度1-6 (每个4字节，24字节)
                    speeds = []
                    for _ in range(6):
                        speed = int(command_line[start:start+8], 16)
                        if speed & 0x80000000:
                            speed = speed - 0x100000000
                        speeds.append(speed)
                        start += 8
                    
                    # # 错误码1-6 (每个2字节，共12字节)
                    # errors = []
                    # for i in range(6):
                    #     error = command_line[start:start+4]
                    #     errors.append(error)
                    #     start += 4
                    
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
                        self.display_callback(f"CRC校验: 正确 (接收: {crc}, 计算: {calculated_crc_hex})", "接收")
                    else:
                        self.display_callback(f"CRC校验: 错误 (接收: {crc}, 计算: {calculated_crc_hex})", "错误")
                    
                    # 记录解析后的详细信息
                    return_msg = f"帧头: {header}, 初始状态: {init_status}, 当前命令: {current_command}, 运行模式: {run_mode}, 位置数据: {positions}, 状态字: {status}, 实际速度: {speeds}, 夹爪数据: {effector_data}, CRC16: {crc}"
                    self.display_callback(return_msg, "接收")
                    if current_command == "07":
                        self.display_callback("收到当前位置响应", "控制")
                        self.main_window.motion_handler.process_differential_motion(positions)
                        return

                except Exception as e:
                    self.display_callback(f"解析AA55数据帧失败: {str(e)}", "错误")
    
    def handle_error_occurred(self, error_msg):
        self.display_callback(f"{error_msg}", "错误")
