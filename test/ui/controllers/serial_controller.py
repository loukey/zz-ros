"""
串口通信控制器
"""
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from models.serial_model import SerialModel
from utils.command_utils import format_command, calculate_crc16, generate_trajectory
import time


class TrajectoryThread(QThread):
    """轨迹发送线程，用于避免UI阻塞"""
    
    # 信号：发送轨迹点(索引, 总数, 命令字符串, 成功状态)
    point_sent = pyqtSignal(int, int, str, bool)
    # 信号：完成发送(成功状态, 命令字符串列表)
    finished = pyqtSignal(bool, list)
    
    def __init__(self, serial_model, angles, curve_type, duration, frequency, encoding, mode_value, control_byte):
        super().__init__()
        self.serial_model = serial_model
        self.angles = angles
        self.curve_type = curve_type
        self.duration = duration
        self.frequency = frequency
        self.encoding = encoding
        self.mode_value = mode_value
        self.control_byte = control_byte
        self.stop_flag = False
    
    def run(self):
        try:
            # 获取当前角度作为起始点（默认为全零）
            current_angles = [0.0] * 6
            
            # 生成轨迹
            time_points, position_data, _, _ = generate_trajectory(
                start_angles=current_angles,
                end_angles=self.angles,
                duration=self.duration,
                frequency=self.frequency,
                curve_type=self.curve_type
            )
            
            # 初始化成功标志和命令字符串
            success = True
            all_cmd_strs = []
            
            # 逐个点发送
            for i, t in enumerate(time_points):
                # 检查是否被要求停止
                if self.stop_flag:
                    break
                
                # 提取当前时间点的6个关节角度
                current_point = [position_data[j][i] for j in range(6)]
                
                # 构建当前点的命令
                cmd = format_command(current_point, control=self.control_byte, mode=self.mode_value)
                
                # 创建可读的命令字符串
                cmd_control = cmd[3]
                cmd_mode = cmd[4]
                
                # 提取6个位置值
                positions = []
                for j in range(6):
                    start_idx = 5 + j * 3
                    pos_value = (cmd[start_idx] << 16) | (cmd[start_idx + 1] << 8) | cmd[start_idx + 2]
                    positions.append(pos_value)
                
                # 提取CRC16值
                crc_value = (cmd[-4] << 8) | cmd[-3]
                
                # 组装完整命令字符串
                cmd_str = f"cmd {cmd_control:02d} {cmd_mode:02d}"
                for pos in positions:
                    cmd_str += f" {pos}"
                cmd_str += f" {crc_value:04X}\\r\\n"
                
                # 发送当前点
                point_success = False
                if self.encoding == 'string':
                    point_success = self.serial_model.send_command(cmd)
                else:  # hex
                    point_success = self.serial_model.send_command(cmd)
                
                # 记录命令字符串
                all_cmd_strs.append(cmd_str)
                
                # 发送信号，通知UI更新
                self.point_sent.emit(i+1, len(time_points), cmd_str, point_success)
                
                # 如果发送失败，整体标记为失败
                if not point_success:
                    success = False
                    break
                
                # 根据频率等待
                time.sleep(self.frequency)
            
            # 发送完成信号
            self.finished.emit(success, all_cmd_strs)
                
        except Exception as e:
            # 发送异常信号
            self.finished.emit(False, [])
    
    def stop(self):
        """停止发送"""
        self.stop_flag = True


class SerialController:
    """串口通信控制器，负责串口连接和命令发送的控制逻辑"""
    
    def __init__(self):
        self.serial_model = SerialModel()
        self.read_thread = QThread()
        self.trajectory_thread = None
        
        # 将串口模型移动到读取线程中
        self.serial_model.moveToThread(self.read_thread)
        self.read_thread.started.connect(self.serial_model.read_data)
    
    def get_available_ports(self):
        """
        获取可用的串口列表
        
        返回:
            list: 包含(port, description)元组的列表
        """
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
        
        return success
    
    def disconnect(self):
        """断开串口连接"""
        if self.read_thread.isRunning():
            self.serial_model.stop()
            self.read_thread.quit()
            self.read_thread.wait()
        
        self.serial_model.disconnect()
    
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
    
    def send_command(self, data):
        """
        发送命令数据
        
        参数:
            data: 要发送的数据（字节数组）
            
        返回:
            bool: 是否发送成功
        """
        return self.serial_model.send_command(data)
    
    def send_control_command(self, command_type, encoding='string', mode='01', return_cmd=False):
        """
        发送控制命令
        
        参数:
            command_type: 控制命令类型（如 'ENABLE', 'DISABLE' 等）
            encoding: 编码格式，'string' 或 'hex'
            mode: 运行模式
            return_cmd: 是否返回完整命令字符串
            
        返回:
            bool 或 (bool, str): 是否发送成功，或者发送成功与命令字符串的元组
        """
        # 命令类型到控制字节的映射
        control_map = {
            'ENABLE': 0x01,    # 使能
            'DISABLE': 0x02,   # 取消使能
            'RELEASE': 0x03,   # 释放刹车
            'LOCK': 0x04,      # 锁止刹车
            'STOP': 0x05,      # 立刻停止
            'MOTION': 0x06     # 运动状态
        }
        
        # 获取控制字节
        control = control_map.get(command_type)
        if control is None:
            # 记录错误
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit(f"未知的命令类型: {command_type}")
            return (False, "") if return_cmd else False
        
        # 转换模式字符串为整数
        mode_value = mode
        if isinstance(mode, str):
            # 如果是十六进制字符串，如"0x01"
            if '0x' in mode.lower():
                mode_value = int(mode, 16)
            # 如果是其他字符串，如"01"或数字字符串
            else:
                try:
                    mode_value = int(mode)
                except ValueError:
                    # 默认使用轮廓位置模式
                    mode_value = 1
        
        # 默认角度值
        angles = [0.0] * 6
        
        # 使用format_command构建命令并获取字节数组
        cmd_bytes = format_command(angles, control=control, mode=mode_value)
        
        # 创建可读的命令字符串
        # 提取控制字节和模式字节
        cmd_control = cmd_bytes[3]
        cmd_mode = cmd_bytes[4]
        
        # 提取6个位置值
        positions = []
        for i in range(6):
            start_idx = 5 + i * 3
            pos_value = (cmd_bytes[start_idx] << 16) | (cmd_bytes[start_idx + 1] << 8) | cmd_bytes[start_idx + 2]
            positions.append(pos_value)
        
        # 提取CRC16值
        crc_value = (cmd_bytes[-4] << 8) | cmd_bytes[-3]
        
        # 组装完整命令字符串
        cmd_str = f"cmd {cmd_control:02d} {cmd_mode:02d}"
        for pos in positions:
            cmd_str += f" {pos}"
        cmd_str += f" {crc_value:04X}\\r\\n"
        
        # 发送命令
        success = False
        if encoding == 'string':
            success = self.serial_model.send_command(cmd_bytes)
        else:  # hex
            success = self.serial_model.send_command(cmd_bytes)
        
        return (success, cmd_str) if return_cmd else success
    
    def send_angles(self, angles, curve_type="Trapezoid", duration=5.0, frequency=0.01, encoding="string", mode="normal", return_cmd=False):
        """
        发送角度命令
        
        参数:
            angles: 关节角度列表，包含6个关节角度
            curve_type: 曲线类型，默认为"Trapezoid"
            duration: 运动持续时间，默认5秒
            frequency: 采样频率，默认0.01秒
            encoding: 编码类型，默认为"string"
            mode: 运行模式，默认为"normal"
            return_cmd: 是否返回完整命令字符串
            
        返回:
            bool 或 (bool, str): 是否发送成功，或者发送成功与命令字符串的元组
        """
        if not hasattr(self.serial_model, 'is_connected') or not self.serial_model.is_connected:
            return (False, "") if return_cmd else False
        
        try:
            # 确保有6个角度值
            if len(angles) != 6:
                raise ValueError("角度值必须是6个")
            
            # 运行模式映射
            mode_map = {
                "normal": 0x01,  # 轮廓位置模式
                "motion": 0x08   # 周期同步位置模式
            }
            
            # 获取模式值
            mode_value = mode_map.get(mode, 0x01)
            
            # 如果是具体的模式编码（如'01', '08'等），尝试转换
            if isinstance(mode, str) and mode.isdigit():
                try:
                    mode_value = int(mode)
                except ValueError:
                    pass
            
            # 控制字节，对于角度发送使用运动状态(0x06)
            control_byte = 0x06
            
            # 根据控制字节和模式决定发送方式
            if control_byte == 0x06 and duration > 0 and frequency > 0:
                # 如果已经有正在运行的轨迹线程，停止它
                if self.trajectory_thread and self.trajectory_thread.isRunning():
                    self.trajectory_thread.stop()
                    self.trajectory_thread.wait()
                
                # 使用线程方式发送轨迹点
                self.trajectory_thread = TrajectoryThread(
                    serial_model=self.serial_model,
                    angles=angles,
                    curve_type=curve_type,
                    duration=duration,
                    frequency=frequency,
                    encoding=encoding,
                    mode_value=mode_value,
                    control_byte=control_byte
                )
                
                # 对于非阻塞调用，直接返回
                if not return_cmd:
                    self.trajectory_thread.start()
                    return True
                
                # 对于阻塞调用，等待线程完成并收集所有命令
                all_cmd_strs = []
                success = True
                
                # 连接信号
                def on_point_sent(index, total, cmd_str, point_success):
                    nonlocal all_cmd_strs
                    all_cmd_strs.append(cmd_str)
                
                def on_finished(traj_success, cmd_strs):
                    nonlocal success
                    success = traj_success
                
                self.trajectory_thread.point_sent.connect(on_point_sent)
                self.trajectory_thread.finished.connect(on_finished)
                
                # 启动线程并等待完成
                self.trajectory_thread.start()
                self.trajectory_thread.wait()
                
                return (success, all_cmd_strs)
            else:
                # 单点发送模式
                # 使用format_command构建二进制命令
                cmd = format_command(angles, control=control_byte, mode=mode_value)
                
                # 创建可读的命令字符串
                # 提取控制字节和模式字节
                cmd_control = cmd[3]
                cmd_mode = cmd[4]
                
                # 提取6个位置值
                positions = []
                for i in range(6):
                    start_idx = 5 + i * 3
                    pos_value = (cmd[start_idx] << 16) | (cmd[start_idx + 1] << 8) | cmd[start_idx + 2]
                    positions.append(pos_value)
                
                # 提取CRC16值
                crc_value = (cmd[-4] << 8) | cmd[-3]
                
                # 组装完整命令字符串
                cmd_str = f"cmd {cmd_control:02d} {cmd_mode:02d}"
                for pos in positions:
                    cmd_str += f" {pos}"
                cmd_str += f" {crc_value:04X}\\r\\n"
                
                # 发送命令
                success = False
                if encoding == 'string':
                    success = self.serial_model.send_command(cmd)
                else:  # hex
                    success = self.serial_model.send_command(cmd)
                
                return (success, cmd_str) if return_cmd else success
                
        except Exception as e:
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit(f"发送角度命令失败: {str(e)}")
            return (False, "") if return_cmd else False
    
    def register_data_received_callback(self, callback):
        """
        注册数据接收回调函数
        
        参数:
            callback: 回调函数，接收一个字符串参数
        """
        self.serial_model.data_received.connect(callback)
    
    def register_connection_changed_callback(self, callback):
        """
        注册连接状态变化回调函数
        
        参数:
            callback: 回调函数，接收一个布尔参数
        """
        self.serial_model.connection_changed.connect(callback)
    
    def register_error_occurred_callback(self, callback):
        """
        注册错误发生回调函数
        
        参数:
            callback: 回调函数，接收一个字符串参数
        """
        self.serial_model.error_occurred.connect(callback) 