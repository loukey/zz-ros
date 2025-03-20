"""
串口通信控制器
"""
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from gui.models.serial_model import SerialModel
from gui.utils.command_utils import format_command, calculate_crc16, generate_trajectory
import time


class TrajectoryThread(QThread):
    """轨迹发送线程，用于避免UI阻塞"""
    
    # 信号：发送轨迹点(索引, 总数, 命令字符串, 成功状态)
    point_sent = pyqtSignal(int, int, str, bool)
    # 信号：完成发送(成功状态, 命令字符串列表)
    finished = pyqtSignal(bool, list)
    
    def __init__(self, serial_model, angles, curve_type, duration, frequency, encoding, mode_value, control_byte, start_angles=None):
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
        self.start_angles = start_angles
    
    def run(self):
        try:
            # 获取当前角度作为起始点（默认为全零）
            current_angles = [0.0] * 6
            
            # 检查是否已经被要求停止
            if self.stop_flag:
                # 提前终止
                self.finished.emit(False, [])
                return
            
            # 生成轨迹
            time_points, position_data, _, _ = generate_trajectory(
                start_angles=self.start_angles if self.start_angles else current_angles,
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
                # 每次发送前检查是否被要求停止
                if self.stop_flag:
                    # 发送已处理点的部分完成信号
                    self.finished.emit(False, all_cmd_strs)
                    return
                
                # 提取当前时间点的6个关节角度
                current_point = [position_data[j][i] for j in range(6)]
                
                # 直接使用format_command获取所需格式的命令
                formatted_cmd = format_command(
                    current_point, 
                    control=self.control_byte, 
                    mode=self.mode_value,
                    result_type=self.encoding
                )
                
                # 再次检查是否被要求停止
                if self.stop_flag:
                    # 发送已处理点的部分完成信号
                    self.finished.emit(False, all_cmd_strs)
                    return
                
                # 发送命令
                point_success = self.serial_model.send_data(formatted_cmd, encoding=self.encoding)
                
                # 获取可读的命令字符串用于显示和记录
                if self.encoding == 'string':
                    # 对于string格式，可以直接使用，但去掉末尾的\r\n
                    cmd_str = formatted_cmd[:-2] if isinstance(formatted_cmd, str) else formatted_cmd
                else:
                    # 获取一个可读的字符串格式(不包含\r\n)以便显示
                    cmd_str = format_command(
                        current_point, 
                        control=self.control_byte, 
                        mode=self.mode_value,
                        result_type='string'
                    )[:-2]
                
                # 记录命令字符串
                all_cmd_strs.append(cmd_str)
                
                # 发送信号，通知UI更新
                self.point_sent.emit(i+1, len(time_points), cmd_str, point_success)
                
                # 如果发送失败，整体标记为失败
                if not point_success:
                    success = False
                    break
                
                # 根据频率等待，但每10毫秒检查一次停止标志
                remaining_time = self.frequency
                while remaining_time > 0 and not self.stop_flag:
                    sleep_time = min(0.01, remaining_time)  # 最多等待10毫秒
                    time.sleep(sleep_time)
                    remaining_time -= sleep_time
                
                # 最后检查一次停止标志
                if self.stop_flag:
                    # 发送已处理点的部分完成信号
                    self.finished.emit(False, all_cmd_strs)
                    return
            
            # 发送完成信号
            self.finished.emit(success, all_cmd_strs)
                
        except Exception as e:
            # 发送异常信号
            self.finished.emit(False, [])
    
    def stop(self):
        """停止发送"""
        self.stop_flag = True
        
        # 在设置标志后等待一小段时间确保线程看到标志
        time.sleep(0.05)
        
        # 如果线程仍在运行，尝试更强硬的终止方式
        if self.isRunning():
            # 清空串口缓冲区
            if self.serial_model and self.serial_model.serial and self.serial_model.serial.is_open:
                try:
                    # 清空输出缓冲区
                    self.serial_model.serial.reset_output_buffer()
                except:
                    pass


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
        time.sleep(2)
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
    
    def send_control_command(self, command_type, encoding='string', mode=0x08, return_cmd=False):
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
            'MOTION': 0x06,    # 运动状态
            'PAUSE': 0x08,     # 暂停
            'POSITION': 0x07   # 获取当前位置
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
        
        # 直接使用format_command获取所需格式的命令
        cmd = format_command(angles, control=control, mode=mode_value, result_type=encoding)
        
        # 发送命令并获取成功标志
        success = self.serial_model.send_data(cmd, encoding=encoding)
        
        # 对于返回值处理，我们需要一个可读的命令字符串(不含\r\n)
        if return_cmd:
            # 获取不带\r\n的可读命令字符串用于返回和显示
            if encoding == 'string':
                # 对于string格式，去掉末尾的\r\n
                readable_cmd = cmd[:-2] if isinstance(cmd, str) else cmd
            else:
                # 获取一个可读的字符串格式(不包含\r\n)
                readable_cmd = format_command(angles, control=control, mode=mode_value, result_type='string')[:-2]
            
            return (success, readable_cmd)
        
        return success
    
    def send_angles(self, angles, curve_type="Trapezoid", duration=5.0, frequency=0.01, encoding="string", mode=0x08, return_cmd=False, start_angles=None):
        """
        发送角度命令
        
        参数:
            angles: 关节角度列表，包含6个关节角度
            curve_type: 曲线类型，默认为"Trapezoid"
            duration: 运动持续时间，默认5秒
            frequency: 采样频率，默认0.01秒
            encoding: 编码类型，默认为"string"
            mode: 运行模式，默认为0x08
            return_cmd: 是否返回完整命令字符串
            start_angles: 起始角度列表，如果提供则使用差分运动
            
        返回:
            bool 或 (bool, str): 是否发送成功，或者发送成功与命令字符串的元组
        """
        if not hasattr(self.serial_model, 'is_connected') or not self.serial_model.is_connected:
            return (False, "") if return_cmd else False
        
        try:
            # 确保有6个角度值
            if len(angles) != 6:
                raise ValueError("角度值必须是6个")
            
            # 如果提供了起始角度，确保也是6个
            if start_angles and len(start_angles) != 6:
                raise ValueError("起始角度值必须是6个")
            
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
                    control_byte=control_byte,
                    start_angles=start_angles  # 传入起始角度
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
                # 直接使用format_command获取所需格式的命令
                cmd = format_command(angles, control=control_byte, mode=mode_value, result_type=encoding)
                
                # 发送命令并获取成功标志
                success = self.serial_model.send_data(cmd, encoding=encoding)
                
                # 对于返回值处理，我们需要一个可读的命令字符串(不含\r\n)
                if return_cmd:
                    # 获取不带\r\n的可读命令字符串用于返回和显示
                    if encoding == 'string':
                        # 对于string格式，去掉末尾的\r\n
                        readable_cmd = cmd[:-2] if isinstance(cmd, str) else cmd
                    else:
                        # 获取一个可读的字符串格式(不包含\r\n)
                        readable_cmd = format_command(angles, control=control_byte, mode=mode_value, result_type='string')[:-2]
                    
                    return (success, readable_cmd)
                
                return success
                
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
    
    def prepare_trajectory(self, angles, curve_type="Trapezoid", duration=5.0, frequency=0.01, encoding="string", mode="normal", start_angles=None):
        """
        准备轨迹发送线程但不启动它
        
        参数:
            angles: 关节角度列表，包含6个关节角度
            curve_type: 曲线类型，默认为"Trapezoid"
            duration: 运动持续时间，默认5秒
            frequency: 采样频率，默认0.01秒
            encoding: 编码类型，默认为"string"
            mode: 运行模式，默认为"normal"
            start_angles: 起始角度列表，如果提供则使用差分运动，否则使用零点作为起始
            
        返回:
            bool: 是否准备成功
        """
        if not hasattr(self.serial_model, 'is_connected') or not self.serial_model.is_connected:
            return False
        
        try:
            # 确保有6个角度值
            if len(angles) != 6:
                raise ValueError("目标角度值必须是6个")
            
            # 如果提供了起始角度，确保也是6个
            if start_angles and len(start_angles) != 6:
                raise ValueError("起始角度值必须是6个")
            
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
            
            # 控制字节，对于角度发送使用运动状态(0x06)
            control_byte = 0x06
            
            # 如果已经有正在运行的轨迹线程，停止它
            if self.trajectory_thread and self.trajectory_thread.isRunning():
                self.trajectory_thread.stop()
                self.trajectory_thread.wait()
            
            # 创建轨迹线程，传入起始角度参数
            self.trajectory_thread = TrajectoryThread(
                serial_model=self.serial_model,
                angles=angles,
                curve_type=curve_type,
                duration=duration,
                frequency=frequency,
                encoding=encoding,
                mode_value=mode_value,
                control_byte=control_byte,
                start_angles=start_angles  # 传入起始角度
            )
            
            # 不启动线程，由调用者决定何时启动
            return True
                
        except Exception as e:
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit(f"准备轨迹线程失败: {str(e)}")
            return False 