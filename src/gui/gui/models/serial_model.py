"""
串口通信数据模型
"""
import serial
import serial.tools.list_ports
import time
from PyQt5.QtCore import QObject, pyqtSignal
import os
import glob


class SerialModel(QObject):
    """串口通信模型类"""
    
    # 定义信号
    data_received = pyqtSignal(str)  # 接收到数据时发送信号
    connection_changed = pyqtSignal(bool)  # 连接状态改变时发送信号
    error_occurred = pyqtSignal(str)  # 发生错误时发送信号
    
    def __init__(self):
        super().__init__()
        self.serial = None
        self.is_connected = False
        self.stop_flag = False
        self.buffer = ""  # 添加缓冲区用于累积数据
        self.last_buffer_time = 0  # 记录上次处理缓冲区的时间
        self.buffer_timeout = 0.2  # 缓冲区超时时间（秒）
    
    def get_available_ports(self):
        """
        获取可用的串口列表
        
        返回:
            list: 包含(port, description)元组的列表
        """
        ports = []
        
        # 获取标准串口设备
        for port in serial.tools.list_ports.comports():
            ports.append((port.device, port.description))
        
        # 在Linux系统上，检查虚拟终端(PTY)设备
        if os.name == 'posix':  # 仅在类Unix系统上执行
            # 获取/dev/pts/下的虚拟终端设备
            pty_devices = glob.glob('/dev/pts/*')
            for device in pty_devices:
                # 跳过非数字命名的设备，通常只考虑数字命名的pts
                if os.path.basename(device).isdigit():
                    # 如果还没有添加到列表中
                    if not any(device == port[0] for port in ports):
                        ports.append((device, f"虚拟串口 ({device})"))
        
        return ports
    
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
        # 如果已经连接，先断开
        if self.is_connected and self.serial:
            self.disconnect()
        
        try:
            # 配置流控制
            xonxoff = False
            rtscts = False
            dsrdtr = False
            
            if flow_control == 'xonxoff':
                xonxoff = True
            elif flow_control == 'rtscts':
                rtscts = True
            elif flow_control == 'dsrdtr':
                dsrdtr = True
            
            # 创建并打开串口
            self.serial = serial.Serial(
                port=port,
                baudrate=baud_rate,
                bytesize=data_bits,
                parity=parity,
                stopbits=stop_bits,
                xonxoff=xonxoff,
                rtscts=rtscts,
                dsrdtr=dsrdtr,
                timeout=0.5  # 设置读取超时时间
            )
            
            if self.serial.is_open:
                self.is_connected = True
                self.stop_flag = False
                self.connection_changed.emit(True)
                return True
            
            return False
            
        except Exception as e:
            self.error_occurred.emit(f"连接失败: {str(e)}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        self.stop_flag = True
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        self.is_connected = False
        self.connection_changed.emit(False)
    
    def stop(self):
        """停止读取数据"""
        self.stop_flag = True
    
    def process_buffer(self, force=False):
        """处理缓冲区中的数据，将完整的行发送出去"""
        current_time = time.time()
        
        # 如果缓冲区为空，直接返回
        if not self.buffer:
            return
            
        # 按\r\n分割缓冲区中的数据
        lines = self.buffer.split('\r\n')
        
        # 如果有完整的行（lines长度大于1）或强制处理
        if len(lines) > 1 or (force and current_time - self.last_buffer_time > self.buffer_timeout):
            # 如果强制处理但没有完整行，则视当前缓冲区为一个完整行
            if force and len(lines) == 1 and current_time - self.last_buffer_time > self.buffer_timeout:
                if self.buffer.strip():  # 只处理非空内容
                    self.data_received.emit(self.buffer)
                self.buffer = ""  # 清空缓冲区
                self.last_buffer_time = current_time
                return
                
            # 处理完整行
            for line in lines[:-1]:
                if line.strip():  # 只发送非空行
                    self.data_received.emit(line)
            
            # 保留最后一个可能不完整的行到缓冲区
            self.buffer = lines[-1]
            self.last_buffer_time = current_time
    
    def read_data(self):
        """读取串口数据"""
        error_count = 0  # 错误计数器
        max_errors = 3   # 最大允许连续错误次数
        
        while not self.stop_flag:
            if self.serial and self.serial.is_open:
                try:
                    if self.serial.in_waiting:
                        data = self.serial.read(self.serial.in_waiting)
                        if data:
                            # 直接解码为UTF-8字符串，忽略错误的字符
                            decoded_data = data.decode('utf-8', errors='ignore')
                            
                            # 将新数据添加到缓冲区
                            self.buffer += decoded_data
                            
                            # 处理缓冲区
                            self.process_buffer()
                            
                            # 成功读取数据，重置错误计数
                            error_count = 0
                    else:
                        # 检查是否需要强制处理长时间未完成的缓冲区
                        self.process_buffer(force=True)
                            
                except serial.SerialException as e:
                    # 串口异常（如设备断开连接）
                    error_count += 1
                    self.error_occurred.emit(f"串口异常: {str(e)}")
                    
                    if "Input/output error" in str(e):
                        # 对于I/O错误，尝试先等待一会儿
                        time.sleep(0.5)
                        if error_count > max_errors:
                            self.error_occurred.emit("串口发生多次I/O错误，尝试重新连接...")
                            # 尝试重新打开串口
                            try:
                                if self.serial and self.serial.is_open:
                                    port = self.serial.port
                                    settings = {
                                        'baudrate': self.serial.baudrate,
                                        'bytesize': self.serial.bytesize,
                                        'parity': self.serial.parity,
                                        'stopbits': self.serial.stopbits,
                                        'xonxoff': self.serial.xonxoff,
                                        'rtscts': self.serial.rtscts,
                                        'dsrdtr': self.serial.dsrdtr,
                                        'timeout': self.serial.timeout
                                    }
                                    self.serial.close()
                                    time.sleep(1)  # 等待一会儿
                                    self.serial.open()
                                    error_count = 0  # 重置错误计数
                                    self.error_occurred.emit("串口已重新连接")
                            except Exception as reopen_e:
                                self.error_occurred.emit(f"重新连接失败: {str(reopen_e)}")
                                self.disconnect()
                                break
                    else:
                        # 其他串口异常，如果连续出现多次，断开连接
                        if error_count > max_errors:
                            self.error_occurred.emit("串口多次出现异常，断开连接")
                            self.disconnect()
                            break
                
                except Exception as e:
                    # 其他异常
                    error_count += 1
                    self.error_occurred.emit(f"读取数据失败: {str(e)}")
                    
                    # 如果连续出现多次错误，断开连接
                    if error_count > max_errors:
                        self.error_occurred.emit("多次读取失败，断开连接")
                        self.disconnect()
                        break
                    
                    # 出错后等待一段时间再继续
                    time.sleep(0.2)
            
            # 如果没有错误，正常延时
            if error_count == 0:
                time.sleep(0.01)
            else:
                # 有错误时增加延时，避免频繁尝试导致更多错误
                time.sleep(0.1)
    
    def send_data(self, data, encoding='string'):
        """
        发送数据
        
        参数:
            data: 要发送的数据
            encoding: 编码格式，'string' 或 'hex'
        
        返回:
            bool: 是否发送成功
        """
        if not self.is_connected or not self.serial or not self.serial.is_open:
            self.error_occurred.emit("串口未连接")
            return False
        
        try:
            if encoding == 'hex':
                # 将十六进制字符串转换为字节
                if isinstance(data, str):
                    data = bytes.fromhex(data.replace(' ', ''))
            else:
                # 将字符串转换为字节，确保添加\r\n结束符
                if isinstance(data, str):
                    if not data.endswith('\r\n'):
                        data += '\r\n'
                    data = data.encode('utf-8')
            
            # 发送数据
            self.serial.write(data)
            
            # 确保数据被发送
            self.serial.flush()
            
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"发送数据失败: {str(e)}")
            return False 