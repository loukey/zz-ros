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
    
    def read_data(self):
        """读取串口数据"""
        while not self.stop_flag:
            if self.serial and self.serial.is_open:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                if data:
                    # 直接解码为UTF-8字符串，忽略错误的字符
                    decoded_data = data.decode('utf-8', errors='ignore')
                    if "\r\n" in decoded_data:
                        self.buffer += decoded_data
                        lines = self.buffer.split("\r\n")
                        self.buffer = lines[-1]
                        self.data_received.emit(lines[0])
                        
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
