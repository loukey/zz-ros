"""
串口通信模块
实现串口的基本操作和通信功能
"""

import serial
import serial.tools.list_ports
import time
from PyQt5.QtCore import QObject, pyqtSignal


class SerialComm(QObject):
    """串口通信类"""
    
    # 定义信号
    data_received = pyqtSignal(str)  # 接收到数据时发送信号
    connection_changed = pyqtSignal(bool)  # 连接状态改变时发送信号
    error_occurred = pyqtSignal(str)  # 发生错误时发送信号
    
    def __init__(self):
        super().__init__()
        self.serial = None
        self.is_connected = False
        self.read_thread = None
        self.stop_flag = False
    
    def get_available_ports(self):
        """
        获取可用的串口列表
        
        返回:
            list: 包含(port, description)元组的列表
        """
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append((port.device, port.description))
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
        try:
            if self.is_connected:
                self.disconnect()
            
            self.serial = serial.Serial(
                port=port,
                baudrate=baud_rate,
                bytesize=data_bits,
                parity=parity,
                stopbits=stop_bits,
                timeout=0.1
            )
            
            if flow_control:
                if flow_control == 'xonxoff':
                    self.serial.xonxoff = True
                elif flow_control == 'rtscts':
                    self.serial.rtscts = True
                elif flow_control == 'dsrdtr':
                    self.serial.dsrdtr = True
            
            self.is_connected = True
            self.stop_flag = False
            self.connection_changed.emit(True)
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"连接串口失败: {str(e)}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial and self.serial.is_open:
            self.stop_flag = True
            self.serial.close()
            self.is_connected = False
            self.connection_changed.emit(False)
    
    def send_data(self, data, encoding='string'):
        """
        发送数据
        
        参数:
            data: 要发送的数据
            encoding: 编码格式，'string' 或 'hex'
        
        返回:
            bool: 是否发送成功
        """
        if not self.is_connected:
            self.error_occurred.emit("串口未连接")
            return False
        
        try:
            if encoding == 'hex':
                # 将十六进制字符串转换为字节
                data = bytes.fromhex(data.replace(' ', ''))
            else:
                # 将字符串转换为字节
                data = data.encode()
            
            self.serial.write(data)
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"发送数据失败: {str(e)}")
            return False
    
    def read_data(self):
        """读取串口数据"""
        while not self.stop_flag:
            if self.serial and self.serial.is_open:
                try:
                    if self.serial.in_waiting:
                        data = self.serial.read(self.serial.in_waiting)
                        if data:
                            self.data_received.emit(data.decode())
                except Exception as e:
                    self.error_occurred.emit(f"读取数据失败: {str(e)}")
                    break
            time.sleep(0.01)
    
    def send_command(self, command):
        """
        发送控制命令
        
        参数:
            command: 命令字符串
        
        返回:
            bool: 是否发送成功
        """
        if not self.is_connected:
            self.error_occurred.emit("串口未连接")
            return False
        
        try:
            # 添加命令结束符
            command = command + '\r\n'
            return self.send_data(command)
            
        except Exception as e:
            self.error_occurred.emit(f"发送命令失败: {str(e)}")
            return False
    
    def send_angles(self, angles, duration, frequency, curve_type='trapezoidal'):
        """
        发送角度值
        
        参数:
            angles: 角度值列表
            duration: 运动时长
            frequency: 发送频率
            curve_type: 曲线类型，'trapezoidal' 或 's_curve'
        
        返回:
            bool: 是否发送成功
        """
        if not self.is_connected:
            self.error_occurred.emit("串口未连接")
            return False
        
        try:
            # 构建命令字符串
            command = f"AA55 01 {len(angles)}"
            for angle in angles:
                command += f" {angle:.6f}"
            command += f" {duration:.2f} {frequency:.2f} {curve_type}"
            
            return self.send_command(command)
            
        except Exception as e:
            self.error_occurred.emit(f"发送角度失败: {str(e)}")
            return False 