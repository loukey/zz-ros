"""
串口通信数据模型
"""
import serial
import serial.tools.list_ports
import time
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import os
import glob
from gui.utils import *
from gui.config import *


class SerialModel(QObject):
    """串口通信模型类"""
    
    # 定义信号
    data_received = pyqtSignal(str)  # 接收到数据时发送信号
    error_occurred = pyqtSignal(str)  # 发生错误时发送信号
    
    def __init__(self):
        super().__init__()
        self.serial = None
        self.is_connected = False
        self.stop_flag = False
        self.buffer = ""  # 添加缓冲区用于累积数据
        self.encoding = 'hex'  # 默认使用string编码
        self.auto_detect = True  # 是否自动检测数据格式
    
    def set_encoding(self, encoding):
        """
        设置编码格式
        
        参数:
            encoding: 编码格式，'string' 或 'hex'
        """
        self.encoding = encoding
        self.buffer = ""  # 重置缓冲区
    
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
            # 先测试串口是否可用
            test_serial = serial.Serial()
            test_serial.port = port
            test_serial.open()
            test_serial.close()
            
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
                return True
            
            return False
            
        except serial.SerialException as e:
            self.error_occurred.emit(f"串口被占用或无法访问: {str(e)}")
            return False
        except Exception as e:
            self.error_occurred.emit(f"连接失败: {str(e)}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
        return True
    
    def stop(self):
        """停止读取数据"""
        self.stop_flag = True
    
    def read_data(self):
        """读取串口数据"""
        while not self.stop_flag:
            try:
                if not self.serial or not self.serial.is_open:
                    time.sleep(0.1)
                    continue

                # 使用更安全的方式检查串口状态
                try:
                    waiting_bytes = self.serial.in_waiting
                except (serial.SerialException, AttributeError) as e:
                    # 如果检查失败，尝试重新打开串口
                    self.error_occurred.emit(f"检查串口状态时出错，尝试重新连接: {str(e)}")
                    try:
                        if self.serial:
                            continue
                        else:
                            self.serial.close()
                            time.sleep(0.1)
                            self.serial.open()
                    except:
                        self.disconnect()
                        break

                if waiting_bytes > 0:
                    try:
                        # 一次最多读取1024字节，避免缓冲区溢出
                        bytes_to_read = min(waiting_bytes, 1024)
                        data = self.serial.read(bytes_to_read)
                        
                        if data:                      
                            if self.encoding == 'hex':
                                # 十六进制格式：将字节转换为十六进制字符串
                                hex_data = data.hex().upper()
                                self.data_received.emit(hex_data)
                            else:
                                # 字符串格式：解码为UTF-8字符串
                                decoded_data = data.decode('utf-8', errors='ignore')
                                if '\r\n' in decoded_data:
                                    self.buffer += decoded_data
                                    lines = self.buffer.split('\r\n')
                                    # 发送完整的行
                                    for line in lines[:-1]:
                                        if line:  # 只发送非空行
                                            self.data_received.emit(line)
                                    self.buffer = lines[-1]
                                else:
                                    self.buffer += decoded_data
                    except serial.SerialException as e:
                        self.error_occurred.emit(f"读取数据时出错: {str(e)}")
                        # 不要立即断开，给一次重试机会
                        time.sleep(0.1)
                        continue
            except Exception as e:
                self.error_occurred.emit(f"串口读取时发生未知错误: {str(e)}")
                time.sleep(0.1)
                continue
            
            # 添加短暂延时，避免过于频繁的访问
            time.sleep(0.01)
    
    def send_data(self, data, encoding='hex'):
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

    def send_control_command(self, 
                             joint_angles=[0.0] * 6, 
                             control=0x00, 
                             mode=0x08, 
                             contour_speed=[0.0] * 6, 
                             contour_acceleration=[0.0] * 6, 
                             contour_deceleration=[0.0] * 6, 
                             torque=[0.0] * 6,
                             effector_mode=0x00, 
                             effector_data=0.0, 
                             encoding='hex', 
                             return_cmd=False):
        cmd = format_command(joint_angles=joint_angles, 
                        control=control, 
                        mode=mode, 
                        contour_speed=contour_speed, 
                        contour_acceleration=contour_acceleration, 
                        contour_deceleration=contour_deceleration, 
                        torque=torque,
                        effector_mode=effector_mode, 
                        effector_data=effector_data, 
                        encoding=encoding)
        success = self.send_data(cmd, encoding=encoding)
        GlobalVars.set_temp_cmd(cmd)
        if return_cmd:
            return success, cmd
        return success
