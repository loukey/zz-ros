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
from queue import Queue


class SerialReader(QObject):
    """串口读取处理类"""
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, serial_port, encoding='hex'):
        super().__init__()
        self.serial_port = serial_port
        self.encoding = encoding
        self.stop_flag = False
        self.buffer = ""
    
    def set_encoding(self, encoding):
        self.encoding = encoding
    
    def stop(self):
        self.stop_flag = True
    
    def read_data(self):
        """读取串口数据"""
        self.stop_flag = False
        while not self.stop_flag:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.1)
                continue
            try:
                waiting_bytes = self.serial_port.in_waiting
            except OSError:
                self.error_occurred.emit("OS Error")
                time.sleep(0.1)
                continue
            if waiting_bytes > 0:
                try:
                    data = self.serial_port.read(waiting_bytes)
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
                    time.sleep(0.1)
                    continue
            else:
                time.sleep(0.01)
            time.sleep(0.01)


class SerialSender(QObject):
    """串口发送处理类"""
    error_occurred = pyqtSignal(str)
    
    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.send_queue = Queue()
        self.stop_flag = False
    
    def stop(self):
        self.stop_flag = True
    
    def add_to_queue(self, cmd):
        """添加命令到发送队列"""
        self.send_queue.put(cmd)
    
    def send_data(self):
        """发送串口数据"""
        self.stop_flag = False
        while not self.stop_flag:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    time.sleep(0.1)
                    continue
                    
                if not self.send_queue.empty():
                    try:
                        cmd = self.send_queue.get()
                        if isinstance(cmd, str):
                            cmd = bytes.fromhex(cmd.replace(' ', ''))
                        self.serial_port.write(cmd)
                        self.serial_port.flush()
                        self.send_queue.task_done()
                    except Exception as e:
                        self.error_occurred.emit(f"发送数据时出错: {str(e)}")
                else:
                    # 队列为空时短暂休眠，避免CPU空转
                    time.sleep(0.01)
            except Exception as e:
                self.error_occurred.emit(f"串口发送时发生未知错误: {str(e)}")
                time.sleep(0.1)


class SerialModel(QObject):
    """串口通信模型类"""
    
    # 定义信号
    data_received = pyqtSignal(str)  # 接收到数据时发送信号
    error_occurred = pyqtSignal(str)  # 发生错误时发送信号
    
    def __init__(self):
        super().__init__()
        self.serial = None
        self.is_connected = False
        self.encoding = 'hex'
        
        # 创建读取和发送处理对象
        self.read_handler = None
        self.send_handler = None

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
        if os.name == 'posix':  
            pty_devices = glob.glob('/dev/pts/*')
            for device in pty_devices:
                if os.path.basename(device).isdigit() and not any(device == port[0] for port in ports):
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
                timeout=0.1  # 设置读取超时时间
            )
            
            if self.serial.is_open:
                self.is_connected = True
                
                # 创建读取和发送处理对象
                self.read_handler = SerialReader(self.serial, self.encoding)
                self.send_handler = SerialSender(self.serial)
                
                # 连接信号
                self.read_handler.data_received.connect(self.data_received.emit)
                self.read_handler.error_occurred.connect(self.error_occurred.emit)
                self.send_handler.error_occurred.connect(self.error_occurred.emit)
                
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
        self.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.is_connected = False
        return True
    
    def stop(self):
        """停止读取和发送"""
        if self.read_handler:
            self.read_handler.stop()
        if self.send_handler:
            self.send_handler.stop()
    
    def read_data(self):
        """读取串口数据 - 委托给reader"""
        if self.read_handler:
            self.read_handler.read_data()
    
    def send_data(self):
        """发送串口数据 - 委托给sender"""
        if self.send_handler:
            self.send_handler.send_data()

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
        if not self.send_handler:
            self.error_occurred.emit("串口未连接")
            return (False, "") if return_cmd else False
            
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
        
        self.send_handler.add_to_queue(cmd)
        GlobalVars.set_temp_cmd(cmd)
        
        if return_cmd:
            return True, cmd
        else:
            return True
