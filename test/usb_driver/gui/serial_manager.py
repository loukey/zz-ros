"""
串口通信管理模块
负责串口设备的连接、断开、数据收发等功能
"""

import serial
import serial.tools.list_ports
import threading
import time


class SerialManager:
    """串口通信管理类"""
    
    def __init__(self, message_callback=None):
        """
        初始化串口管理器
        
        参数:
            message_callback: 接收到消息时的回调函数，接收参数(message, message_type)
        """
        self.serial = None
        self.is_connected = False
        self.receiver_thread = None
        self.receiving = False
        self.message_callback = message_callback
    
    def get_available_ports(self):
        """
        获取所有可用串口
        
        返回:
            port_list: 包含(port, description)元组的列表
        """
        ports = serial.tools.list_ports.comports()
        port_list = []
        
        for port in ports:
            port_list.append((port.device, port.description))
        
        return port_list
    
    def connect(self, port, baud_rate, data_bits, parity, stop_bits, flow_control):
        """
        连接到指定串口
        
        参数:
            port: 串口名称
            baud_rate: 波特率
            data_bits: 数据位
            parity: 校验位
            stop_bits: 停止位
            flow_control: 流控制类型
            
        返回:
            success: 是否成功连接
            message: 连接结果消息
        """
        # 如果已经连接，先断开
        if self.is_connected and self.serial:
            self.disconnect()
        
        # 设置流控制参数
        xonxoff = False
        rtscts = False
        dsrdtr = False
        
        if flow_control == 'xonxoff':
            xonxoff = True
        elif flow_control == 'rtscts':
            rtscts = True
        elif flow_control == 'dsrdtr':
            dsrdtr = True
        
        # 尝试连接串口
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baud_rate,
                bytesize=data_bits,
                parity=parity,
                stopbits=stop_bits,
                xonxoff=xonxoff,
                rtscts=rtscts,
                dsrdtr=dsrdtr,
                timeout=1
            )
            
            # 判断串口是否成功打开
            if self.serial.is_open:
                self.is_connected = True
                
                # 构建连接信息
                info = f"已成功连接到串口 {port}"
                details = f"波特率: {baud_rate}, 数据位: {data_bits}, 校验位: {parity}, 停止位: {stop_bits}"
                
                # 启动接收线程
                self.start_receiver()
                
                # 进行简单的通信测试
                self.test_communication()
                
                return True, (info, details)
            else:
                return False, "串口无法打开，请检查设备或参数"
                
        except serial.SerialException as e:
            return False, f"无法连接到串口: {str(e)}"
        except Exception as e:
            return False, f"连接出错: {str(e)}"
    
    def disconnect(self):
        """
        断开当前连接的串口
        
        返回:
            success: 是否成功断开
            message: 断开结果消息
        """
        if not self.is_connected or not self.serial:
            return True, "当前未连接任何串口"
        
        try:
            # 停止接收线程
            self.stop_receiver()
            
            self.serial.close()
            self.is_connected = False
            self.serial = None
            
            return True, "串口已断开连接"
        except Exception as e:
            return False, f"断开连接时出错: {str(e)}"
    
    def send_data(self, data, is_binary=False):
        """
        发送数据到串口
        
        参数:
            data: 要发送的数据，字符串或字节
            is_binary: 是否为二进制数据
            
        返回:
            success: 是否成功发送
            message: 发送结果消息
        """
        if not self.is_connected or not self.serial:
            return False, "请先连接串口"
        
        try:
            # 如果不是二进制数据，则编码为字节
            if not is_binary and isinstance(data, str):
                data = data.encode()
            
            # 发送数据
            self.serial.write(data)
            
            return True, "数据发送成功"
        except Exception as e:
            return False, f"发送数据时出错: {str(e)}"
    
    def send_angles(self, angles):
        """
        发送角度数据到串口
        
        参数:
            angles: 角度列表
            
        返回:
            success: 是否成功发送
            message: 发送结果消息
        """
        try:
            # 构建要发送的数据格式
            data = ",".join([f"{angle:.6f}" for angle in angles]) + "\n"
            
            # 发送数据
            return self.send_data(data)
            
        except Exception as e:
            return False, f"发送角度数据时出错: {str(e)}"
    
    def test_communication(self):
        """
        测试串口通信
        
        返回:
            response: 测试响应数据，如果没有响应则为None
        """
        if not self.is_connected or not self.serial:
            return None
        
        try:
            # 测试写入数据
            test_data = b'\x00'  # 发送一个空字节作为测试
            self.serial.write(test_data)
            
            # 通知发送了测试数据
            if self.message_callback:
                self.message_callback("发送测试数据: 0x00", "发送")
            
            # 设置短超时尝试读取响应
            old_timeout = self.serial.timeout
            self.serial.timeout = 0.5
            
            response = None
            try:
                response = self.serial.read(1)  # 尝试读取一个字节
                if response and self.message_callback:
                    # 通知收到响应
                    hex_response = response.hex()
                    self.message_callback(f"测试响应: 0x{hex_response}", "接收")
                elif self.message_callback:
                    # 通知无响应
                    self.message_callback("测试无响应，设备未返回数据", "接收")
            except Exception as e:
                # 通知读取错误
                if self.message_callback:
                    self.message_callback(f"读取测试响应时出错: {str(e)}", "错误")
            
            # 恢复原来的超时设置
            self.serial.timeout = old_timeout
            
            return response
            
        except Exception as e:
            # 通知测试错误
            if self.message_callback:
                self.message_callback(f"测试通信时出错: {str(e)}", "错误")
            return None
    
    def start_receiver(self):
        """启动数据接收线程"""
        if self.receiving:
            return
            
        self.receiving = True
        self.receiver_thread = threading.Thread(target=self._receive_data, daemon=True)
        self.receiver_thread.start()
        
        if self.message_callback:
            self.message_callback("已启动数据接收", "信息")
    
    def stop_receiver(self):
        """停止数据接收线程"""
        self.receiving = False
        if self.receiver_thread:
            # 等待线程结束，最多等待1秒
            if self.receiver_thread.is_alive():
                self.receiver_thread.join(1.0)
            self.receiver_thread = None
            
        if self.message_callback:
            self.message_callback("已停止数据接收", "信息")
    
    def _receive_data(self):
        """接收串口数据的线程函数"""
        buffer = b''
        
        while self.receiving and self.is_connected and self.serial:
            try:
                # 读取可用的数据
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    if data:
                        # 将数据添加到缓冲区
                        buffer += data
                        
                        # 处理可能的完整消息
                        while b'\n' in buffer:
                            # 找到一个完整的行
                            line, buffer = buffer.split(b'\n', 1)
                            try:
                                # 尝试解码为字符串
                                line_str = line.decode('utf-8').strip()
                                # 通知收到消息
                                if self.message_callback:
                                    self.message_callback(line_str, "接收")
                            except UnicodeDecodeError:
                                # 如果无法解码为UTF-8，显示十六进制
                                hex_str = ' '.join([f'{b:02X}' for b in line])
                                if self.message_callback:
                                    self.message_callback(f"HEX: {hex_str}", "接收")
                
                # 短暂休眠，避免CPU占用过高
                time.sleep(0.01)
                
            except Exception as e:
                # 通知错误
                if self.message_callback:
                    self.message_callback(f"接收错误: {str(e)}", "错误")
                time.sleep(1)  # 出错后等待一段时间再继续 