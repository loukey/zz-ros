"""
串口通信控制器
"""
from PyQt5.QtCore import QThread, pyqtSignal
from models.serial_model import SerialModel
from utils.command_utils import format_command
import time


class SerialController:
    """串口通信控制器，仅负责串口连接和原始命令发送"""
    
    def __init__(self):
        self.serial_model = SerialModel()
        self.read_thread = QThread()
        
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
    
    def send_control_command(self, 
                             joint_angles=[0.0] * 6, 
                             command_type='NONE', 
                             mode=0x08, 
                             contour_speed=[0.0] * 6, 
                             contour_acceleration=[0.0] * 6, 
                             contour_deceleration=[0.0] * 6, 
                             effector_mode=0x00, 
                             effector_data=0.0, 
                             encoding='string', 
                             return_cmd=False):
        """
        发送控制命令
        参数:
            joint_angles: 关节角度列表，包含6个关节角度
            command_type: 控制命令类型（如 'ENABLE', 'DISABLE' 等）
            encoding: 编码格式，'string' 或 'hex'
            mode: 运行模式
            return_cmd: 是否返回完整命令字符串
            
        返回:
            bool 或 (bool, str): 是否发送成功，或者发送成功与命令字符串的元组
        """
        # 检查串口连接状态
        if not hasattr(self.serial_model, 'is_connected') or not self.serial_model.is_connected:
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit("串口未连接")
            return (False, "") if return_cmd else False
        
        # 命令类型到控制字节的映射
        control_map = {
            'NONE': 0x00,      # 无
            'ENABLE': 0x01,    # 使能
            'DISABLE': 0x02,   # 取消使能
            'RELEASE': 0x03,   # 释放刹车
            'LOCK': 0x04,      # 锁止刹车
            'STOP': 0x05,      # 立刻停止
            'MOTION': 0x06,    # 运动状态
            'POSITION': 0x07,  # 获取当前位置
            'PAUSE': 0x08      # 暂停
        }
        
        # 获取控制字节
        control = control_map.get(command_type)
        
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
            
            # 记录命令内容
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit(f"准备发送命令: {cmd}")
            
            # 发送命令
            success = self.send_data(cmd, encoding=encoding)
            
            if not success:
                if hasattr(self, 'error_occurred'):
                    self.error_occurred.emit(f"发送命令失败: {command_type}")
                return (False, "") if return_cmd else False
            
            # 记录发送成功
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit(f"命令发送成功: {cmd}")
            
            if return_cmd:
                return True, cmd
            return True
            
        except Exception as e:
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit(f"发送控制命令失败: {str(e)}")
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
