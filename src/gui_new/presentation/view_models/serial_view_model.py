"""
串口视图模型
"""
from typing import List, Tuple, Optional
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from application.dto.command_dto import SerialConfigDTO
from shared.config.container import ServiceLocator


class SerialViewModel(BaseViewModel):
    """串口视图模型"""
    
    # 串口特有信号
    connection_status_changed = pyqtSignal(bool)
    port_list_updated = pyqtSignal(list)
    data_received = pyqtSignal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # UI状态
        self.is_connected = False
        self.current_port = ""
        self.baudrate = 115200
        self.available_ports = []
        
        # 确保机器人实例存在
        self._ensure_robot_instance()
    
    def _ensure_robot_instance(self):
        """确保机器人实例存在"""
        try:
            # 检查是否已有机器人实例
            state = self.robot_service.get_robot_state()
            if not state:
                # 创建机器人实例
                self.robot_service.create_robot(
                    model_name="RobotArm_6DOF",
                    serial_number="RA6D001"
                )
        except Exception as e:
            self.emit_error(f"初始化机器人实例失败: {str(e)}")
    
    def connect_port(self, port: str) -> bool:
        """连接指定端口"""
        if not port:
            self.emit_error("请选择串口")
            return False
        
        try:
            # 创建串口配置DTO
            config_dto = SerialConfigDTO(
                port=port,
                baud_rate=self.baudrate,
                data_bits=8,
                parity='N',
                stop_bits=1
            )
            
            # 调用Application服务连接
            success, message = self.robot_service.connect_robot(config_dto)
            
            if success:
                self.is_connected = True
                self.current_port = port
                self.connection_status_changed.emit(True)
                self.emit_status(f"已连接到 {port}")
                return True
            else:
                self.emit_error(f"连接失败: {message}")
                return False
                
        except Exception as e:
            self.emit_error(f"连接失败: {str(e)}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        try:
            success, message = self.robot_service.disconnect_robot()
            
            if success:
                self.is_connected = False
                self.connection_status_changed.emit(False)
                self.emit_status("已断开连接")
            else:
                self.emit_error(f"断开连接失败: {message}")
                
        except Exception as e:
            self.emit_error(f"断开连接失败: {str(e)}")
    
    def set_port(self, port: str):
        """设置串口"""
        self.current_port = port
    
    def set_baudrate(self, baudrate: int):
        """设置波特率"""
        self.baudrate = baudrate
    
    def refresh_ports(self) -> List[Tuple[str, str]]:
        """刷新可用串口列表"""
        try:
            # TODO: 当Infrastructure层实现后，这里调用真实的串口扫描服务
            # 目前使用模拟数据
            import serial.tools.list_ports
            
            ports = []
            for port_info in serial.tools.list_ports.comports():
                ports.append((port_info.device, port_info.description or "Unknown"))
            
            if not ports:
                # 如果没有找到真实端口，提供一些模拟数据用于测试
                ports = [
                    ("COM1", "Serial Port 1"),
                    ("COM3", "USB Serial Port"),
                    ("COM4", "Bluetooth Serial Port")
                ]
            
            self.available_ports = ports
            self.port_list_updated.emit(ports)
            self.emit_status(f"找到 {len(ports)} 个可用串口")
            return ports
            
        except ImportError:
            # 如果没有安装pyserial，使用模拟数据
            ports = [
                ("COM1", "Serial Port 1"),
                ("COM3", "USB Serial Port"),
                ("COM4", "Bluetooth Serial Port")
            ]
            self.available_ports = ports
            self.port_list_updated.emit(ports)
            self.emit_status("使用模拟串口数据")
            return ports
            
        except Exception as e:
            self.emit_error(f"刷新串口列表失败: {str(e)}")
            return []
    
    def send_data(self, data: str):
        """发送数据"""
        if not self.is_connected:
            self.emit_error("串口未连接")
            return
        
        try:
            # TODO: 当Infrastructure层实现后，这里调用真实的数据发送服务
            # 目前仅记录日志
            self.emit_status(f"发送数据: {data}")
            
        except Exception as e:
            self.emit_error(f"发送数据失败: {str(e)}")
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def get_current_port(self) -> str:
        """获取当前端口"""
        return self.current_port
    
    def get_baudrate(self) -> int:
        """获取波特率"""
        return self.baudrate
    
    def cleanup(self):
        """清理资源"""
        if self.is_connected:
            self.disconnect() 