"""
串口视图模型 - Presentation层
纯信号转发层，不包含业务逻辑
"""
from PyQt5.QtCore import pyqtSignal
from typing import List, Dict, Any, Optional
from presentation.view_models.base_view_model import BaseViewModel
from application.services.serial_service import SerialService


class SerialViewModel(BaseViewModel):
    """串口视图模型 - 纯信号转发层"""
    
    # UI状态信号定义
    port_list_updated = pyqtSignal(list)  # 端口列表更新
    connection_status_changed = pyqtSignal(bool)  # 连接状态变化
    data_received = pyqtSignal(str)  # 数据接收
    
    def __init__(self, serial_service: SerialService, parent=None):
        """初始化视图模型 - 纯信号转发"""
        super().__init__(parent)
        
        # 通过依赖注入接收应用服务
        self.serial_service = serial_service
        
        # 连接Service信号到ViewModel信号 - 纯转发
        self._connect_service_signals()
    
    # ===== UI命令方法 - 纯转发 =====
    
    def refresh_ports(self) -> None:
        """刷新端口列表命令 - 直接转发到Service"""
        self.serial_service.refresh_ports()
    
    def connect_serial(self, port: str, config: Dict[str, Any]) -> None:
        """连接串口命令 - 直接转发到Service"""
        self.serial_service.connect_serial(port, config)
    
    def disconnect_serial(self) -> None:
        """断开连接命令 - 直接转发到Service"""
        self.serial_service.disconnect_serial()
    
    def send_data(self, data: str) -> None:
        """发送数据命令 - 直接转发到Service"""
        self.serial_service.send_data(data)
    
    # ===== UI状态查询 - 委托给Service =====
    
    def get_available_ports(self) -> List[str]:
        """获取可用端口列表 - 委托给Service"""
        return self.serial_service.get_available_ports()
    
    def get_current_port(self) -> Optional[str]:
        """获取当前连接的端口 - 委托给Service"""
        return self.serial_service.get_current_port()
    
    def get_connection_status(self) -> bool:
        """获取连接状态 - 委托给Service"""
        return self.serial_service.is_connected()
    
    def get_port_info(self, port_name: str) -> Optional[Dict[str, str]]:
        """获取端口信息 - 委托给Service"""
        return self.serial_service.get_port_info(port_name)
    
    # ===== 私有方法 - 纯信号转发 =====
    
    def _connect_service_signals(self) -> None:
        """连接Service信号到ViewModel信号 - 纯转发"""
        # 数据接收信号 - 直接转发
        self.serial_service.data_received.connect(self.data_received.emit)
        
        # 连接状态变化信号 - 直接转发
        self.serial_service.connection_status_changed.connect(self.connection_status_changed.emit)
        
        # 端口列表更新信号 - 直接转发
        self.serial_service.port_list_updated.connect(self.port_list_updated.emit)
    
    def _disconnect_service_signals(self) -> None:
        """断开Service信号连接"""
        try:
            self.serial_service.data_received.disconnect(self.data_received.emit)
            self.serial_service.connection_status_changed.disconnect(self.connection_status_changed.emit)
            self.serial_service.port_list_updated.disconnect(self.port_list_updated.emit)
        except TypeError:
            # 如果信号未连接，disconnect会抛出TypeError，忽略即可
            pass
    
    def cleanup(self) -> None:
        """清理视图模型 - 断开信号连接"""
        self._disconnect_service_signals()
        super().cleanup() 