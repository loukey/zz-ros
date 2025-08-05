"""
串口应用服务 - Application层
只负责串口连接管理和状态协调，不处理数据收发
数据收发由各个Domain服务直接使用SerialDomainService
"""
from typing import List, Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal
from ..commands import MessageDisplay
from .base_service import BaseService
from domain import SerialDomainService


class SerialApplicationService(BaseService):
    """串口应用服务 - 只负责串口连接管理，不处理数据收发"""

    connection_status_changed = pyqtSignal(bool)
    port_list_updated = pyqtSignal(list)
    
    def __init__(self, message_display: MessageDisplay, serial_domain_service: SerialDomainService):
        """初始化串口服务"""
        super().__init__(message_display)
        
        self.serial_domain_service = serial_domain_service
        
        # 连接Domain层信号到Application层
        self._connect_domain_signals()
    
    def refresh_ports(self) -> List[str]:
        """
        刷新可用端口列表
        
        返回:
            List[str]: 可用端口名称列表
        """
        try:
            ports = self.serial_domain_service.scan_available_ports()
            self.port_list_updated.emit(ports)
            self._display_message(f"发现 {len(ports)} 个可用端口", "系统")
            return ports
        except Exception as e:
            self._display_message(f"刷新端口失败: {str(e)}", "错误")
            return []
    
    def get_port_info(self, port_name: str) -> Optional[Dict[str, str]]:
        """
        获取指定端口信息
        
        参数:
            port_name: 端口名称
            
        返回:
            Optional[Dict[str, str]]: 端口信息字典，失败时返回None
        """
        try:
            return self.serial_domain_service.get_port_info(port_name)
        except Exception as e:
            self._display_message(f"获取端口信息失败: {str(e)}", "错误")
            return None
    
    def get_available_ports(self) -> List[str]:
        """
        获取当前可用端口列表（不刷新）
        
        返回:
            List[str]: 可用端口名称列表
        """
        try:
            return self.serial_domain_service.scan_available_ports()
        except Exception as e:
            self._display_message(f"获取端口列表失败: {str(e)}", "错误")
            return []
    
    def get_current_port(self) -> Optional[str]:
        """
        获取当前连接的端口名称
        
        返回:
            Optional[str]: 端口名称，未连接时返回None
        """
        return self.serial_domain_service.get_current_port()
    
    def connect_serial(self, port: str, config: Dict[str, Any]) -> bool:
        """
        连接串口
        
        参数:
            port: 串口名称
            config: 串口配置参数
            
        返回:
            bool: 连接是否成功
        """
        self._display_message(f"连接串口: {port}, 配置: {config}", "系统")
        try:
            success = self.serial_domain_service.connect_port(port, config)
            if success:
                self._display_message(f"成功连接到端口 {port}", "系统")
            return success
        except Exception as e:
            self._display_message(f"连接失败: {str(e)}", "错误")
            return False
    
    def disconnect_serial(self) -> bool:
        """
        断开串口连接
        
        返回:
            bool: 断开是否成功
        """
        try:
            success = self.serial_domain_service.disconnect_port()
            if success:
                self._display_message("已断开串口连接", "系统")
            return success
        except Exception as e:
            self._display_message(f"断开连接失败: {str(e)}", "错误")
            return False
    
    def is_connected(self) -> bool:
        """
        检查串口是否已连接
        
        返回:
            bool: 是否已连接
        """
        return self.serial_domain_service.is_connected()
    
    def _connect_domain_signals(self) -> None:
        """连接Domain层信号到Application层"""
        self.serial_domain_service.connection_status_changed.connect(self._on_domain_connection_status_changed)
    
    def _disconnect_domain_signals(self) -> None:
        """断开Domain层信号连接"""
        try:
            self.serial_domain_service.connection_status_changed.disconnect(self._on_domain_connection_status_changed)
        except TypeError:
            # 如果信号未连接，disconnect会抛出TypeError，忽略即可
            pass
    
    def _on_domain_connection_status_changed(self, connected: bool) -> None:
        """处理Domain层的连接状态变化，转发到Application层信号"""
        self.connection_status_changed.emit(connected)
    
    def cleanup(self) -> None:
        """清理资源"""
        try:
            self._disconnect_domain_signals()
            self.serial_domain_service.cleanup()
        except Exception:
            pass  # 忽略清理时的异常 
