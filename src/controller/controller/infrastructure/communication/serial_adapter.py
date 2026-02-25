"""
串口适配器 - Infrastructure层
负责管理串口连接和断开
"""
from typing import Optional, Dict, Any
from .native_serial_port import NativeSerialPort


class SerialAdapter:
    """串口连接适配器。

    Attributes:
        serial_port (Optional[NativeSerialPort]): 原生串口实例。
        current_config (Optional[Dict[str, Any]]): 当前配置字典。
    """

    def __init__(self):
        """初始化串口适配器。"""
        self.serial_port: Optional[NativeSerialPort] = None
        self.current_config: Optional[Dict[str, Any]] = None

    def connect(self, port: str, config: Dict[str, Any]) -> bool:
        """连接串口。

        Args:
            port (str): 串口名称。
            config (Dict[str, Any]): 串口配置参数。

        Returns:
            bool: 连接是否成功。
        """
        try:
            # 如果已经连接，先断开
            if self.is_connected():
                self.disconnect()

            # 创建串口连接
            self.serial_port = NativeSerialPort()
            if not self.serial_port.open(port, config):
                self.serial_port = None
                self.current_config = None
                return False

            # 保存配置
            self.current_config = config.copy()
            self.current_config['port'] = port

            return True

        except Exception:
            self.serial_port = None
            self.current_config = None
            return False

    def disconnect(self) -> bool:
        """断开串口连接。

        Returns:
            bool: 断开是否成功。
        """
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            return True
        except Exception:
            return False
        finally:
            self.serial_port = None
            self.current_config = None

    def is_connected(self) -> bool:
        """检查串口是否已连接。

        Returns:
            bool: 是否已连接。
        """
        return (self.serial_port is not None and
                self.serial_port.is_open)

    def get_serial_port(self) -> Optional[NativeSerialPort]:
        """获取串口对象。

        Returns:
            Optional[NativeSerialPort]: 串口对象，未连接时返回None。
        """
        if self.is_connected():
            return self.serial_port
        return None

    def get_current_config(self) -> Optional[Dict[str, Any]]:
        """获取当前连接配置。

        Returns:
            Optional[Dict[str, Any]]: 当前配置，未连接时返回 None。
        """
        return self.current_config.copy() if self.current_config else None

    def get_port_name(self) -> Optional[str]:
        """获取当前连接的端口名称。

        Returns:
            Optional[str]: 端口名称，未连接时返回 None。
        """
        if self.current_config:
            return self.current_config.get('port')
        return None
