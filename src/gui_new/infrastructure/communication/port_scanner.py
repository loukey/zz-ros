"""
串口扫描器 - Infrastructure层
负责扫描可用的串口端口
"""
import serial.tools.list_ports
from typing import List, Dict, Optional


class PortScanner:
    """串口端口扫描器"""
    
    @staticmethod
    def scan_ports() -> List[str]:
        """
        扫描可用的串口端口
        
        返回:
            List[str]: 可用端口名称列表
        """
        try:
            ports = serial.tools.list_ports.comports()
            return [port.device for port in ports]
        except Exception:
            return []
    
    @staticmethod
    def get_port_info(port_name: str) -> Optional[Dict[str, str]]:
        """
        获取指定端口的详细信息
        
        参数:
            port_name: 端口名称
            
        返回:
            Dict[str, str]: 端口信息字典，包含description和hwid
        """
        try:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if port.device == port_name:
                    return {
                        'device': port.device,
                        'description': port.description or '',
                        'hwid': port.hwid or ''
                    }
            return None
        except Exception:
            return None
    
    @staticmethod
    def get_all_port_info() -> List[Dict[str, str]]:
        """
        获取所有可用端口的详细信息
        
        返回:
            List[Dict[str, str]]: 所有端口信息列表
        """
        try:
            ports = serial.tools.list_ports.comports()
            return [
                {
                    'device': port.device,
                    'description': port.description or '',
                    'hwid': port.hwid or ''
                }
                for port in ports
            ]
        except Exception:
            return [] 