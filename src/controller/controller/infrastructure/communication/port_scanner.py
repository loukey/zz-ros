"""
串口扫描器 - Infrastructure层
负责扫描可用的串口端口
"""
import serial.tools.list_ports
import os
import glob
from typing import List, Dict, Optional


class PortScanner:
    """串口端口扫描器"""
    
    @staticmethod
    def scan_ports() -> List[str]:
        """
        扫描可用的串口端口（包含真实串口和虚拟串口）
        
        返回:
            List[str]: 可用端口名称列表
        """
        all_ports = []
        
        try:
            # 1. 扫描真实硬件串口
            ports = serial.tools.list_ports.comports()
            all_ports.extend([port.device for port in ports])
            
            # 2. 扫描虚拟串口（伪终端）
            virtual_port_patterns = [
                '/dev/pts/*',      # 伪终端
                '/tmp/tty*',       # 临时虚拟串口（socat创建的）
                '/dev/ttyV*',      # 虚拟串口设备
            ]
            
            for pattern in virtual_port_patterns:
                for device in glob.glob(pattern):
                    # 检查是否是字符设备且可访问
                    if os.path.exists(device) and PortScanner._is_accessible(device):
                        if device not in all_ports:
                            all_ports.append(device)
            
            # 排序：真实串口在前，虚拟串口在后
            all_ports.sort(key=lambda x: (not x.startswith('/dev/ttyUSB'), 
                                         not x.startswith('/dev/ttyACM'), 
                                         x))
            
            return all_ports
            
        except Exception:
            return all_ports
    
    @staticmethod
    def _is_accessible(device: str) -> bool:
        """
        检查设备是否可访问
        
        参数:
            device: 设备路径
            
        返回:
            bool: 是否可访问
        """
        try:
            # 尝试以非阻塞方式打开设备
            import serial
            ser = serial.Serial()
            ser.port = device
            ser.timeout = 0
            ser.open()
            ser.close()
            return True
        except:
            # 如果无法打开，检查是否至少有读权限
            return os.access(device, os.R_OK)
    
    @staticmethod
    def get_port_info(port_name: str) -> Optional[Dict[str, str]]:
        """
        获取指定端口的详细信息（支持真实串口和虚拟串口）
        
        参数:
            port_name: 端口名称
            
        返回:
            Dict[str, str]: 端口信息字典，包含description和hwid
        """
        try:
            # 1. 先尝试从真实硬件串口获取信息
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if port.device == port_name:
                    return {
                        'device': port.device,
                        'description': port.description or '',
                        'hwid': port.hwid or ''
                    }
            
            # 2. 如果是虚拟串口，返回自定义信息
            if os.path.exists(port_name):
                description = "虚拟串口"
                
                # 根据路径判断类型
                if port_name.startswith('/tmp/'):
                    description = "临时虚拟串口 (socat)"
                elif port_name.startswith('/dev/pts/'):
                    description = "伪终端 (pty)"
                elif port_name.startswith('/dev/ttyV'):
                    description = "虚拟串口设备"
                
                return {
                    'device': port_name,
                    'description': description,
                    'hwid': 'VIRTUAL'
                }
            
            return None
        except Exception:
            return None
    
    @staticmethod
    def get_all_port_info() -> List[Dict[str, str]]:
        """
        获取所有可用端口的详细信息（包含真实串口和虚拟串口）
        
        返回:
            List[Dict[str, str]]: 所有端口信息列表
        """
        try:
            all_port_info = []
            
            # 获取所有端口
            all_ports = PortScanner.scan_ports()
            
            # 为每个端口获取信息
            for port_name in all_ports:
                port_info = PortScanner.get_port_info(port_name)
                if port_info:
                    all_port_info.append(port_info)
            
            return all_port_info
        except Exception:
            return [] 