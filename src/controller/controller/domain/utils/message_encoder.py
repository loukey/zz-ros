import yaml
import struct
import os
from typing import Any, Dict, List
from dataclasses import make_dataclass, field
from .robot_utils import RobotUtils

# 尝试导入 ROS2 包查找工具
try:
    from ament_index_python.packages import get_package_share_directory
    _HAS_AMENT_INDEX = True
except ImportError:
    _HAS_AMENT_INDEX = False
from abc import ABC, abstractmethod

class BaseFormatter(ABC):
    @abstractmethod
    def format(self, value: Any, params: Dict[str, Any], robot_utils=None) -> str:
        pass

class HexFormatter(BaseFormatter):
    def format(self, value: Any, params: Dict[str, Any], robot_utils=None) -> str:
        width = params.get('width', 2)
        return f"{value:0{width}X}"

class ListFormatter(BaseFormatter):
    def format(self, value: Any, params: Dict[str, Any], robot_utils=None) -> str:
        if not isinstance(value, list):
            raise ValueError("ListFormatter requires a list value")
        
        if 'transform' in params and robot_utils:
            transform_method = getattr(robot_utils, params['transform'])
            transformed_values = transform_method(value)
        else:
            transformed_values = value
        
        result = ""
        item_bytes = params['item_bytes']
        
        for val in transformed_values:
            if item_bytes == 1:
                result += f"{val & 0xFF:02X}"
            elif item_bytes == 2:
                result += f"{(val >> 8) & 0xFF:02X}{val & 0xFF:02X}"
            elif item_bytes == 3:
                result += f"{(val >> 16) & 0xFF:02X}{(val >> 8) & 0xFF:02X}{val & 0xFF:02X}"
            elif item_bytes == 4:
                result += f"{(val >> 24) & 0xFF:02X}{(val >> 16) & 0xFF:02X}{(val >> 8) & 0xFF:02X}{val & 0xFF:02X}"
            else:
                raise ValueError(f"Unsupported item_bytes: {item_bytes}")
        
        return result

class StructFormatter(BaseFormatter):
    def format(self, value: Any, params: Dict[str, Any], robot_utils=None) -> str:
        format_string = params['format_string']
        packed = struct.pack(format_string, value)
        return ''.join(f"{b:02X}" for b in packed)

class MessageEncoder:
    def __init__(self, config_path = "controller/config/message_encoder_config.yaml"):
        # 使用 ROS2 包资源管理方式查找配置文件
        if _HAS_AMENT_INDEX:
            try:
                # ROS2 方式：从 share 目录加载
                package_share_dir = get_package_share_directory('controller')
                config_path = os.path.join(package_share_dir, 'config', 'message_encoder_config.yaml')
            except Exception:
                # 如果包未安装，使用相对路径（开发模式）
                pass
        
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        self.robot_utils = RobotUtils()
        self.formatters = self._register_formatters()
        self.Message = self._create_message_class()
    
    def _register_formatters(self) -> Dict[str, BaseFormatter]:
        return {
            'hex_formatter': HexFormatter(),
            'list_formatter': ListFormatter(), 
            'struct_formatter': StructFormatter(),
        }
    
    def _create_message_class(self):
        """动态创建MessageOut类"""
        fields_config = self.config['message_formats']['message_out']['fields']
        fields = []
        
        for field_name, field_config in fields_config.items():
            field_type = self._parse_type(field_config['type'])
            default_value = field_config['default']
            
            if isinstance(default_value, list):
                fields.append((
                    field_name, 
                    field_type, 
                    field(default_factory=lambda val=default_value: val.copy())
                ))
            else:
                fields.append((field_name, field_type, default_value))
        
        return make_dataclass('MessageOut', fields)
    
    def _parse_type(self, type_str: str):
        if type_str == "int":
            return int
        elif type_str == "float":
            return float
        elif type_str == "List[float]":
            return List[float]
        else:
            raise ValueError(f"Unsupported type: {type_str}")
    
    def create_message(self, **kwargs):
        """创建MessageOut实例"""
        return self.Message(**kwargs)
    
    def interpret_message(self, message) -> str:
        """根据配置动态解释消息"""
        protocol_config = self.config['protocol']
        fields_config = self.config['message_formats']['message_out']['fields']
        
        # 1. 添加协议头
        command = protocol_config['header']
        
        # 2. 按配置顺序处理每个字段
        for field_name, field_config in fields_config.items():
            field_value = getattr(message, field_name)
            command += self._format_field(field_value, field_config)
        
        # 3. 计算CRC（如果启用）
        if protocol_config.get('crc_enabled', False):
            crc = self.robot_utils.calculate_crc16(bytes.fromhex(command))
            command += f"{(crc >> 8) & 0xFF:02X}{crc & 0xFF:02X}"
        
        # 4. 添加协议尾
        command += protocol_config['footer']
        
        return command
    
    def _format_field(self, value: Any, config: Dict[str, Any]) -> str:
        """通用字段格式化方法"""
        formatter_name = config['formatter']
        formatter_config = self.config['formatters'][formatter_name] 
        handler_name = formatter_config['handler']
        params = formatter_config.get('params', {})
        
        formatter = self.formatters[handler_name]
        return formatter.format(value, params, self.robot_utils)
