import yaml
import struct
from typing import Any, Dict, List
from dataclasses import make_dataclass, field
from domain import Robot
from abc import ABC, abstractmethod

class BaseFormatter(ABC):
    @abstractmethod
    def format(self, value: Any, params: Dict[str, Any], robot=None) -> str:
        pass

class HexFormatter(BaseFormatter):
    def format(self, value: Any, params: Dict[str, Any], robot=None) -> str:
        width = params.get('width', 2)
        return f"{value:0{width}X}"

class ListFormatter(BaseFormatter):
    def format(self, value: Any, params: Dict[str, Any], robot=None) -> str:
        if not isinstance(value, list):
            raise ValueError("ListFormatter requires a list value")
        
        # 如果需要转换，调用robot的相应方法
        if 'transform' in params and robot:
            transform_method = getattr(robot, params['transform'])
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
    def format(self, value: Any, params: Dict[str, Any], robot=None) -> str:
        format_string = params['format_string']
        packed = struct.pack(format_string, value)
        return ''.join(f"{b:02X}" for b in packed)

class ConfigurableMessageUtils:
    def __init__(self, config_path: str = "config/message_out_config.yaml"):
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        self.robot = Robot()
        self.formatters = self._register_formatters()
        self.MessageOut = self._create_message_out_class()
    
    def _register_formatters(self) -> Dict[str, BaseFormatter]:
        return {
            'hex_formatter': HexFormatter(),
            'list_formatter': ListFormatter(), 
            'struct_formatter': StructFormatter(),
        }
    
    def _create_message_out_class(self):
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
    
    def create_message_out(self, **kwargs):
        """创建MessageOut实例"""
        return self.MessageOut(**kwargs)
    
    def interpret_message(self, message_out) -> str:
        """根据配置动态解释消息"""
        protocol_config = self.config['protocol']
        fields_config = self.config['message_formats']['message_out']['fields']
        
        # 1. 添加协议头
        command = protocol_config['header']
        
        # 2. 按配置顺序处理每个字段
        for field_name, field_config in fields_config.items():
            field_value = getattr(message_out, field_name)
            command += self._format_field(field_value, field_config)
        
        # 3. 计算CRC（如果启用）
        if protocol_config.get('crc_enabled', False):
            crc = self.robot.calculate_crc16(bytes.fromhex(command))
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
        return formatter.format(value, params, self.robot)