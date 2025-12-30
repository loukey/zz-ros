import yaml
import struct
import os
from typing import Any, Dict, List, Union
from dataclasses import make_dataclass, field
from .robot_utils import RobotUtils

# 尝试导入 ROS2 包查找工具
try:
    from ament_index_python.packages import get_package_share_directory
    _HAS_AMENT_INDEX = True
except ImportError:
    _HAS_AMENT_INDEX = False
from abc import ABC, abstractmethod


class BaseParser(ABC):
    @abstractmethod
    def parse(self, hex_data: str, params: Dict[str, Any], robot_utils=None) -> Any:
        pass


class HexParser(BaseParser):
    def parse(self, hex_data: str, params: Dict[str, Any], robot_utils=None) -> str:
        return hex_data


class IntParser(BaseParser):
    def parse(self, hex_data: str, params: Dict[str, Any], robot_utils=None) -> int:
        bytes_count = params.get('bytes', 1)
        signed = params.get('signed', False)
        byte_order = params.get('byte_order', 'big')
        
        # 将十六进制字符串转换为整数
        value = int(hex_data, 16)
        
        # 处理有符号数的补码表示
        if signed:
            max_value = 1 << (bytes_count * 8)
            if value >= (max_value >> 1):
                value -= max_value
                
        return value


class ListParser(BaseParser):
    def parse(self, hex_data: str, params: Dict[str, Any], robot_utils=None) -> List[Union[int, str]]:
        item_bytes = params.get('item_bytes', 4)
        item_count = params.get('item_count', 6)
        signed = params.get('signed', True)
        format_type = params.get('format', 'int')
        transform = params.get('transform', None)
        
        result = []
        hex_per_item = item_bytes * 2  # 每个字节对应2个十六进制字符
        
        for i in range(item_count):
            start = i * hex_per_item
            end = start + hex_per_item
            item_hex = hex_data[start:end]
            
            if format_type == 'hex':
                result.append(item_hex)
            else:  # format_type == 'int'
                value = int(item_hex, 16)
                
                # 处理有符号数
                if signed:
                    max_value = 1 << (item_bytes * 8)
                    if value >= (max_value >> 1):
                        value -= max_value
                        
                result.append(value)
        
        # 如果需要转换，调用robot_utils的相应方法
        if transform and robot_utils:
            transform_method = getattr(robot_utils, transform)
            result = transform_method(result)
            
        return result


class EffectorParser(BaseParser):
    def parse(self, hex_data: str, params: Dict[str, Any], robot_utils=None) -> float:
        # 夹爪数据是4字节，前2字节是整数部分，后2字节是小数部分
        integer_part = int(hex_data[:4], 16)
        decimal_part = int(hex_data[4:8], 16)
        return float(f"{integer_part}.{decimal_part}")


class MessageDecoder:
    """消息解码器。
    
    基于 YAML 配置文件动态解析十六进制消息。
    
    Attributes:
        config (dict): 加载的配置字典。
        robot_utils (RobotUtils): 工具类实例。
        parsers (Dict[str, BaseParser]): 已注册的解析器字典。
        MessageIn (type): 动态创建的消息数据类。
    """
    
    def __init__(self, config_path = "controller/config/message_decoder_config.yaml"):
        """初始化消息解码器。
        
        Args:
            config_path (str, optional): 配置文件路径. Defaults to "controller/config/message_decoder_config.yaml".
        """
        # 使用 ROS2 包资源管理方式查找配置文件
        if _HAS_AMENT_INDEX:
            try:
                # ROS2 方式：从 share 目录加载
                package_share_dir = get_package_share_directory('controller')
                config_path = os.path.join(package_share_dir, 'config', 'message_decoder_config.yaml')
            except Exception:
                # 如果包未安装，使用相对路径（开发模式）
                pass
        
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
            
        self.robot_utils = RobotUtils()
        self.parsers = self._register_parsers()
        self.MessageIn = self._create_message_class()
    
    def _register_parsers(self) -> Dict[str, BaseParser]:
        """注册可用的解析器。"""
        return {
            'hex_parser': HexParser(),
            'int_parser': IntParser(),
            'list_parser': ListParser(),
            'effector_parser': EffectorParser(),
        }
    
    def _create_message_class(self):
        """动态创建 MessageIn 数据类。"""
        fields_config = self.config['message_formats']['message_in']['fields']
        fields = []
        
        for field_name, field_config in fields_config.items():
            field_type = self._parse_type(field_config['type'])
            
            # 为列表类型提供默认值
            if field_config['type'].startswith('List'):
                fields.append((
                    field_name,
                    field_type,
                    field(default_factory=list)
                ))
            else:
                # 根据类型提供默认值
                if field_type == int:
                    default_value = 0
                elif field_type == float:
                    default_value = 0.0
                else:  # str
                    default_value = ""
                fields.append((field_name, field_type, default_value))
                
        return make_dataclass('MessageIn', fields)
    
    def _parse_type(self, type_str: str):
        """解析类型字符串为 Python 类型。"""
        if type_str == "int":
            return int
        elif type_str == "float":
            return float
        elif type_str == "str":
            return str
        elif type_str == "List[int]":
            return List[int]
        elif type_str == "List[str]":
            return List[str]
        elif type_str == "List[float]":
            return List[float]
        else:
            raise ValueError(f"Unsupported type: {type_str}")
    
    def decode_message(self, hex_message: str) -> Any:
        """解码十六进制消息。
        
        Args:
            hex_message (str): 原始十六进制字符串。
            
        Returns:
            Any: 解码后的消息对象 (MessageIn)。
            
        Raises:
            ValueError: 如果消息格式错误或校验失败。
        """
        # 移除空格和换行符
        hex_message = hex_message.replace(' ', '').replace('\n', '').replace('\r', '')
        
        # 验证协议头和尾
        protocol_config = self.config['protocol']
        header = protocol_config['header']
        footer = protocol_config['footer']
        
        if not hex_message.startswith(header):
            raise ValueError(f"Invalid header. Expected {header}, got {hex_message[:4]}")
            
        if not hex_message.endswith(footer):
            raise ValueError(f"Invalid footer. Expected {footer}, got {hex_message[-4:]}")
        
        
        # 验证CRC（如果启用）
        if protocol_config.get('crc_enabled', False):
            crc_bytes = protocol_config.get('crc_bytes', 2) * 2  # 转换为十六进制字符数
            message_without_crc = hex_message[:-len(footer)-crc_bytes]
            received_crc = hex_message[-len(footer)-crc_bytes:-len(footer)]
            
            calculated_crc = self.robot_utils.calculate_crc16(message_without_crc)
            calculated_crc_hex = f"{calculated_crc:04X}"
            
            if calculated_crc_hex != received_crc:
                raise ValueError(f"CRC校验失败: 接收={received_crc}, 计算={calculated_crc_hex}")
        
        # 解析各个字段
        fields_config = self.config['message_formats']['message_in']['fields']
        parsed_data = {}
        current_pos = 0
        
        for field_name, field_config in fields_config.items():
            field_bytes = field_config['bytes']
            hex_chars = field_bytes * 2  # 每个字节对应2个十六进制字符
            
            # 特殊处理CRC字段
            if field_name == 'crc':
                if protocol_config.get('crc_enabled', False):
                    field_hex = hex_message[current_pos:current_pos + hex_chars]
                else:
                    field_hex = ""
            else:
                field_hex = hex_message[current_pos:current_pos + hex_chars]
            
            # 解析字段
            parsed_value = self._parse_field(field_hex, field_config)
            parsed_data[field_name] = parsed_value
            
            current_pos += hex_chars
        
        return self.MessageIn(**parsed_data)
    
    def _parse_field(self, hex_data: str, config: Dict[str, Any]) -> Any:
        """通用字段解析方法。"""
        parser_name = config['parser']
        parser_config = self.config['parsers'][parser_name]
        handler_name = parser_config['handler']
        params = parser_config.get('params', {})
        
        parser = self.parsers[handler_name]
        return parser.parse(hex_data, params, self.robot_utils)
    
    def decode_message_detailed(self, hex_message: str) -> Dict[str, Any]:
        """解码消息并返回详细信息（包括原始十六进制值）。
        
        Args:
            hex_message (str): 原始十六进制字符串。
            
        Returns:
            Dict[str, Any]: 包含详细信息的字典。
        """
        message_in = self.decode_message(hex_message)
        
        # 创建详细信息字典
        detailed_info = {}
        fields_config = self.config['message_formats']['message_in']['fields']
        
        for field_name, _ in fields_config.items():
            value = getattr(message_in, field_name)
            detailed_info[field_name] = {
                'value': value,
                'description': fields_config[field_name].get('description', field_name)
            }
        
        return detailed_info
