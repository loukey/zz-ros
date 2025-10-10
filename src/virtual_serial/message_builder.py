"""
虚拟串口消息构建器 - 生成MessageIn格式的消息
"""
import struct
from typing import List


class MessageBuilder:
    """根据MessageIn格式构建响应消息"""
    
    def __init__(self):
        self.HEADER = "AA55"
        self.FOOTER = "0D0A"
    
    def build_message_in(
        self,
        init_status: int = 0x01,
        control: int = 0x06,
        mode: int = 0x08,
        positions: List[int] = None,
        status: List[int] = None,
        speeds: List[int] = None,
        torques: List[int] = None,
        double_encoder_interpolations: List[int] = None,
        errors: List[int] = None,
        effector_data: float = 0.0
    ) -> str:
        """
        构建MessageIn格式的响应消息
        
        MessageIn格式：
        - header: 2字节 (AA55)
        - init_status: 1字节
        - control: 1字节
        - mode: 1字节
        - positions: 24字节 (6个关节，每个4字节，有符号大端序)
        - status: 12字节 (6个状态字，每个2字节)
        - speeds: 24字节 (6个速度，每个4字节)
        - torques: 12字节 (6个力矩，每个2字节)
        - double_encoder_interpolations: 24字节 (6个插值，每个4字节)
        - errors: 12字节 (6个错误码，每个2字节)
        - effector_data: 4字节 (夹爪数据，格式：整数部分2字节+小数部分2字节)
        - crc: 2字节
        - footer: 2字节 (0D0A)
        """
        # 默认值
        if positions is None:
            positions = [0] * 6
        if status is None:
            status = [0x0000] * 6
        if speeds is None:
            speeds = [0] * 6
        if torques is None:
            torques = [0] * 6
        if double_encoder_interpolations is None:
            double_encoder_interpolations = [0] * 6
        if errors is None:
            errors = [0x0000] * 6
        
        # 开始构建消息（不包含header）
        message = ""
        
        # 1. init_status (1字节)
        message += f"{init_status:02X}"
        
        # 2. control (1字节)
        message += f"{control:02X}"
        
        # 3. mode (1字节)
        message += f"{mode:02X}"
        
        # 4. positions (24字节，6个关节，每个4字节，有符号大端序)
        for pos in positions:
            # 转换为有符号32位整数，大端序
            message += self._int32_to_hex(pos, signed=True)
        
        # 5. status (12字节，6个状态字，每个2字节)
        for st in status:
            message += f"{st:04X}"
        
        # 6. speeds (24字节，6个速度，每个4字节)
        for speed in speeds:
            message += self._int32_to_hex(speed, signed=True)
        
        # 7. torques (12字节，6个力矩，每个2字节)
        for torque in torques:
            message += self._int16_to_hex(torque, signed=True)
        
        # 8. double_encoder_interpolations (24字节)
        for interp in double_encoder_interpolations:
            message += self._int32_to_hex(interp, signed=True)
        
        # 9. errors (12字节，6个错误码，每个2字节)
        for err in errors:
            message += f"{err:04X}"
        
        # 10. effector_data (4字节：整数部分2字节 + 小数部分2字节)
        message += self._float_to_effector_hex(effector_data)
        
        # 11. 计算CRC (对整个消息包含header，不包含crc和footer)
        crc_data = self.HEADER + message
        crc = self._calculate_crc16(bytes.fromhex(crc_data))
        message += f"{crc:04X}"
        
        # 12. 组合完整消息
        full_message = self.HEADER + message + self.FOOTER
        
        return full_message
    
    def _int32_to_hex(self, value: int, signed: bool = True) -> str:
        """32位整数转16进制字符串（大端序）"""
        if signed:
            # 有符号32位
            if value < 0:
                value = (1 << 32) + value
        else:
            value = value & 0xFFFFFFFF
        
        # 大端序：高字节在前
        return f"{(value >> 24) & 0xFF:02X}{(value >> 16) & 0xFF:02X}{(value >> 8) & 0xFF:02X}{value & 0xFF:02X}"
    
    def _int16_to_hex(self, value: int, signed: bool = True) -> str:
        """16位整数转16进制字符串（大端序）"""
        if signed:
            # 有符号16位
            if value < 0:
                value = (1 << 16) + value
        else:
            value = value & 0xFFFF
        
        # 大端序：高字节在前
        return f"{(value >> 8) & 0xFF:02X}{value & 0xFF:02X}"
    
    def _float_to_effector_hex(self, value: float) -> str:
        """浮点数转夹爪数据格式（整数部分2字节 + 小数部分2字节）"""
        # 分离整数和小数部分
        int_part = int(value)
        dec_part = int((value - int_part) * 10)  # 取一位小数
        
        # 大端序
        int_hex = f"{(int_part >> 8) & 0xFF:02X}{int_part & 0xFF:02X}"
        dec_hex = f"{(dec_part >> 8) & 0xFF:02X}{dec_part & 0xFF:02X}"
        
        return int_hex + dec_hex
    
    @staticmethod
    def _calculate_crc16(data: bytes) -> int:
        """
        计算CRC16校验码（CRC16-CCITT，与RobotUtils完全一致）
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= (int(byte) << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc

