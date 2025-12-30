from controller.domain import MotionRunner, RobotUtils
from ..commands import MessageDisplay


class MotionListener:
    """运动消息监听器。
    
    负责监听运动执行器发送的消息，解码位置数据并显示。
    
    Attributes:
        motion_runner (MotionRunner): 运动执行器。
        message_display (MessageDisplay): 消息显示服务。
        robot_utils (RobotUtils): 机器人工具类，用于坐标转换。
    """

    def __init__(self, motion_runner: MotionRunner, message_display: MessageDisplay):
        """初始化运动监听器。
        
        Args:
            motion_runner (MotionRunner): 运动执行器。
            message_display (MessageDisplay): 消息显示服务。
        """
        self.motion_runner = motion_runner
        self.message_display = message_display
        self.robot_utils = RobotUtils()
        self._connect_signals()

    def _connect_signals(self):
        """连接信号。"""
        self.motion_runner.motion_msg_signal.connect(self.handle_motion_msg)

    def handle_motion_msg(self, message, message_type):
        """处理运动消息，手动解析位置数据。
        
        消息格式：AA55 + control(1) + mode(1) + joint_angles(24) + ... + CRC(2) + 0D0A
        编码过程：弧度 -> radian2position -> 位置值(32位) -> 大端序16进制
        
        Args:
            message (str): 原始16进制消息字符串。
            message_type (str): 消息类型。
        """
        try:
            # 手动解析位置数据
            joint_angles = self._extract_joint_angles(message)
            
            if joint_angles:
                # 格式化显示弧度值
                angles_str = ", ".join([f"{angle:.4f}" for angle in joint_angles])
                display_text = f"[{message_type}] 位置(弧度): [{angles_str}]"
                self.message_display.display_message(display_text, message_type)
                
                # 同时显示原始16进制数据
                raw_text = f"[{message_type}] 原始数据: {message}"
                self.message_display.display_message(raw_text, message_type)
            else:
                # 无法解析，只显示原始消息
                self.message_display.display_message(str(message), message_type)
                
        except Exception as e:
            # 解析失败，显示原始消息
            self.message_display.display_message(str(message), message_type)
    
    def _extract_joint_angles(self, message: str) -> list:
        """从16进制消息中提取关节角度（反向解码）。
        
        编码流程：
        1. 弧度值 -> radian2position() 转换为位置值（无符号32位整数）
        2. 位置值 -> ListFormatter 按大端序编码为16进制（4字节，8个字符）
        
        解码流程（反向）：
        1. 16进制字符串 -> 解析为位置值（无符号32位整数）
        2. 位置值列表 -> position2radian() 转换为弧度
        
        Args:
            message (str): 16进制消息字符串。
            
        Returns:
            list: 关节角度列表（弧度），解析失败返回空列表。
        """
        try:
            # 检查消息是否以AA55开头
            if not message.startswith("AA55"):
                return []
            
            # joint_angles 位置：AA55(4) + control(2) + mode(2) = 8个字符后开始
            # 长度：6个关节 * 4字节 * 2字符/字节 = 48个字符
            start_idx = 8
            end_idx = start_idx + 48
            
            if len(message) < end_idx:
                return []
            
            # 提取joint_angles的16进制数据
            angles_hex = message[start_idx:end_idx]
            
            # 第一步：解析每个关节的位置值（有符号32位整数）
            positions = []
            for i in range(6):
                # 提取单个关节的数据（4字节=8个16进制字符）
                joint_hex = angles_hex[i*8:(i+1)*8]
                
                # 先转换为无符号整数
                value = int(joint_hex, 16)
                
                # ⭐ 处理有符号数的补码表示（与ListParser保持一致）
                # 4字节有符号整数范围：-2147483648 到 2147483647
                if value >= 0x80000000:  # 如果最高位为1，表示负数
                    value -= 0x100000000  # 转换为负数（2^32）
                
                positions.append(value)
            
            # 第二步：使用 position2radian 转换为弧度
            # 这里完全按照 RobotUtils 的逻辑反向计算
            joint_angles = self.robot_utils.position2radian(positions)
            
            return joint_angles
            
        except Exception:
            return []

