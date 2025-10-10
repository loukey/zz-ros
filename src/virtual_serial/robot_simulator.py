"""
虚拟机械臂模拟器 - 模拟机械臂状态和运动
"""
from math import pi
from typing import List


class RobotSimulator:
    """模拟机械臂状态"""
    
    def __init__(self):
        # 初始化状态
        self.is_initialized = False
        self.current_control = 0x06
        self.current_mode = 0x08
        
        # 机械臂状态（使用与RobotUtils相同的参数）
        self.JOINT_OFFSETS = [78623, 369707, 83986, 391414, 508006, 456372]
        self.RADIAN_TO_POS_SCALE_FACTOR = (2**19) / (2 * pi)  # 弧度转位置值的系数
        self.init_radians = [0, -pi/2, 0, pi/2, 0, 0]
        
        # 当前位置（位置编码器值）
        self.current_positions = self._radians_to_positions(self.init_radians)
        
        # 目标位置
        self.target_positions = self.current_positions.copy()
        
        # 其他状态
        self.current_speeds = [0] * 6
        self.current_torques = [0] * 6
        self.joint_status = [0x0000] * 6  # 关节状态字
        self.joint_errors = [0x0000] * 6  # 关节错误码
        self.double_encoder_interpolations = [0] * 6
        self.effector_data = 0.0  # 夹爪数据
        
    def _radians_to_positions(self, radians: List[float]) -> List[int]:
        """弧度转位置编码器值（与RobotUtils.radian2position一致）"""
        positions = []
        for i, rad in enumerate(radians):
            position = int((rad - self.init_radians[i]) * self.RADIAN_TO_POS_SCALE_FACTOR + self.JOINT_OFFSETS[i]) & 0xFFFFFFFF
            # 转换为有符号32位
            if position >= 2**31:
                position -= 2**32
            positions.append(position)
        return positions
    
    def process_command(self, command_hex: str) -> dict:
        """
        处理接收到的命令，返回当前状态
        
        MessageOut格式：
        AA55 + control(1) + mode(1) + joint_angles(24) + speeds(18) + accel(18) + decel(18) + torque(12) + effector_mode(1) + effector_data(4) + CRC(2) + 0D0A
        """
        try:
            # 解析命令
            if not command_hex.startswith("AA55"):
                return self._get_current_state()
            
            # 提取control和mode
            control = int(command_hex[4:6], 16)
            mode = int(command_hex[6:8], 16)
            
            self.current_control = control
            self.current_mode = mode
            
            # 根据control处理不同命令
            if control == 0x00:
                # 夹爪控制
                self._process_gripper_command(command_hex)
            elif control == 0x06:
                # 运动控制
                self._process_motion_command(command_hex)
            elif control == 0x01:
                # 初始化
                self.is_initialized = True
            elif control == 0x02:
                # 停止
                self.target_positions = self.current_positions.copy()
                self.current_speeds = [0] * 6
            
            # 简单模拟：直接到达目标位置
            self.current_positions = self.target_positions.copy()
            
            return self._get_current_state()
            
        except Exception as e:
            print(f"处理命令出错: {e}")
            return self._get_current_state()
    
    def _process_motion_command(self, command_hex: str):
        """处理运动命令，提取目标位置"""
        # joint_angles 位置：AA55(4) + control(2) + mode(2) = 8
        # 长度：6个关节 * 4字节 * 2字符/字节 = 48
        start_idx = 8
        end_idx = start_idx + 48
        
        if len(command_hex) < end_idx:
            return
        
        angles_hex = command_hex[start_idx:end_idx]
        
        # 解析每个关节的位置值
        positions = []
        for i in range(6):
            joint_hex = angles_hex[i*8:(i+1)*8]
            # 大端序，无符号32位
            position = int.from_bytes(bytes.fromhex(joint_hex), byteorder='big', signed=False)
            
            # 转换为有符号
            if position >= 2**31:
                position -= 2**32
            
            positions.append(position)
        
        self.target_positions = positions
        
        # 模拟速度（简单设置为非零值表示运动中）
        self.current_speeds = [1000] * 6
    
    def _process_gripper_command(self, command_hex: str):
        """处理夹爪命令"""
        # 夹爪数据在最后：从后往前数
        # footer(4) + CRC(4) + effector_data(8) + effector_mode(2)
        try:
            effector_data_start = len(command_hex) - 4 - 4 - 8
            effector_data_hex = command_hex[effector_data_start:effector_data_start + 8]
            
            # 解析effector_data（4字节float，大端序）
            effector_bytes = bytes.fromhex(effector_data_hex)
            import struct
            self.effector_data = struct.unpack('>f', effector_bytes)[0]
        except Exception as e:
            print(f"解析夹爪数据出错: {e}")
    
    def _get_current_state(self) -> dict:
        """获取当前状态"""
        return {
            'init_status': 0x01 if self.is_initialized else 0x00,
            'control': self.current_control,
            'mode': self.current_mode,
            'positions': self.current_positions,
            'status': self.joint_status,
            'speeds': self.current_speeds,
            'torques': self.current_torques,
            'double_encoder_interpolations': self.double_encoder_interpolations,
            'errors': self.joint_errors,
            'effector_data': self.effector_data
        }

