"""
机械臂状态快照 - Domain层值对象
不可变的状态数据载体
"""
from dataclasses import dataclass
from typing import Tuple, Any, Optional
import time
from ..utils import RobotUtils


@dataclass(frozen=True)
class RobotStateSnapshot:
    """
    机械臂状态快照 - 不可变数据载体
    
    特点：
    - frozen=True: 不可变，线程安全
    - 包含完整的机械臂状态
    - 可以保存历史快照序列
    """
    # 基本信息
    init_status: int              # 初始化状态
    control: int                  # 当前命令 0x06/0x07...
    mode: int                     # 运行模式 0x08/0x0A...
    
    # 关节数据（6个关节）
    joint_positions: Tuple[int, ...]      # 编码器位置（原始值）
    joint_angles: Tuple[float, ...]       # 关节角度（弧度）
    joint_speeds: Tuple[int, ...]         # 关节速度
    joint_torques: Tuple[int, ...]        # 关节力矩
    joint_status: Tuple[int, ...]         # 关节状态码
    
    # 其他数据
    double_encoder_interpolations: Tuple[int, ...]  # 双编码器插值
    errors: Tuple[int, ...]               # 错误码
    effector_data: Any                    # 夹爪数据
    
    # 时间戳
    timestamp: float                      # 接收时间
    
    @classmethod
    def from_decoded_message(cls, decoded_msg):
        """
        从解码消息创建快照
        
        Args:
            decoded_msg: 解码后的消息对象
            
        Returns:
            RobotStateSnapshot: 状态快照
            
        注意：
            decoded_msg.positions 已经通过 position2radian 转换为弧度值
            所以不需要再次转换
        """        
        robot_utils = RobotUtils()
        
        # decoded_msg.positions 已经是弧度值（由MessageDecoder的transform完成）
        # 所以直接使用，不需要再次转换
        angles = decoded_msg.positions  # 已经是弧度值
        
        # 反向计算原始编码器位置值（用于保存完整状态）
        positions = robot_utils.radian2position(angles)
        print(positions)
        return cls(
            init_status=decoded_msg.init_status,
            control=decoded_msg.control,
            mode=decoded_msg.mode,
            joint_positions=tuple(positions),  # 反向计算的编码器值
            joint_angles=tuple(angles),        # 直接使用解码后的弧度值
            joint_speeds=tuple(decoded_msg.speeds),
            joint_torques=tuple(decoded_msg.torques),
            joint_status=tuple(decoded_msg.status),
            double_encoder_interpolations=tuple(decoded_msg.double_encoder_interpolations),
            errors=tuple(decoded_msg.errors),
            effector_data=decoded_msg.effector_data,
            timestamp=time.time()
        )

