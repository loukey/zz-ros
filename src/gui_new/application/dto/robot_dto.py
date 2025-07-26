"""
机器人相关的数据传输对象
"""
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime


@dataclass
class RobotStateDTO:
    """机器人状态DTO"""
    joint_angles: List[float]  # 关节角度（弧度）
    position: Optional[List[float]] = None  # 位置 [x, y, z]
    orientation: Optional[List[float]] = None  # 姿态 [x, y, z, w] (四元数)
    velocities: Optional[List[float]] = None  # 关节速度
    torques: Optional[List[float]] = None  # 关节力矩
    is_moving: bool = False
    is_connected: bool = False
    last_updated: Optional[datetime] = None


@dataclass
class RobotConfigDTO:
    """机器人配置DTO"""
    model_name: str
    serial_number: str
    max_velocity: List[float]
    max_acceleration: List[float]
    max_jerk: List[float]
    joint_limits_min: List[float]
    joint_limits_max: List[float]


@dataclass
class JointAnglesDTO:
    """关节角度DTO"""
    angles: List[float]  # 6个关节角度（弧度）
    
    def to_degrees(self) -> List[float]:
        """转换为角度"""
        import numpy as np
        return [np.degrees(angle) for angle in self.angles]


@dataclass
class PoseDTO:
    """位姿DTO""" 
    position: List[float]  # [x, y, z]
    orientation: List[float]  # [x, y, z, w] (四元数) 