"""
轨迹相关值对象
"""
from dataclasses import dataclass
from typing import List, Tuple
from enum import Enum
import numpy as np
from .pose import JointAngles


class CurveType(Enum):
    """轨迹曲线类型"""
    S_CURVE = "S型"
    TRAPEZOIDAL = "梯形"
    LINEAR = "直线"


@dataclass(frozen=True)
class VelocityProfile:
    """速度配置文件"""
    max_velocity: tuple[float, ...]      # 最大速度 (6个关节)
    max_acceleration: tuple[float, ...]  # 最大加速度 (6个关节)  
    max_jerk: tuple[float, ...]         # 最大加加速度 (6个关节)
    
    def __post_init__(self):
        if len(self.max_velocity) != 6:
            raise ValueError("必须提供6个关节的最大速度")
        if len(self.max_acceleration) != 6:
            raise ValueError("必须提供6个关节的最大加速度")
        if len(self.max_jerk) != 6:
            raise ValueError("必须提供6个关节的最大加加速度")
    
    @classmethod
    def default(cls) -> 'VelocityProfile':
        """默认速度配置"""
        from math import pi
        return cls(
            max_velocity=tuple([pi/4] * 6),
            max_acceleration=tuple([pi/8] * 6),
            max_jerk=tuple([pi/16] * 6)
        )


@dataclass(frozen=True)
class TrajectoryPoint:
    """轨迹点"""
    time: float                  # 时间戳
    joint_angles: JointAngles   # 关节角度
    velocities: tuple[float, ...] # 关节速度 (可选)
    accelerations: tuple[float, ...] # 关节加速度 (可选)
    
    def __post_init__(self):
        if self.velocities and len(self.velocities) != 6:
            raise ValueError("速度必须为6个关节")
        if self.accelerations and len(self.accelerations) != 6:
            raise ValueError("加速度必须为6个关节")


@dataclass(frozen=True)
class TrajectorySegment:
    """轨迹段"""
    start_angles: JointAngles
    end_angles: JointAngles
    curve_type: CurveType
    velocity_profile: VelocityProfile
    duration: float
    
    def __post_init__(self):
        if self.duration <= 0:
            raise ValueError("轨迹持续时间必须大于0")


@dataclass(frozen=True)
class ContourParameters:
    """轮廓参数"""
    speeds: tuple[float, ...]         # 轮廓速度 (6个关节)
    accelerations: tuple[float, ...]  # 轮廓加速度 (6个关节)
    decelerations: tuple[float, ...]  # 轮廓减速度 (6个关节)
    
    def __post_init__(self):
        if len(self.speeds) != 6:
            raise ValueError("必须提供6个关节的轮廓速度")
        if len(self.accelerations) != 6:
            raise ValueError("必须提供6个关节的轮廓加速度")
        if len(self.decelerations) != 6:
            raise ValueError("必须提供6个关节的轮廓减速度")
    
    @classmethod
    def zero(cls) -> 'ContourParameters':
        """零轮廓参数"""
        return cls(
            speeds=tuple([0.0] * 6),
            accelerations=tuple([0.0] * 6), 
            decelerations=tuple([0.0] * 6)
        ) 