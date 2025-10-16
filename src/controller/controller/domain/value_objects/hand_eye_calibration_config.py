"""
手眼标定配置值对象
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)  # 不可变
class CameraIntrinsics:
    """相机内参"""
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class TargetOffset:
    """目标偏移量（米）"""
    x: float
    y: float
    z: float
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


@dataclass(frozen=True)
class EndEffectorAdjustment:
    """末端姿态调整（弧度）"""
    y_rotation: float
    x_rotation: float


@dataclass(frozen=True)
class HandEyeCalibrationConfig:
    """
    手眼标定完整配置
    
    这是一个值对象（Value Object），不可变且无副作用
    """
    hand_eye_matrix: np.ndarray  # 4x4 齐次变换矩阵
    camera_intrinsics: CameraIntrinsics
    target_offset: TargetOffset
    end_effector_adjustment: EndEffectorAdjustment
    
    def __post_init__(self):
        """验证数据完整性"""
        if self.hand_eye_matrix.shape != (4, 4):
            raise ValueError("手眼标定矩阵必须是 4x4 矩阵")

