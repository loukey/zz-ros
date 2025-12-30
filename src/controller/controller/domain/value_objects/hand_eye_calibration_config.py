"""
手眼标定配置值对象
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)  # 不可变
class CameraIntrinsics:
    """相机内参。
    
    Attributes:
        fx (float): x 方向焦距。
        fy (float): y 方向焦距。
        cx (float): 光心 x 坐标。
        cy (float): 光心 y 坐标。
    """
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class TargetOffset:
    """目标偏移量（米）。
    
    Attributes:
        x (float): x 方向偏移。
        y (float): y 方向偏移。
        z (float): z 方向偏移。
    """
    x: float
    y: float
    z: float
    
    def to_array(self) -> np.ndarray:
        """转换为 numpy 数组。
        
        Returns:
            np.ndarray: [x, y, z] 数组。
        """
        return np.array([self.x, self.y, self.z])


@dataclass(frozen=True)
class EndEffectorAdjustment:
    """末端姿态调整（弧度）。
    
    Attributes:
        z_rotation (float): 绕 Z 轴旋转角度。
        y_rotation (float): 绕 Y 轴旋转角度。
        x_rotation (float): 绕 X 轴旋转角度。
    """
    z_rotation: float
    y_rotation: float
    x_rotation: float


@dataclass(frozen=True)
class HandEyeCalibrationConfig:
    """手眼标定完整配置。
    
    这是一个值对象（Value Object），不可变且无副作用。
    
    Attributes:
        hand_eye_matrix (np.ndarray): 4x4 齐次变换矩阵。
        camera_intrinsics (CameraIntrinsics): 相机内参。
        target_offset (TargetOffset): 目标偏移量。
        end_effector_adjustment (EndEffectorAdjustment): 末端姿态调整。
    """
    
    hand_eye_matrix: np.ndarray  # 4x4 齐次变换矩阵
    camera_intrinsics: CameraIntrinsics
    target_offset: TargetOffset
    end_effector_adjustment: EndEffectorAdjustment
    
    def __post_init__(self):
        """验证数据完整性。
        
        Raises:
            ValueError: 如果手眼标定矩阵形状不是 4x4。
        """
        if self.hand_eye_matrix.shape != (4, 4):
            raise ValueError("手眼标定矩阵必须是 4x4 矩阵")

