"""
位姿相关值对象
"""
from dataclasses import dataclass
from typing import List
import numpy as np
from math import pi


@dataclass(frozen=True)
class JointAngles:
    """关节角度值对象"""
    angles: tuple[float, ...]  # 6个关节角度（弧度）
    
    def __post_init__(self):
        if len(self.angles) != 6:
            raise ValueError("必须提供6个关节角度")
    
    @classmethod
    def from_degrees(cls, angles_deg: List[float]) -> 'JointAngles':
        """从角度创建"""
        return cls(tuple(np.radians(angles_deg)))
    
    @classmethod 
    def from_list(cls, angles: List[float]) -> 'JointAngles':
        """从列表创建"""
        return cls(tuple(angles))
    
    @classmethod
    def zero(cls) -> 'JointAngles':
        """零角度"""
        return cls((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    
    @classmethod
    def initial_pose(cls) -> 'JointAngles':
        """初始姿态"""
        return cls((0.0, -pi/2, 0.0, pi/2, 0.0, 0.0))
    
    def to_list(self) -> List[float]:
        """转换为列表"""
        return list(self.angles)
    
    def to_degrees(self) -> List[float]:
        """转换为角度"""
        return [np.degrees(angle) for angle in self.angles]


@dataclass(frozen=True)
class Position:
    """位置值对象"""
    x: float
    y: float  
    z: float
    
    def to_array(self) -> np.ndarray:
        """转换为numpy数组"""
        return np.array([self.x, self.y, self.z])
    
    def distance_to(self, other: 'Position') -> float:
        """计算到另一点的距离"""
        return np.linalg.norm(self.to_array() - other.to_array())


@dataclass(frozen=True)
class Orientation:
    """姿态值对象（四元数）"""
    x: float
    y: float
    z: float
    w: float
    
    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> 'Orientation':
        """从欧拉角创建"""
        from scipy.spatial.transform import Rotation as R
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        return cls(quat[0], quat[1], quat[2], quat[3])
    
    def to_array(self) -> np.ndarray:
        """转换为numpy数组"""
        return np.array([self.x, self.y, self.z, self.w])
    
    def to_matrix(self) -> np.ndarray:
        """转换为旋转矩阵"""
        from scipy.spatial.transform import Rotation as R
        return R.from_quat([self.x, self.y, self.z, self.w]).as_matrix()


@dataclass(frozen=True)
class Pose:
    """位姿值对象（位置 + 姿态）"""
    position: Position
    orientation: Orientation
    
    @classmethod
    def from_transform_matrix(cls, matrix: np.ndarray) -> 'Pose':
        """从变换矩阵创建"""
        from scipy.spatial.transform import Rotation as R
        position = Position(matrix[0, 3], matrix[1, 3], matrix[2, 3])
        quat = R.from_matrix(matrix[:3, :3]).as_quat()
        orientation = Orientation(quat[0], quat[1], quat[2], quat[3])
        return cls(position, orientation)
    
    def to_transform_matrix(self) -> np.ndarray:
        """转换为变换矩阵"""
        matrix = np.eye(4)
        matrix[:3, :3] = self.orientation.to_matrix()
        matrix[:3, 3] = self.position.to_array()
        return matrix 