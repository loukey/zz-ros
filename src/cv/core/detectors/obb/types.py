from dataclasses import dataclass, field
import numpy as np
from typing import List, Optional, Tuple, Dict

@dataclass
class OBBDetection:
    """单个OBB检测结果"""
    class_id: int
    class_name: str
    confidence: float
    corners: np.ndarray    # shape (4, 2), 四个角点坐标
    center: np.ndarray     # shape (2,), 几何中心点
    angle: float = 0.0     # 旋转角度
    
    # 原始数据备份
    raw_coords: Optional[np.ndarray] = None 

@dataclass
class RobotPoseResult:
    """最终的机器人姿态结果"""
    central: OBBDetection
    head: Optional[OBBDetection]
    
    # 计算得出的结果
    final_angle: float     # 最终计算的角度
    real_center: np.ndarray # 修正后的真实中心点
    direction_vector: np.ndarray # 方向向量
    
    # 可视化所需的辅助数据
    arrow_start: Optional[np.ndarray] = None
    arrow_end: Optional[np.ndarray] = None

