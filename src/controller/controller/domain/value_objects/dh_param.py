from dataclasses import dataclass
import numpy as np
from math import pi

@dataclass
class DHParam:
    """机械臂 DH 参数配置。
    
    存储运动学和动力学所需的 DH 参数。
    
    Attributes:
        kinematic_dh (np.array): 运动学 DH 参数表。
        dynamic_dh (np.array): 动力学 DH 参数表。
    """
    
    kinematic_dh: np.ndarray
    dynamic_dh: np.ndarray

    def __init__(self):
        """初始化默认 DH 参数。"""
        self.kinematic_dh = np.array([
            [0.0, 0.0, 0.0,  0.0],
            [0.0, -pi/2, 0.0, -pi/2],
            [0.425, pi, 0.0, 0.0],
            [0.401, pi, 0.0856, pi/2],
            [0.0, pi/2, 0.086, 0.0],
            [0.0, -pi/2, 0.231, 0.0]  
        ], dtype=float)

        self.dynamic_dh = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, -pi/2, 0.0, -pi/2],
            [0.425, pi, 0.0, 0.0],
            [0.401, pi, 0.0856, pi/2],
            [0.0, pi/2, 0.086, 0.0],
            [0.0, -pi/2, 0.0725, 0.0]
        ])

    def get_kinematic_dh(self) -> np.ndarray:
        """获取运动学 DH 参数。
        
        Returns:
            np.ndarray: 运动学 DH 参数矩阵。
        """
        return self.kinematic_dh

    def get_dynamic_dh(self) -> np.ndarray:
        """获取动力学 DH 参数。
        
        Returns:
            np.ndarray: 动力学 DH 参数矩阵。
        """
        return self.dynamic_dh
