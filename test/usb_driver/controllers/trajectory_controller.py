"""
轨迹计算工具
提供轨迹点计算功能，不处理状态管理和通信
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import List, Tuple
from kinematic import *
import numpy as np

class TrajectoryController(QObject):
    """轨迹计算工具类
    
    只负责计算轨迹点，不处理通信和状态管理
    """
    
    # 只保留一个错误信号
    calculation_error = pyqtSignal(str)  # 计算错误信号
    
    def __init__(self):
        super().__init__()
    
    def calculate_trajectory(self, start_angles: List[float], end_angles: List[float], 
                          duration: float = 5.0, frequency: float = 0.1, 
                          curve_type: str = "Trapezoid") -> Tuple[List[List[float]], List[float]]:
        """计算轨迹点
        
        Args:
            start_angles: 起始角度列表
            end_angles: 目标角度列表
            duration: 轨迹总时长(秒)
            frequency: 发送频率(秒)
            curve_type: 轨迹类型，"trapezoidal"或"s-curve"
            
        Returns:
            Tuple[List[List[float]], List[float]]: (轨迹点列表, 时间点列表)
        """
        try:
            if curve_type.lower() == "trapezoidal":
                times, velocities, accelerations, positions = trapezoidal_velocity_planning(start_angles=start_angles, target_angles=end_angles, dt=frequency)
            else:
                s_curve = SCurve()
                times, velocities, accelerations, positions = s_curve.planning(start_angles=start_angles, target_angles=end_angles, dt=frequency)            
            return positions, times
            
        except Exception as e:
            import traceback
            error_msg = f"轨迹计算错误: {str(e)}"
            self.calculation_error.emit(error_msg)
            return [], []
    
    def calculate_single_point(self, angles: List[float]) -> List[float]:
        """直接返回单个点，用于单点移动模式
        
        Args:
            angles: 目标角度列表
            
        Returns:
            List[float]: 目标角度列表的副本
        """
        return angles.copy()  # 返回一个副本，避免引用原始数据 