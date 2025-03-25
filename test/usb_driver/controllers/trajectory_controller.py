"""
轨迹计算工具
提供轨迹点计算功能，不处理状态管理和通信
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import List, Tuple
from kinematic.velocity_planning import trapezoidal_velocity_planning, s_curve_velocity_planning
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
            curve_type: 轨迹类型，"Trapezoid"或"S-Curve"
            
        Returns:
            Tuple[List[List[float]], List[float]]: (轨迹点列表, 时间点列表)
        """
        try:
            print(f"开始计算轨迹: 类型={curve_type}, 持续时间={duration}秒, 频率={frequency}秒")
            print(f"起始角度: {start_angles}")
            print(f"目标角度: {end_angles}")
            
            # 计算角度差值
            angles_diff = [end - start for end, start in zip(end_angles, start_angles)]
            print(f"角度差值: {angles_diff}")
            
            # 根据曲线类型选择不同的轨迹计算函数
            if curve_type.lower() == "trapezoidal":
                # 梯形速度曲线
                print("使用梯形速度曲线规划")
                times, velocities, accelerations, positions = trapezoidal_velocity_planning(angles=angles_diff, dt=frequency)
            else:
                # S型速度曲线
                print("使用S型速度曲线规划")
                times, velocities, accelerations, positions = s_curve_velocity_planning(angles=angles_diff, dt=frequency)
            print(f"轨迹计算完成: 共{len(positions)}个点, 总时长{times[-1] if len(times) > 0 else 0}秒")
            
            return positions, times
            
        except Exception as e:
            import traceback
            error_msg = f"轨迹计算错误: {str(e)}"
            print(error_msg)
            print(traceback.format_exc())
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