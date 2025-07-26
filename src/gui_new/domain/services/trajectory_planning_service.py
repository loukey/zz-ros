"""
轨迹规划服务 - 纯业务逻辑，不依赖外部系统
"""
from typing import List, Tuple, Optional
import numpy as np
from domain.value_objects.trajectory import CurveType, VelocityProfile, TrajectoryPoint
from domain.value_objects.pose import JointAngles


class TrajectoryPlanningService:
    """轨迹规划服务"""
    
    def __init__(self):
        self.default_max_velocity = 1.0  # rad/s
        self.default_max_acceleration = 2.0  # rad/s²
        self.default_max_jerk = 10.0  # rad/s³
    
    def plan_trajectory(self, key_points: List[List[float]], curve_type: str = "S_CURVE",
                       max_velocity: float = None, max_acceleration: float = None,
                       max_jerk: float = None) -> Tuple[str, List[TrajectoryPoint]]:
        """规划轨迹"""
        try:
            # 使用默认参数
            max_vel = max_velocity or self.default_max_velocity
            max_accel = max_acceleration or self.default_max_acceleration
            max_j = max_jerk or self.default_max_jerk
            
            trajectory_id = f"traj_{len(key_points)}_{curve_type}"
            points = []
            
            # 简化实现：在关键点之间进行线性插值
            for i in range(len(key_points) - 1):
                start_point = key_points[i]
                end_point = key_points[i + 1]
                
                # 插值生成中间点
                segment_points = self._interpolate_segment(
                    start_point, end_point, curve_type, max_vel, max_accel
                )
                points.extend(segment_points)
            
            return trajectory_id, points
            
        except Exception:
            return None, []
    
    def _interpolate_segment(self, start: List[float], end: List[float], 
                           curve_type: str, max_vel: float, max_accel: float) -> List[TrajectoryPoint]:
        """插值生成轨迹段"""
        points = []
        
        # 简化实现：固定20个插值点
        num_points = 20
        
        for i in range(num_points + 1):
            t = i / num_points
            
            # 根据曲线类型计算插值系数
            if curve_type == "S_CURVE":
                # S型曲线插值
                s = self._s_curve_interpolation(t)
            elif curve_type == "TRAPEZOIDAL":
                # 梯形曲线插值
                s = self._trapezoidal_interpolation(t)
            else:
                # 线性插值
                s = t
            
            # 计算当前位置
            current_angles = []
            for j in range(len(start)):
                angle = start[j] + s * (end[j] - start[j])
                current_angles.append(angle)
            
            # 创建轨迹点
            point = TrajectoryPoint(
                angles=JointAngles.from_list(current_angles),
                time=t,
                velocity=max_vel * (1.0 - abs(2*t - 1)),  # 简化速度计算
                acceleration=0.0  # 简化加速度
            )
            points.append(point)
        
        return points
    
    def _s_curve_interpolation(self, t: float) -> float:
        """S型曲线插值"""
        if t <= 0.5:
            return 2 * t * t
        else:
            return 1 - 2 * (1 - t) * (1 - t)
    
    def _trapezoidal_interpolation(self, t: float) -> float:
        """梯形曲线插值"""
        if t <= 0.25:
            return 2 * t * t
        elif t <= 0.75:
            return 0.125 + 0.5 * t
        else:
            return 1 - 2 * (1 - t) * (1 - t)
    
    def smooth_trajectory(self, points: List[TrajectoryPoint], 
                         smoothing_factor: float = 0.1) -> List[TrajectoryPoint]:
        """轨迹平滑处理"""
        if len(points) < 3:
            return points
        
        smoothed_points = [points[0]]  # 保持第一个点
        
        for i in range(1, len(points) - 1):
            # 简单的移动平均平滑
            prev_angles = points[i-1].angles.angles
            curr_angles = points[i].angles.angles
            next_angles = points[i+1].angles.angles
            
            smoothed_angles = []
            for j in range(len(curr_angles)):
                smoothed = (prev_angles[j] + curr_angles[j] + next_angles[j]) / 3.0
                mixed = curr_angles[j] * (1 - smoothing_factor) + smoothed * smoothing_factor
                smoothed_angles.append(mixed)
            
            smoothed_point = TrajectoryPoint(
                angles=JointAngles.from_list(smoothed_angles),
                time=points[i].time,
                velocity=points[i].velocity,
                acceleration=points[i].acceleration
            )
            smoothed_points.append(smoothed_point)
        
        smoothed_points.append(points[-1])  # 保持最后一个点
        
        return smoothed_points 