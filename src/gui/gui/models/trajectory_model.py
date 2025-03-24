"""
轨迹控制模型模块
"""
from PyQt5.QtCore import QObject, pyqtSignal
import numpy as np
from typing import List, Tuple, Optional

class TrajectoryModel(QObject):
    """轨迹控制模型类"""
    
    # 信号定义
    trajectory_updated = pyqtSignal(list, list)  # 轨迹更新信号
    trajectory_finished = pyqtSignal(bool, list)  # 轨迹完成信号
    trajectory_error = pyqtSignal(str)  # 错误信号
    
    def __init__(self):
        super().__init__()
        self.trajectory_points = []  # 轨迹点列表
        self.trajectory_times = []  # 轨迹时间列表
        self.current_index = 0  # 当前轨迹点索引
        self.is_running = False  # 轨迹是否正在运行
        self.is_paused = False  # 轨迹是否暂停
        self.frequency = 0.01  # 轨迹发送频率
        self.start_angles = None  # 起始角度
        self.end_angles = None  # 目标角度
        self.duration = 5.0  # 轨迹总时长
        self.curve_type = "Trapezoid"  # 轨迹类型
    
    def set_trajectory_params(self, start_angles: List[float], end_angles: List[float], 
                            duration: float = 5.0, frequency: float = 0.01, 
                            curve_type: str = "Trapezoid") -> None:
        """设置轨迹参数
        
        Args:
            start_angles: 起始角度列表
            end_angles: 目标角度列表
            duration: 轨迹总时长(秒)
            frequency: 发送频率(秒)
            curve_type: 轨迹类型，"Trapezoid"或"S-Curve"
        """
        self.start_angles = start_angles
        self.end_angles = end_angles
        self.duration = duration
        self.frequency = frequency
        self.curve_type = curve_type
        
        # 生成轨迹点
        self._generate_trajectory()
    
    def _generate_trajectory(self) -> None:
        """生成轨迹点和时间列表"""
        try:
            # 轨迹点数量
            num_points = int(self.duration / self.frequency) + 1
            
            # 创建时间列表
            times = np.linspace(0, self.duration, num_points)
            
            # 针对每个关节生成轨迹
            all_joint_positions = []
            
            for i in range(len(self.start_angles)):
                start = self.start_angles[i]
                end = self.end_angles[i]
                
                if self.curve_type == "Trapezoid":
                    # 梯形速度规划
                    accel_time = self.duration / 4.0
                    
                    positions = []
                    for t in times:
                        if t < accel_time:  # 加速阶段
                            pos = start + (end - start) * (t / self.duration) * (t / accel_time) / 2.0
                        elif t < self.duration - accel_time:  # 匀速阶段
                            pos = start + (end - start) * (t / self.duration - (accel_time / self.duration) / 4.0)
                        else:  # 减速阶段
                            decel_t = self.duration - t
                            pos = end - (end - start) * (decel_t / self.duration) * (decel_t / accel_time) / 2.0
                        positions.append(pos)
                else:  # S-Curve
                    # S形速度规划
                    positions = []
                    for t in times:
                        normalized_t = t / self.duration
                        scale = (1 - np.cos(np.pi * normalized_t)) / 2.0
                        pos = start + (end - start) * scale
                        positions.append(pos)
                
                all_joint_positions.append(positions)
            
            # 将每个关节的轨迹转置为时间点的列表
            self.trajectory_points = []
            for i in range(num_points):
                point = [all_joint_positions[j][i] for j in range(len(self.start_angles))]
                self.trajectory_points.append(point)
            
            self.trajectory_times = times.tolist()
            self.current_index = 0
            
            # 发送轨迹更新信号
            self.trajectory_updated.emit(self.trajectory_points, self.trajectory_times)
            
        except Exception as e:
            self.trajectory_error.emit(f"生成轨迹失败: {str(e)}")
    
    def get_current_point(self) -> Optional[List[float]]:
        """获取当前轨迹点
        
        Returns:
            当前轨迹点，如果没有更多点则返回None
        """
        if self.current_index < len(self.trajectory_points):
            return self.trajectory_points[self.current_index]
        return None
    
    def get_next_point(self) -> Optional[List[float]]:
        """获取下一个轨迹点
        
        Returns:
            下一个轨迹点，如果没有更多点则返回None
        """
        if self.current_index + 1 < len(self.trajectory_points):
            return self.trajectory_points[self.current_index + 1]
        return None
    
    def advance_to_next_point(self) -> None:
        """前进到下一个轨迹点"""
        if self.current_index < len(self.trajectory_points):
            self.current_index += 1
    
    def reset(self) -> None:
        """重置轨迹状态"""
        self.current_index = 0
        self.is_running = False
        self.is_paused = False
    
    def is_finished(self) -> bool:
        """检查轨迹是否完成
        
        Returns:
            是否完成
        """
        return self.current_index >= len(self.trajectory_points)
    
    def get_progress(self) -> Tuple[int, int]:
        """获取轨迹进度
        
        Returns:
            (当前点索引, 总点数)
        """
        return self.current_index, len(self.trajectory_points) 