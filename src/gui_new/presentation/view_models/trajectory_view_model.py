"""
轨迹视图模型
"""
from typing import List, Dict, Any, Optional, Tuple
from PyQt5.QtCore import pyqtSignal, QTimer
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from application.dto.robot_dto import JointAnglesDTO
from shared.config.container import ServiceLocator
import numpy as np


class TrajectoryViewModel(BaseViewModel):
    """轨迹视图模型"""
    
    # 轨迹特有信号
    progress_changed = pyqtSignal(int, bool)  # 进度百分比, 是否完成
    trajectory_planned = pyqtSignal(int)  # 轨迹点数量
    execution_started = pyqtSignal()
    execution_completed = pyqtSignal(bool, str)  # 成功, 消息
    point_reached = pyqtSignal(int, list)  # 到达的点索引, 当前角度
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # UI状态
        self.trajectory_points = []  # 轨迹点列表 [角度数组]
        self.current_trajectory_id = None  # 当前轨迹ID
        self.is_executing = False  # 是否正在执行
        self.current_point_index = 0  # 当前执行到的点索引
        self.execution_progress = 0  # 执行进度百分比
        
        # 轨迹参数
        self.curve_type = "S_CURVE"  # 曲线类型
        self.max_velocity = 1.0  # 最大速度 (rad/s)
        self.max_acceleration = 2.0  # 最大加速度 (rad/s²)
        self.max_jerk = 10.0  # 最大加加速度 (rad/s³)
        
        # 执行定时器（模拟轨迹执行）
        self.execution_timer = QTimer()
        self.execution_timer.timeout.connect(self._update_execution_progress)
    
    def plan_trajectory(self, start_angles: List[float], end_angles: List[float], 
                       curve_type: str = "S_CURVE", 
                       velocity_params: Optional[Dict[str, float]] = None) -> bool:
        """规划轨迹"""
        try:
            # 验证输入数据
            if len(start_angles) != 6 or len(end_angles) != 6:
                self.emit_error("起始角度和结束角度必须包含6个关节")
                return False
            
            # 设置速度参数
            if velocity_params:
                self.max_velocity = velocity_params.get('max_velocity', self.max_velocity)
                self.max_acceleration = velocity_params.get('max_acceleration', self.max_acceleration)
                self.max_jerk = velocity_params.get('max_jerk', self.max_jerk)
            
            # 转换为弧度
            start_rad = [np.radians(angle) for angle in start_angles]
            end_rad = [np.radians(angle) for angle in end_angles]
            
            # 创建关键点数组（起点和终点）
            key_points = [start_rad, end_rad]
            
            # 调用Application服务规划轨迹
            trajectory_id, points = self.robot_service.plan_trajectory(
                key_points=key_points,
                curve_type=curve_type,
                max_velocity=self.max_velocity,
                max_acceleration=self.max_acceleration,
                max_jerk=self.max_jerk
            )
            
            if trajectory_id and points:
                # 转换轨迹点为度数并保存
                self.trajectory_points = []
                for point in points:
                    angles_deg = [np.degrees(angle) for angle in point.angles]
                    self.trajectory_points.append(angles_deg)
                
                self.current_trajectory_id = trajectory_id
                self.curve_type = curve_type
                
                # 发出信号
                self.trajectory_planned.emit(len(self.trajectory_points))
                self.emit_status(f"轨迹规划完成 - {len(self.trajectory_points)}个点, 曲线类型: {curve_type}")
                return True
            else:
                self.emit_error("轨迹规划失败")
                return False
                
        except Exception as e:
            self.emit_error(f"轨迹规划失败: {str(e)}")
            return False
    
    def plan_multi_point_trajectory(self, key_points: List[List[float]], 
                                  curve_type: str = "S_CURVE") -> bool:
        """规划多点轨迹"""
        try:
            # 验证输入数据
            if len(key_points) < 2:
                self.emit_error("至少需要2个关键点")
                return False
            
            for i, point in enumerate(key_points):
                if len(point) != 6:
                    self.emit_error(f"第{i+1}个关键点必须包含6个关节角度")
                    return False
            
            # 转换为弧度
            key_points_rad = []
            for point in key_points:
                point_rad = [np.radians(angle) for angle in point]
                key_points_rad.append(point_rad)
            
            # 调用Application服务规划轨迹
            trajectory_id, points = self.robot_service.plan_trajectory(
                key_points=key_points_rad,
                curve_type=curve_type,
                max_velocity=self.max_velocity,
                max_acceleration=self.max_acceleration,
                max_jerk=self.max_jerk
            )
            
            if trajectory_id and points:
                # 转换轨迹点为度数并保存
                self.trajectory_points = []
                for point in points:
                    angles_deg = [np.degrees(angle) for angle in point.angles]
                    self.trajectory_points.append(angles_deg)
                
                self.current_trajectory_id = trajectory_id
                self.curve_type = curve_type
                
                # 发出信号
                self.trajectory_planned.emit(len(self.trajectory_points))
                self.emit_status(f"多点轨迹规划完成 - {len(key_points)}个关键点, "
                               f"{len(self.trajectory_points)}个插值点")
                return True
            else:
                self.emit_error("多点轨迹规划失败")
                return False
                
        except Exception as e:
            self.emit_error(f"多点轨迹规划失败: {str(e)}")
            return False
    
    def execute_trajectory(self) -> bool:
        """执行轨迹"""
        if not self.trajectory_points:
            self.emit_error("没有可执行的轨迹")
            return False
        
        if self.is_executing:
            self.emit_error("轨迹正在执行中")
            return False
        
        try:
            # 开始执行
            self.is_executing = True
            self.current_point_index = 0
            self.execution_progress = 0
            
            # 发出开始信号
            self.execution_started.emit()
            self.emit_status(f"开始执行轨迹 - {len(self.trajectory_points)}个点")
            
            # TODO: 当Infrastructure层实现后，这里调用真实的轨迹执行服务
            # 目前使用定时器模拟执行过程
            self.execution_timer.start(100)  # 每100ms更新一次
            
            return True
            
        except Exception as e:
            self.is_executing = False
            self.emit_error(f"开始执行轨迹失败: {str(e)}")
            return False
    
    def stop_trajectory_execution(self) -> bool:
        """停止轨迹执行"""
        try:
            if not self.is_executing:
                self.emit_status("轨迹未在执行中")
                return True
            
            # 停止执行
            self.execution_timer.stop()
            self.is_executing = False
            
            # 停止机器人运动
            success, message = self.robot_service.stop_movement()
            
            self.execution_completed.emit(False, "轨迹执行已停止")
            self.emit_status("轨迹执行已停止")
            
            return success
            
        except Exception as e:
            self.emit_error(f"停止轨迹执行失败: {str(e)}")
            return False
    
    def _update_execution_progress(self):
        """更新执行进度（模拟）"""
        try:
            if not self.is_executing or not self.trajectory_points:
                return
            
            total_points = len(self.trajectory_points)
            
            # 模拟到达下一个点
            if self.current_point_index < total_points:
                current_angles = self.trajectory_points[self.current_point_index]
                
                # 发出到达点信号
                self.point_reached.emit(self.current_point_index, current_angles)
                
                # 更新进度
                self.execution_progress = int((self.current_point_index + 1) * 100 / total_points)
                self.progress_changed.emit(self.execution_progress, False)
                
                self.current_point_index += 1
            else:
                # 执行完成
                self.execution_timer.stop()
                self.is_executing = False
                self.progress_changed.emit(100, True)
                self.execution_completed.emit(True, "轨迹执行完成")
                self.emit_status("轨迹执行完成")
                
        except Exception as e:
            self.execution_timer.stop()
            self.is_executing = False
            self.emit_error(f"轨迹执行过程中出错: {str(e)}")
            self.execution_completed.emit(False, f"执行出错: {str(e)}")
    
    def get_trajectory_info(self) -> Dict[str, Any]:
        """获取轨迹信息"""
        return {
            'points_count': len(self.trajectory_points),
            'curve_type': self.curve_type,
            'is_executing': self.is_executing,
            'current_point_index': self.current_point_index,
            'progress': self.execution_progress,
            'trajectory_id': self.current_trajectory_id
        }
    
    def get_trajectory_points(self) -> List[List[float]]:
        """获取轨迹点列表"""
        return [point.copy() for point in self.trajectory_points]
    
    def clear_trajectory(self):
        """清除当前轨迹"""
        if self.is_executing:
            self.stop_trajectory_execution()
        
        self.trajectory_points.clear()
        self.current_trajectory_id = None
        self.current_point_index = 0
        self.execution_progress = 0
        
        self.emit_status("已清除轨迹")
    
    def set_trajectory_parameters(self, max_velocity: float, max_acceleration: float, 
                                max_jerk: float):
        """设置轨迹参数"""
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk
        
        self.emit_status(f"轨迹参数已更新 - 速度: {max_velocity}, "
                        f"加速度: {max_acceleration}, 加加速度: {max_jerk}")
    
    def cleanup(self):
        """清理资源"""
        super().cleanup()
        if self.execution_timer.isActive():
            self.execution_timer.stop()
        self.clear_trajectory() 