"""
轨迹控制控制器模块
"""
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from typing import List, Optional
import time

from .serial_controller import SerialController
from .ros_controller import ROSController
from ..models.trajectory_model import TrajectoryModel

class TrajectoryController(QObject):
    """轨迹控制控制器类"""
    
    # 信号定义
    trajectory_started = pyqtSignal()  # 轨迹开始信号
    trajectory_paused = pyqtSignal()  # 轨迹暂停信号
    trajectory_resumed = pyqtSignal()  # 轨迹恢复信号
    trajectory_stopped = pyqtSignal()  # 轨迹停止信号
    trajectory_finished = pyqtSignal(bool, list)  # 轨迹完成信号
    trajectory_error = pyqtSignal(str)  # 错误信号
    progress_updated = pyqtSignal(int, int)  # 进度更新信号
    
    def __init__(self, serial_controller: SerialController, ros_controller: Optional[ROSController] = None):
        super().__init__()
        self.serial_controller = serial_controller
        self.ros_controller = ros_controller
        self.trajectory_model = TrajectoryModel()
        
        # 创建定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self._on_timer_timeout)
        
        # 连接信号
        self.trajectory_model.trajectory_updated.connect(self._on_trajectory_updated)
        self.trajectory_model.trajectory_error.connect(self._on_trajectory_error)
    
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
        self.trajectory_model.set_trajectory_params(start_angles, end_angles, duration, frequency, curve_type)
    
    def start_trajectory(self) -> None:
        """开始执行轨迹"""
        if not self.trajectory_model.is_running:
            self.trajectory_model.is_running = True
            self.trajectory_model.is_paused = False
            self.trajectory_model.reset()
            
            # 设置定时器间隔
            self.timer.setInterval(int(self.trajectory_model.frequency * 1000))
            self.timer.start()
            
            # 发送开始信号
            self.trajectory_started.emit()
            
            # 如果有ROS控制器，开始轨迹发布
            if self.ros_controller:
                self.ros_controller.start_trajectory(self.trajectory_model.trajectory_points)
    
    def pause_trajectory(self) -> None:
        """暂停轨迹执行"""
        if self.trajectory_model.is_running and not self.trajectory_model.is_paused:
            self.trajectory_model.is_paused = True
            self.timer.stop()
            
            # 发送暂停信号
            self.trajectory_paused.emit()
            
            # 如果有ROS控制器，停止轨迹发布
            if self.ros_controller:
                self.ros_controller.stop_trajectory()
    
    def resume_trajectory(self) -> None:
        """恢复轨迹执行"""
        if self.trajectory_model.is_running and self.trajectory_model.is_paused:
            self.trajectory_model.is_paused = False
            self.timer.start()
            
            # 发送恢复信号
            self.trajectory_resumed.emit()
            
            # 如果有ROS控制器，恢复轨迹发布
            if self.ros_controller:
                self.ros_controller.start_trajectory(self.trajectory_model.trajectory_points)
    
    def stop_trajectory(self) -> None:
        """停止轨迹执行"""
        if self.trajectory_model.is_running:
            self.trajectory_model.is_running = False
            self.trajectory_model.is_paused = False
            self.timer.stop()
            
            # 发送停止信号
            self.trajectory_stopped.emit()
            
            # 如果有ROS控制器，停止轨迹发布
            if self.ros_controller:
                self.ros_controller.stop_trajectory()
    
    def _on_timer_timeout(self) -> None:
        """定时器超时处理"""
        if not self.trajectory_model.is_running or self.trajectory_model.is_paused:
            return
        
        # 获取当前点
        current_point = self.trajectory_model.get_current_point()
        if current_point is None:
            self._finish_trajectory(False)
            return
        
        try:
            # 发送当前点
            self.serial_controller.send_angles(current_point)
            
            # 如果有ROS控制器，发送当前点
            if self.ros_controller:
                self.ros_controller.publish_pose(current_point)
            
            # 更新进度
            current_index, total_points = self.trajectory_model.get_progress()
            self.progress_updated.emit(current_index, total_points)
            
            # 前进到下一个点
            self.trajectory_model.advance_to_next_point()
            
            # 检查是否完成
            if self.trajectory_model.is_finished():
                self._finish_trajectory(True)
        
        except Exception as e:
            self.trajectory_error.emit(f"发送轨迹点失败: {str(e)}")
            self._finish_trajectory(False)
    
    def _finish_trajectory(self, success: bool) -> None:
        """完成轨迹执行
        
        Args:
            success: 是否成功完成
        """
        self.stop_trajectory()
        self.trajectory_finished.emit(success, self.trajectory_model.trajectory_points)
    
    def _on_trajectory_updated(self, points: List[List[float]], times: List[float]) -> None:
        """轨迹更新处理
        
        Args:
            points: 轨迹点列表
            times: 时间列表
        """
        # 可以在这里添加轨迹更新后的处理逻辑
        pass
    
    def _on_trajectory_error(self, error_msg: str) -> None:
        """轨迹错误处理
        
        Args:
            error_msg: 错误信息
        """
        self.trajectory_error.emit(error_msg) 