"""
轨迹实体 - 轨迹规划和执行的核心业务对象
"""
from dataclasses import dataclass, field
from typing import List, Optional
from datetime import datetime
from enum import Enum
from ..value_objects.pose import JointAngles
from ..value_objects.trajectory import (
    TrajectoryPoint, TrajectorySegment, CurveType, VelocityProfile
)
from .base_entity import BaseEntity


class TrajectoryStatus(Enum):
    """轨迹状态"""
    PLANNED = "已规划"
    EXECUTING = "执行中"
    COMPLETED = "已完成"
    PAUSED = "已暂停"
    STOPPED = "已停止"
    ERROR = "错误"


@dataclass
class Trajectory(BaseEntity):
    """轨迹实体"""
    name: str = "Unnamed Trajectory"
    description: str = ""
    _segments: List[TrajectorySegment] = field(default_factory=list)
    _points: List[TrajectoryPoint] = field(default_factory=list)
    _status: TrajectoryStatus = TrajectoryStatus.PLANNED
    _current_point_index: int = 0
    _total_duration: float = 0.0
    _created_at: datetime = field(default_factory=datetime.now)
    _executed_at: Optional[datetime] = None
    
    @property
    def segments(self) -> List[TrajectorySegment]:
        """获取轨迹段"""
        return self._segments.copy()
    
    @property
    def points(self) -> List[TrajectoryPoint]:
        """获取轨迹点"""
        return self._points.copy()
    
    @property
    def status(self) -> TrajectoryStatus:
        """获取状态"""
        return self._status
    
    @property
    def total_duration(self) -> float:
        """获取总持续时间"""
        return self._total_duration
    
    @property
    def progress(self) -> float:
        """获取执行进度 (0.0-1.0)"""
        if not self._points:
            return 0.0
        return self._current_point_index / len(self._points)
    
    def add_segment(self, segment: TrajectorySegment) -> None:
        """添加轨迹段"""
        if self._status == TrajectoryStatus.EXECUTING:
            raise ValueError("轨迹执行中，无法添加新段")
        
        self._segments.append(segment)
        self._total_duration += segment.duration
        
        # 领域事件：轨迹段已添加
        from ...shared.events.robot_events import TrajectorySegmentAddedEvent
        self.add_domain_event(TrajectorySegmentAddedEvent(
            trajectory_id=self.id,
            segment=segment,
            timestamp=datetime.now()
        ))
    
    def set_points(self, points: List[TrajectoryPoint]) -> None:
        """设置轨迹点序列"""
        if self._status == TrajectoryStatus.EXECUTING:
            raise ValueError("轨迹执行中，无法修改轨迹点")
        
        # 验证轨迹点时间顺序
        for i in range(1, len(points)):
            if points[i].time <= points[i-1].time:
                raise ValueError(f"轨迹点时间顺序错误：索引{i}")
        
        self._points = points.copy()
        if points:
            self._total_duration = points[-1].time
        else:
            self._total_duration = 0.0
        
        # 重置执行状态
        self._current_point_index = 0
        self._status = TrajectoryStatus.PLANNED
        
        # 领域事件：轨迹点已设置
        from ...shared.events.robot_events import TrajectoryPointsSetEvent
        self.add_domain_event(TrajectoryPointsSetEvent(
            trajectory_id=self.id,
            point_count=len(points),
            duration=self._total_duration,
            timestamp=datetime.now()
        ))
    
    def start_execution(self) -> None:
        """开始执行轨迹"""
        if not self._points:
            raise ValueError("轨迹为空，无法执行")
        
        if self._status in [TrajectoryStatus.EXECUTING]:
            raise ValueError(f"轨迹状态错误：{self._status.value}")
        
        self._status = TrajectoryStatus.EXECUTING
        self._current_point_index = 0
        self._executed_at = datetime.now()
        
        # 领域事件：轨迹执行开始
        from ...shared.events.robot_events import TrajectoryExecutionStartedEvent
        self.add_domain_event(TrajectoryExecutionStartedEvent(
            trajectory_id=self.id,
            start_time=self._executed_at,
            total_points=len(self._points)
        ))
    
    def pause_execution(self) -> None:
        """暂停执行"""
        if self._status != TrajectoryStatus.EXECUTING:
            raise ValueError(f"无法暂停：当前状态为{self._status.value}")
        
        self._status = TrajectoryStatus.PAUSED
        
        # 领域事件：轨迹执行暂停
        from ...shared.events.robot_events import TrajectoryExecutionPausedEvent
        self.add_domain_event(TrajectoryExecutionPausedEvent(
            trajectory_id=self.id,
            paused_at_index=self._current_point_index,
            timestamp=datetime.now()
        ))
    
    def resume_execution(self) -> None:
        """恢复执行"""
        if self._status != TrajectoryStatus.PAUSED:
            raise ValueError(f"无法恢复：当前状态为{self._status.value}")
        
        self._status = TrajectoryStatus.EXECUTING
        
        # 领域事件：轨迹执行恢复
        from ...shared.events.robot_events import TrajectoryExecutionResumedEvent
        self.add_domain_event(TrajectoryExecutionResumedEvent(
            trajectory_id=self.id,
            resumed_at_index=self._current_point_index,
            timestamp=datetime.now()
        ))
    
    def stop_execution(self) -> None:
        """停止执行"""
        if self._status not in [TrajectoryStatus.EXECUTING, TrajectoryStatus.PAUSED]:
            return
        
        self._status = TrajectoryStatus.STOPPED
        
        # 领域事件：轨迹执行停止
        from ...shared.events.robot_events import TrajectoryExecutionStoppedEvent
        self.add_domain_event(TrajectoryExecutionStoppedEvent(
            trajectory_id=self.id,
            stopped_at_index=self._current_point_index,
            timestamp=datetime.now()
        ))
    
    def advance_to_next_point(self) -> Optional[TrajectoryPoint]:
        """前进到下一个轨迹点"""
        if self._status != TrajectoryStatus.EXECUTING:
            return None
        
        if self._current_point_index >= len(self._points):
            # 轨迹执行完成
            self._status = TrajectoryStatus.COMPLETED
            
            # 领域事件：轨迹执行完成
            from ...shared.events.robot_events import TrajectoryExecutionCompletedEvent
            self.add_domain_event(TrajectoryExecutionCompletedEvent(
                trajectory_id=self.id,
                completion_time=datetime.now(),
                total_points=len(self._points)
            ))
            return None
        
        current_point = self._points[self._current_point_index]
        self._current_point_index += 1
        
        return current_point
    
    def get_current_point(self) -> Optional[TrajectoryPoint]:
        """获取当前轨迹点"""
        if (self._current_point_index > 0 and 
            self._current_point_index <= len(self._points)):
            return self._points[self._current_point_index - 1]
        return None
    
    def get_next_point(self) -> Optional[TrajectoryPoint]:
        """获取下一个轨迹点"""
        if self._current_point_index < len(self._points):
            return self._points[self._current_point_index]
        return None
    
    def validate(self) -> List[str]:
        """验证轨迹有效性"""
        errors = []
        
        if not self._points:
            errors.append("轨迹点列表为空")
            return errors
        
        # 检查时间单调性
        for i in range(1, len(self._points)):
            if self._points[i].time <= self._points[i-1].time:
                errors.append(f"时间顺序错误：点{i}的时间不大于点{i-1}")
        
        # 检查关节角度连续性
        for i in range(1, len(self._points)):
            prev_angles = self._points[i-1].joint_angles.angles
            curr_angles = self._points[i].joint_angles.angles
            
            for j, (prev_angle, curr_angle) in enumerate(zip(prev_angles, curr_angles)):
                angle_diff = abs(curr_angle - prev_angle)
                if angle_diff > 0.5:  # 约28.6度
                    errors.append(f"关节{j+1}在点{i-1}到点{i}间变化过大：{angle_diff:.3f}弧度")
        
        return errors
    
    def clear(self) -> None:
        """清空轨迹"""
        if self._status == TrajectoryStatus.EXECUTING:
            raise ValueError("轨迹执行中，无法清空")
        
        self._segments.clear()
        self._points.clear()
        self._total_duration = 0.0
        self._current_point_index = 0
        self._status = TrajectoryStatus.PLANNED 
        