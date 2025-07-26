"""
机器人领域事件定义
"""
from dataclasses import dataclass
from typing import List, Any, Optional
from datetime import datetime


@dataclass
class DomainEvent:
    """领域事件基类"""
    occurred_at: datetime


# 机器人连接相关事件
@dataclass
class RobotConnectedEvent(DomainEvent):
    """机器人连接事件"""
    robot_id: str
    config: Any
    
    def __init__(self, robot_id: str, config: Any, timestamp: datetime):
        self.robot_id = robot_id
        self.config = config
        self.occurred_at = timestamp


@dataclass
class RobotDisconnectedEvent(DomainEvent):
    """机器人断开连接事件"""
    robot_id: str
    
    def __init__(self, robot_id: str, timestamp: datetime):
        self.robot_id = robot_id
        self.occurred_at = timestamp


# 机器人状态更新事件
@dataclass
class JointAnglesUpdatedEvent(DomainEvent):
    """关节角度更新事件"""
    robot_id: str
    joint_angles: Any
    
    def __init__(self, robot_id: str, joint_angles: Any, timestamp: datetime):
        self.robot_id = robot_id
        self.joint_angles = joint_angles
        self.occurred_at = timestamp


@dataclass
class RobotPoseUpdatedEvent(DomainEvent):
    """机器人位姿更新事件"""
    robot_id: str
    pose: Any
    
    def __init__(self, robot_id: str, pose: Any, timestamp: datetime):
        self.robot_id = robot_id
        self.pose = pose
        self.occurred_at = timestamp


# 机器人运动事件
@dataclass
class RobotMovementStartedEvent(DomainEvent):
    """机器人开始运动事件"""
    robot_id: str
    
    def __init__(self, robot_id: str, timestamp: datetime):
        self.robot_id = robot_id
        self.occurred_at = timestamp


@dataclass
class RobotMovementStoppedEvent(DomainEvent):
    """机器人停止运动事件"""
    robot_id: str
    
    def __init__(self, robot_id: str, timestamp: datetime):
        self.robot_id = robot_id
        self.occurred_at = timestamp


# 轨迹相关事件
@dataclass
class TrajectorySegmentAddedEvent(DomainEvent):
    """轨迹段添加事件"""
    trajectory_id: str
    segment: Any
    
    def __init__(self, trajectory_id: str, segment: Any, timestamp: datetime):
        self.trajectory_id = trajectory_id
        self.segment = segment
        self.occurred_at = timestamp


@dataclass
class TrajectoryPointsSetEvent(DomainEvent):
    """轨迹点设置事件"""
    trajectory_id: str
    point_count: int
    duration: float
    
    def __init__(self, trajectory_id: str, point_count: int, duration: float, timestamp: datetime):
        self.trajectory_id = trajectory_id
        self.point_count = point_count
        self.duration = duration
        self.occurred_at = timestamp


@dataclass
class TrajectoryExecutionStartedEvent(DomainEvent):
    """轨迹执行开始事件"""
    trajectory_id: str
    start_time: datetime
    total_points: int
    
    def __init__(self, trajectory_id: str, start_time: datetime, total_points: int):
        self.trajectory_id = trajectory_id
        self.start_time = start_time
        self.total_points = total_points
        self.occurred_at = start_time


@dataclass
class TrajectoryExecutionPausedEvent(DomainEvent):
    """轨迹执行暂停事件"""
    trajectory_id: str
    paused_at_index: int
    
    def __init__(self, trajectory_id: str, paused_at_index: int, timestamp: datetime):
        self.trajectory_id = trajectory_id
        self.paused_at_index = paused_at_index
        self.occurred_at = timestamp


@dataclass
class TrajectoryExecutionResumedEvent(DomainEvent):
    """轨迹执行恢复事件"""
    trajectory_id: str
    resumed_at_index: int
    
    def __init__(self, trajectory_id: str, resumed_at_index: int, timestamp: datetime):
        self.trajectory_id = trajectory_id
        self.resumed_at_index = resumed_at_index
        self.occurred_at = timestamp


@dataclass
class TrajectoryExecutionStoppedEvent(DomainEvent):
    """轨迹执行停止事件"""
    trajectory_id: str
    stopped_at_index: int
    
    def __init__(self, trajectory_id: str, stopped_at_index: int, timestamp: datetime):
        self.trajectory_id = trajectory_id
        self.stopped_at_index = stopped_at_index
        self.occurred_at = timestamp


@dataclass
class TrajectoryExecutionCompletedEvent(DomainEvent):
    """轨迹执行完成事件"""
    trajectory_id: str
    completion_time: datetime
    total_points: int
    
    def __init__(self, trajectory_id: str, completion_time: datetime, total_points: int):
        self.trajectory_id = trajectory_id
        self.completion_time = completion_time
        self.total_points = total_points
        self.occurred_at = completion_time


# 错误和警告事件
@dataclass
class RobotErrorEvent(DomainEvent):
    """机器人错误事件"""
    robot_id: str
    error_code: str
    error_message: str
    
    def __init__(self, robot_id: str, error_code: str, error_message: str, timestamp: datetime):
        self.robot_id = robot_id
        self.error_code = error_code
        self.error_message = error_message
        self.occurred_at = timestamp


@dataclass
class RobotWarningEvent(DomainEvent):
    """机器人警告事件"""  
    robot_id: str
    warning_code: str
    warning_message: str
    
    def __init__(self, robot_id: str, warning_code: str, warning_message: str, timestamp: datetime):
        self.robot_id = robot_id
        self.warning_code = warning_code
        self.warning_message = warning_message
        self.occurred_at = timestamp 