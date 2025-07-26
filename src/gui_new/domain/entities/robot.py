"""
机器人实体 - 核心业务对象
"""
from dataclasses import dataclass, field
from typing import Optional, List
from datetime import datetime
from ..value_objects.pose import JointAngles, Pose
from ..value_objects.trajectory import TrajectoryPoint
from ..value_objects.command import SerialConfig, ControlMode
from .base_entity import BaseEntity


@dataclass
class RobotState:
    """机器人状态"""
    current_angles: JointAngles
    current_pose: Optional[Pose] = None
    velocities: Optional[tuple[float, ...]] = None
    torques: Optional[tuple[float, ...]] = None
    is_moving: bool = False
    last_updated: datetime = field(default_factory=datetime.now)


@dataclass 
class Robot(BaseEntity):
    """机器人实体"""
    model_name: str = "Unknown"
    serial_number: str = "Unknown"
    dof: int = 6  # 自由度
    _state: RobotState = field(default_factory=lambda: RobotState(JointAngles.initial_pose()))
    _connection_config: Optional[SerialConfig] = None
    _is_connected: bool = False
    _trajectory_history: List[TrajectoryPoint] = field(default_factory=list)
    
    def __post_init__(self):
        super().__post_init__()
        if self.dof != 6:
            raise ValueError("当前仅支持6自由度机器人")
    
    @property
    def current_state(self) -> RobotState:
        """获取当前状态"""
        return self._state
    
    @property 
    def is_connected(self) -> bool:
        """是否已连接"""
        return self._is_connected
    
    def update_joint_angles(self, angles: JointAngles) -> None:
        """更新关节角度"""
        self._state.current_angles = angles
        self._state.last_updated = datetime.now()
        
        # 领域事件：关节角度已更新
        from shared.events.robot_events import JointAnglesUpdatedEvent
        self.add_domain_event(JointAnglesUpdatedEvent(
            robot_id=self.id,
            joint_angles=angles,
            timestamp=self._state.last_updated
        ))
    
    def update_pose(self, pose: Pose) -> None:
        """更新末端执行器位姿"""
        self._state.current_pose = pose
        self._state.last_updated = datetime.now()
        
        # 领域事件：位姿已更新
        from shared.events.robot_events import RobotPoseUpdatedEvent
        self.add_domain_event(RobotPoseUpdatedEvent(
            robot_id=self.id,
            pose=pose,
            timestamp=self._state.last_updated
        ))
    
    def start_movement(self) -> None:
        """开始运动"""
        if not self._is_connected:
            raise ValueError("机器人未连接，无法开始运动")
        
        self._state.is_moving = True
        
        # 领域事件：运动开始
        from shared.events.robot_events import RobotMovementStartedEvent
        self.add_domain_event(RobotMovementStartedEvent(
            robot_id=self.id,
            timestamp=datetime.now()
        ))
    
    def stop_movement(self) -> None:
        """停止运动"""
        self._state.is_moving = False
        
        # 领域事件：运动停止
        from shared.events.robot_events import RobotMovementStoppedEvent
        self.add_domain_event(RobotMovementStoppedEvent(
            robot_id=self.id,
            timestamp=datetime.now()
        ))
    
    def connect(self, config: SerialConfig) -> None:
        """连接机器人"""
        self._connection_config = config
        self._is_connected = True
        
        # 领域事件：连接已建立
        from shared.events.robot_events import RobotConnectedEvent
        self.add_domain_event(RobotConnectedEvent(
            robot_id=self.id,
            config=config,
            timestamp=datetime.now()
        ))
    
    def disconnect(self) -> None:
        """断开连接"""
        if self._state.is_moving:
            self.stop_movement()
        
        self._is_connected = False
        self._connection_config = None
        
        # 领域事件：连接已断开
        from shared.events.robot_events import RobotDisconnectedEvent
        self.add_domain_event(RobotDisconnectedEvent(
            robot_id=self.id,
            timestamp=datetime.now()
        ))
    
    def add_trajectory_point(self, point: TrajectoryPoint) -> None:
        """添加轨迹点到历史记录"""
        self._trajectory_history.append(point)
        
        # 保持历史记录在合理范围内
        if len(self._trajectory_history) > 1000:
            self._trajectory_history = self._trajectory_history[-500:]
    
    def clear_trajectory_history(self) -> None:
        """清除轨迹历史"""
        self._trajectory_history.clear()
    
    def get_trajectory_history(self) -> List[TrajectoryPoint]:
        """获取轨迹历史"""
        return self._trajectory_history.copy()
    
    def validate_joint_angles(self, angles: JointAngles) -> bool:
        """验证关节角度是否在安全范围内"""
        # 这里应该包含实际的关节限位检查
        # 简化版本：检查是否在合理范围内
        from math import pi
        limits = [
            (-pi, pi),      # 关节1
            (-pi/2, pi/2),  # 关节2  
            (-pi, pi),      # 关节3
            (-pi, pi),      # 关节4
            (-pi/2, pi/2),  # 关节5
            (-pi, pi)       # 关节6
        ]
        
        for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles.angles, limits)):
            if not (min_limit <= angle <= max_limit):
                return False
        
        return True 