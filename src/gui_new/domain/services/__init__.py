"""
Domain服务层

按职责分为四个子模块：
- algorithm: 纯算法计算（运动学、动力学、轨迹、平滑）
- motion: 运动控制（构造器、执行器）
- communication: 通信服务（串口、消息编解码）
- state: 状态管理（机器人状态、示教记录）
"""
from .algorithm import KinematicDomainService, DynamicDomainService, SCurve, SmoothDomainService
from .motion import MotionRunner, MotionConstructor
from .communication import SerialDomainService, MessageDomainService
from .state import RobotStateDomainService, TeachRecordDomainService

__all__ = [
    # Algorithm
    'KinematicDomainService', 
    'DynamicDomainService', 
    'SCurve',
    'SmoothDomainService',
    # Motion
    'MotionRunner',
    'MotionConstructor',
    # Communication
    'SerialDomainService',
    'MessageDomainService',
    # State
    'RobotStateDomainService',
    'TeachRecordDomainService',
]
