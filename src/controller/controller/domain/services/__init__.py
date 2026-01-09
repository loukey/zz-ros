"""
Domain服务层

按职责分为六个子模块：
- algorithm: 纯算法计算（运动学、动力学、轨迹、平滑）
- motion: 运动控制（构造器、执行器）
- communication: 通信服务（串口、消息编解码）
- state: 状态管理（机器人状态、示教记录）
- planning: 运动规划管理
- vision: 视觉服务（摄像头、识别）
"""
from .algorithm import KinematicDomainService, DynamicDomainService, SCurve, SmoothDomainService, LinearMotionDomainService, HandEyeTransformDomainService, CurveMotionDomainService, LinearMotionBlendDomainService
from .motion import MotionRunner, MotionConstructor, TrajectoryPlanningService
from .communication import SerialDomainService, MessageDomainService
from .state import RobotStateDomainService, TeachRecordDomainService
from .planning import MotionPlanningDomainService
from .vision import CameraDomainService, RecognitionDomainService

__all__ = [
    # Algorithm
    'KinematicDomainService', 
    'DynamicDomainService', 
    'SCurve',
    'SmoothDomainService',
    'LinearMotionDomainService',
    'HandEyeTransformDomainService',
    'CurveMotionDomainService',
    'LinearMotionBlendDomainService',
    # Motion
    'MotionRunner',
    'MotionConstructor',
    'TrajectoryPlanningService',
    # Communication
    'SerialDomainService',
    'MessageDomainService',
    # State
    'RobotStateDomainService',
    'TeachRecordDomainService',
    # Planning
    'MotionPlanningDomainService',
    # Vision
    'CameraDomainService',
    'RecognitionDomainService',
]
