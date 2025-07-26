"""
Domain Value Objects - 领域值对象
不可变的业务数据对象
"""
from .pose import JointAngles, Position, Orientation, Pose
from .trajectory import CurveType, VelocityProfile, TrajectoryPoint, TrajectorySegment, ContourParameters
from .command import ControlMode, RunMode, EffectorMode, SerialCommand, SerialConfig
from .detection import Point2D, DetectionResult, CameraSettings, CameraFrame

__all__ = [
    'JointAngles', 'Position', 'Orientation', 'Pose',
    'CurveType', 'VelocityProfile', 'TrajectoryPoint', 'TrajectorySegment', 'ContourParameters',
    'ControlMode', 'RunMode', 'EffectorMode', 'SerialCommand', 'SerialConfig',
    'Point2D', 'DetectionResult', 'CameraSettings', 'CameraFrame'
] 