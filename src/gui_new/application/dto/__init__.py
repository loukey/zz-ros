"""
Data Transfer Objects - 数据传输对象
用于层间数据传递
"""
from .robot_dto import RobotStateDTO, RobotConfigDTO, JointAnglesDTO, PoseDTO
from .command_dto import SendAnglesDTO, ControlCommandDTO, SerialConfigDTO, SerialDataDTO

__all__ = [
    'RobotStateDTO', 'RobotConfigDTO', 'JointAnglesDTO', 'PoseDTO',
    'SendAnglesDTO', 'ControlCommandDTO', 'SerialConfigDTO', 'SerialDataDTO'
] 