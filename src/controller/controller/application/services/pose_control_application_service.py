"""
姿态控制应用服务
"""
from PyQt5.QtCore import pyqtSignal
from scipy.spatial.transform import Rotation as R
from .base_service import BaseService
from ..commands import MessageDisplay
from controller.domain import KinematicDomainService, RobotStateDomainService


class PoseControlApplicationService(BaseService):
    """姿态控制应用服务。

    封装绝对姿态和相对姿态（末端坐标系）控制的业务逻辑，
    协调 KinematicDomainService 和 RobotStateDomainService 完成位姿解算。

    Attributes:
        pose_error (pyqtSignal): 姿态控制错误信号。
    """

    pose_error = pyqtSignal(str)

    def __init__(self, message_display: MessageDisplay,
                 kinematic_service: KinematicDomainService,
                 robot_state_service: RobotStateDomainService):
        """初始化姿态控制应用服务。

        Args:
            message_display (MessageDisplay): 消息显示服务。
            kinematic_service (KinematicDomainService): 运动学域服务。
            robot_state_service (RobotStateDomainService): 机器人状态域服务。
        """
        super().__init__(message_display)
        self.kinematic_service = kinematic_service
        self.robot_state_service = robot_state_service

    def get_current_pose(self) -> tuple[list[float], list[float]]:
        """获取当前位姿（欧拉角+位置）。

        Returns:
            tuple: (euler_xyz, position_xyz)
                - euler_xyz: xyz内旋欧拉角 [rx, ry, rz]（弧度）
                - position_xyz: 位置 [x, y, z]
        """
        current_angles = self.robot_state_service.get_current_angles()
        quat, pos = self.kinematic_service.get_gripper2base(current_angles)
        euler = R.from_quat(quat).as_euler('xyz')
        return euler.tolist(), pos.tolist()

    def solve_absolute_pose(self, euler_xyz: list[float], position_xyz: list[float]) -> list[float]:
        """通过绝对欧拉角+位置求解目标关节角度。

        Args:
            euler_xyz: xyz内旋欧拉角 [rx, ry, rz]（弧度）
            position_xyz: 目标位置 [x, y, z]

        Returns:
            list[float]: 目标关节角度。

        Raises:
            ValueError: 逆运动学求解失败。
        """
        rm = R.from_euler('xyz', euler_xyz).as_matrix()
        current_angles = self.robot_state_service.get_current_angles()
        return self.kinematic_service.inverse_kinematic(rm, position_xyz, current_angles)

    def solve_relative_pose(self, euler_xyz: list[float], position_xyz: list[float]) -> list[float]:
        """在末端执行器坐标系下通过增量欧拉角+位置求解目标关节角度。

        Args:
            euler_xyz: xyz内旋欧拉角增量 [rx, ry, rz]（弧度），相对于末端坐标系
            position_xyz: 位置增量 [x, y, z]（米），相对于末端坐标系

        Returns:
            list[float]: 目标关节角度。

        Raises:
            ValueError: 逆运动学求解失败。
        """
        current_angles = self.robot_state_service.get_current_angles()
        rm, pos = self.kinematic_service.apply_relative_transform(
            current_angles, euler_xyz, position_xyz
        )
        return self.kinematic_service.inverse_kinematic(rm, pos, current_angles)
