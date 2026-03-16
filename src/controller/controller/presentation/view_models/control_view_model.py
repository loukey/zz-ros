import numpy as np
from scipy.spatial.transform import Rotation as R
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from controller.application import CommandHubService
from controller.domain.services.algorithm import KinematicDomainService
from controller.domain.services.state import RobotStateDomainService


class ControlViewModel(BaseViewModel):
    """控制视图模型。

    负责处理用户输入的控制命令，并分发到命令中心服务。

    Attributes:
        command_hub_service (CommandHubService): 命令中心服务。
    """

    pose_error = pyqtSignal(str)

    def __init__(self, command_hub_service: CommandHubService,
                 kinematic_domain_service: KinematicDomainService,
                 robot_state_domain_service: RobotStateDomainService,
                 parent=None):
        """初始化控制视图模型。

        Args:
            command_hub_service (CommandHubService): 命令中心服务。
            kinematic_domain_service (KinematicDomainService): 运动学域服务。
            robot_state_domain_service (RobotStateDomainService): 机器人状态域服务。
            parent (QObject, optional): 父对象. Defaults to None.
        """
        super().__init__(parent)
        self.command_hub_service = command_hub_service
        self.kinematic_domain_service = kinematic_domain_service
        self.robot_state_domain_service = robot_state_domain_service

    def send_command(self, config_dict: dict):
        """发送控制命令。

        Args:
            config_dict (dict): 命令配置字典，包含控制参数。
        """
        self.command_hub_service.command_distribution(config_dict)

    def get_current_pose(self) -> tuple[list[float], list[float]]:
        """获取当前位姿（欧拉角+位置）。

        Returns:
            tuple: (euler_xyz, position_xyz)
                - euler_xyz: xyz内旋欧拉角 [rx, ry, rz]（弧度）
                - position_xyz: 位置 [x, y, z]
        """
        current_angles = self.robot_state_domain_service.get_current_angles()
        quat, pos = self.kinematic_domain_service.get_gripper2base(current_angles)
        euler = R.from_quat(quat).as_euler('xyz')
        return euler.tolist(), pos.tolist()

    def send_pose_command(self, euler_xyz: list[float], position_xyz: list[float],
                          run_mode: int = 0x08, contour_params: dict = None):
        """通过欧拉角+位置发送姿态控制命令。

        Args:
            euler_xyz: xyz内旋欧拉角 [rx, ry, rz]（弧度）
            position_xyz: 目标位置 [x, y, z]
            run_mode: 运行模式，默认周期同步位置模式
            contour_params: 轮廓参数
        """
        try:
            rm = R.from_euler('xyz', euler_xyz).as_matrix()
            pos = np.array(position_xyz)
            current_angles = self.robot_state_domain_service.get_current_angles()
            target_angles = self.kinematic_domain_service.inverse_kinematic(
                rm, pos, current_angles
            )
            config = {
                'control': 0x06,
                'mode': run_mode,
                'target_angles': target_angles,
                'contour_params': contour_params
            }
            self.command_hub_service.command_distribution(config)
        except ValueError as e:
            self.pose_error.emit(f"逆运动学求解失败: {e}")
    