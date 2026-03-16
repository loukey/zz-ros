from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from controller.application import CommandHubService, PoseControlApplicationService


class ControlViewModel(BaseViewModel):
    """控制视图模型。

    负责处理用户输入的控制命令，并分发到命令中心服务。

    Attributes:
        command_hub_service (CommandHubService): 命令中心服务。
        pose_control_service (PoseControlApplicationService): 姿态控制应用服务。
    """

    pose_error = pyqtSignal(str)

    def __init__(self, command_hub_service: CommandHubService,
                 pose_control_service: PoseControlApplicationService,
                 parent=None):
        """初始化控制视图模型。

        Args:
            command_hub_service (CommandHubService): 命令中心服务。
            pose_control_service (PoseControlApplicationService): 姿态控制应用服务。
            parent (QObject, optional): 父对象. Defaults to None.
        """
        super().__init__(parent)
        self.command_hub_service = command_hub_service
        self.pose_control_service = pose_control_service

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
        return self.pose_control_service.get_current_pose()

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
            target_angles = self.pose_control_service.solve_absolute_pose(euler_xyz, position_xyz)
            config = {
                'control': 0x06,
                'mode': run_mode,
                'target_angles': target_angles,
                'contour_params': contour_params
            }
            self.command_hub_service.command_distribution(config)
        except ValueError as e:
            self.pose_error.emit(f"逆运动学求解失败: {e}")

    def send_relative_pose_command(self, euler_xyz: list[float], position_xyz: list[float],
                                    run_mode: int = 0x08, contour_params: dict = None):
        """在末端执行器坐标系下发送相对姿态控制命令。

        Args:
            euler_xyz: xyz内旋欧拉角增量 [rx, ry, rz]（弧度），相对于末端坐标系
            position_xyz: 位置增量 [x, y, z]（米），相对于末端坐标系
            run_mode: 运行模式，默认周期同步位置模式
            contour_params: 轮廓参数
        """
        try:
            target_angles = self.pose_control_service.solve_relative_pose(euler_xyz, position_xyz)
            config = {
                'control': 0x06,
                'mode': run_mode,
                'target_angles': target_angles,
                'contour_params': contour_params
            }
            self.command_hub_service.command_distribution(config)
        except ValueError as e:
            self.pose_error.emit(f"逆运动学求解失败: {e}")
