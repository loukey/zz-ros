from .base_service import BaseService
from ..commands import MessageDisplay
from .command_hub_service import CommandHubService
from domain import SCurve


class MotionApplicationService(BaseService):
    def __init__(
        self, 
        message_display: MessageDisplay, 
        command_hub_service: CommandHubService
        ):
        super().__init__(message_display)
        self.command_hub_service = command_hub_service
        self.s_curve = SCurve()

    def send_angles(self, config_dict: dict):
        control = config_dict.get('control')
        mode = config_dict.get('mode')
        joint_angles = config_dict.get('joint_angles')
        pass

    def multi_motion(self, config_dict: dict):
        pass

    def teach_motion(self, config_dict: dict):
        pass
    