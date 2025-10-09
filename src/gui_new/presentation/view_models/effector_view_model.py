"""
末端执行器视图模型 - Presentation层
"""
from .base_view_model import BaseViewModel
from application import CommandHubService


class EffectorViewModel(BaseViewModel):
    """末端执行器视图模型"""
    
    def __init__(self, command_hub_service: CommandHubService, parent=None):
        super().__init__(parent)
        self.command_hub_service = command_hub_service
    
    def send_effector_command(self, config_dict: dict):
        """发送末端执行器命令"""
        self.command_hub_service.command_distribution(config_dict)

