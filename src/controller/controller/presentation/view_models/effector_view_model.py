"""
末端执行器视图模型 - Presentation层
"""
from .base_view_model import BaseViewModel
from controller.application import CommandHubService


class EffectorViewModel(BaseViewModel):
    """末端执行器视图模型。
    
    负责处理末端执行器（如夹爪、吸盘）的控制命令。
    
    Attributes:
        command_hub_service (CommandHubService): 命令中心服务。
    """
    
    def __init__(self, command_hub_service: CommandHubService, parent=None):
        """初始化末端执行器视图模型。
        
        Args:
            command_hub_service (CommandHubService): 命令中心服务。
            parent (QObject, optional): 父对象. Defaults to None.
        """
        super().__init__(parent)
        self.command_hub_service = command_hub_service
    
    def send_effector_command(self, config_dict: dict):
        """发送末端执行器命令。
        
        Args:
            config_dict (dict): 命令配置字典。
        """
        self.command_hub_service.command_distribution(config_dict)

