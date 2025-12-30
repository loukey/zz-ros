from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from controller.application import CommandHubService


class ControlViewModel(BaseViewModel):
    """控制视图模型。
    
    负责处理用户输入的控制命令，并分发到命令中心服务。
    
    Attributes:
        command_hub_service (CommandHubService): 命令中心服务。
    """

    def __init__(self, command_hub_service: CommandHubService, parent=None):
        """初始化控制视图模型。
        
        Args:
            command_hub_service (CommandHubService): 命令中心服务。
            parent (QObject, optional): 父对象. Defaults to None.
        """
        super().__init__(parent)
        self.command_hub_service = command_hub_service

    def send_command(self, config_dict: dict):
        """发送控制命令。
        
        Args:
            config_dict (dict): 命令配置字典，包含控制参数。
        """
        self.command_hub_service.command_distribution(config_dict)
    