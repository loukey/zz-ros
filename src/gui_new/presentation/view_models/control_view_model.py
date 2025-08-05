from PyQt5.QtCore import pyqtSignal
from typing import List, Dict, Any, Optional
from .base_view_model import BaseViewModel
from application import CommandHubService


class ControlViewModel(BaseViewModel):
    connection_status_changed = pyqtSignal(bool)

    def __init__(self, command_hub_service: CommandHubService, parent=None):
        super().__init__(parent)
        self.command_hub_service = command_hub_service

    def send_command(self, config_dict: dict):
        self.command_hub_service.command_distribution(config_dict)
    