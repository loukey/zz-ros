from .base_controller import BaseController
from PyQt5.QtCore import pyqtSlot


class EffectorController(BaseController):    
    def __init__(self, serial_model):
        super().__init__()
        self.serial_model = serial_model
    
    @pyqtSlot(dict)
    def handle_effector_command_requested(self, params):
        self.send_effector_command(**params)

    def send_effector_command(self, encoding_type, effector_params):        
        command, value = effector_params
        # 发送控制命令
        success, cmd = self.serial_model.send_control_command(
            effector_mode=command,
            effector_data=value,
            encoding=encoding_type,
            return_cmd=True
        )
        
        if success:
            self.display(f"{cmd}", "发送")
        else:
            self.display("发送执行器命令失败", "错误")
