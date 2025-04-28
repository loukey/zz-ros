"""
执行器处理模块
"""
from PyQt5.QtWidgets import QMessageBox

class EffectorHandler:
    """执行器处理类，处理夹爪等执行器相关功能"""
    
    def __init__(self, main_window):
        self.main_window = main_window
    
    def handle_send_clicked(self):
        """处理发送按钮点击事件"""
        encoding_type = self.main_window.control_buttons.get_encoding_type()   
        if not self.main_window.serial_handler.is_connected():
            QMessageBox.warning(self.main_window, "警告", "请先连接串口")
            return
        
        # 获取执行器参数
        params = self.main_window.effector_settings.get_effector_params()
        if params is None:
            QMessageBox.warning(self.main_window, "警告", "参数获取失败")
            return
        
        command, value = params
        # 发送控制命令
        success, cmd_str = self.main_window.serial_handler.serial_controller.send_control_command(
            effector_mode=command,
            effector_data=value,
            encoding=encoding_type,
            return_cmd=True
        )
        
        if success:
            self.main_window.data_display.append_message(f"{cmd_str}", "发送")
        else:
            self.main_window.data_display.append_message("发送执行器命令失败", "错误")
