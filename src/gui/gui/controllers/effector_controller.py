class EffectorController:
    """执行器处理类，处理夹爪等执行器相关功能"""
    
    def __init__(self, send_data_callback, display_callback):
        self.display_callback = display_callback
        self.send_data_callback = send_data_callback
    
    def send_effector_command(self, encoding_type, effector_params):        
        command, value = effector_params
        # 发送控制命令
        success, cmd_str = self.send_data_callback(
            effector_mode=command,
            effector_data=value,
            encoding=encoding_type,
            return_cmd=True
        )
        
        if success:
            self.display_callback(f"{cmd_str}", "发送")
        else:
            self.display_callback("发送执行器命令失败", "错误")
