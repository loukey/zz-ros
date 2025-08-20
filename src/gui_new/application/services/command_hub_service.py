from ..commands import MessageDisplay
from .base_service import BaseService
from domain import MessageDomainService, MotionRunner
from domain.services import SerialDomainService


class CommandHubService(BaseService):

    def __init__(self, 
    message_domain_service: MessageDomainService, 
    motion_runner: MotionRunner,
    serial_domain_service: SerialDomainService, 
    message_display: MessageDisplay):
        super().__init__(message_display)
        self.message_domain_service = message_domain_service
        self.motion_runner = motion_runner
        self.serial_domain_service = serial_domain_service

    def single_send_command(self, **kwargs):
        """发送单个命令"""
        try:
            msg = self.message_domain_service.encode_message(**kwargs)
            self._display_message(msg, "发送")
            success = self.serial_domain_service.send_data(msg)
            if not success:
                self._display_message("发送失败：串口未连接", "错误")
            return success
        except Exception as e:
            self._display_message(f"发送命令失败: {str(e)}", "错误")
            return False

    def command_distribution(self, config_dict: dict):
        control = config_dict.get('control')
        mode = config_dict.get('mode')
        if control in [0x01, 0x02, 0x03, 0x04]:
            # 系统指令
            self.single_send_command(**config_dict)
        elif control in [0x05, 0x08]:
            self.single_send_command(**config_dict)
            self.motion_runner.stop_motion()
        elif control == 0x07:
            self.get_current_position()
        elif control == 0x06:
            # 运动指令
            pass

    def get_current_position(self):
        self.message_display.clear_messages()
        self._display_message("正在获取当前位置...", "控制")
        self.single_send_command(control=0x07)
 