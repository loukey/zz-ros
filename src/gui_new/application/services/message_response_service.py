from domain import MessageDomainService
from domain.services import SerialDomainService
from .base_service import BaseService
from ..commands import MessageDisplay
from PyQt5.QtCore import pyqtSignal


class MessageResponseService(BaseService):
    # 解码消息信号 - 发送到StatusViewModel
    decoded_message_received = pyqtSignal(object)
    
    def __init__(
        self, 
        message_display: MessageDisplay,
        serial_domain_service: SerialDomainService,
        message_domain_service: MessageDomainService):
        super().__init__(message_display)
        self.serial_domain_service = serial_domain_service
        self.message_domain_service = message_domain_service
        self._connect_signals()
        self.message_buffer = ""
        """
        single: 只获取一次当前位置并返回
        motion: 获取当前位置，并进行一次运动
        multi_motion: 包含运动+夹爪
        teach_motion: 示教模式运动
        """
        self.motion_status = "single"

    def _connect_signals(self):
        """连接串口domain层的数据接收信号"""
        self.serial_domain_service.data_received.connect(self.handle_message)

    def handle_message(self, message_in: str):
        self.message_buffer += message_in
        if "0D0A" in self.message_buffer:
            lines = self.message_buffer.split("0D0A")
            self.message_buffer = lines[-1]
            command_line = lines[0]
            if command_line.startswith("AA55"):
                try:
                    decoded_message = self.message_domain_service.decode_message(command_line + "0D0A")           
                    # 发送解码消息到StatusViewModel
                    self.decoded_message_received.emit(decoded_message)

                except Exception as e:
                    self._display_message(f"解码消息失败: {str(e)}", "错误")
                    return
            else:
                self._display_message(command_line, "接收")
