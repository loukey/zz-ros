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

    def _connect_signals(self):
        """连接串口domain层的数据接收信号"""
        self.serial_domain_service.data_received.connect(self.handle_message)

    def handle_message(self, message_in: str):
        self.message_buffer += message_in
        if "0D0A" in self.message_buffer:
            lines = self.message_buffer.split("0D0A")
            self.message_buffer = lines[-1]
            command_line = lines[0] + "0D0A"
            if command_line.startswith("AA55"):
                try:
                    decoded_message = self.message_domain_service.decode_message(command_line)
                    
                    # 发送解码消息到StatusViewModel
                    self.decoded_message_received.emit(decoded_message)
                    
                    # 提取各个字段用于显示
                    init_status = decoded_message.init_status
                    control = decoded_message.control
                    mode = decoded_message.mode
                    positions = decoded_message.positions
                    status = decoded_message.status
                    speeds = decoded_message.speeds
                    torques = decoded_message.torques
                    double_encoder_interpolations = decoded_message.double_encoder_interpolations
                    errors = decoded_message.errors
                    effector_data = decoded_message.effector_data
                    
                    # 显示解码后的消息（简化版本）
                    self._display_message(f"解码成功: CMD=0x{control:02X}, MODE=0x{mode:02X}, STA=0x{status:02X}, ERR=0x{errors:02X}", "接收")
                    
                except Exception as e:
                    self._display_message(f"解码消息失败: {str(e)}", "错误")
                    return
            else:
                self._display_message(command_line, "接收")
