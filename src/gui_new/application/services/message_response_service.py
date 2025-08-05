from domain import MessageDomainService
from domain.services import SerialDomainService
from .base_service import BaseService
from ..commands import MessageDisplay


class MessageResponseService(BaseService):
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
                except Exception as e:
                    self._display_message(f"解码消息失败: {str(e)}", "错误")
                    return
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
                self._display_message(f"初始状态: {init_status}, 当前命令: {control}, 运行模式: {mode}, 位置数据: {positions}, 状态字: {status}, 实际速度: {speeds}, 力矩: {torques}, 双编码器插值: {double_encoder_interpolations}, 错误码:{errors}, 夹爪数据: {effector_data}", "接收")
            else:
                self._display_message(command_line, "接收")
