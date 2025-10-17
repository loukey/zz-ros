"""
消息响应服务 - Application层
处理串口接收的数据，解码并更新状态服务
"""
from controller.domain import MessageDomainService, SerialDomainService, RobotStateDomainService, MotionConstructor, MotionRunner
from .base_service import BaseService
from ..commands import MessageDisplay
from PyQt5.QtCore import pyqtSignal


class MessageResponseService(BaseService):
    """
    消息响应服务
    
    职责：
    1. 接收串口数据
    2. 拼接缓冲，检测完整帧
    3. 解码消息
    4. 更新状态服务（统一入口）
    5. 处理运动消息（根据 Constructor 状态决定是否自动执行）
    """
    
    # 信号：获取当前位置（用于UI显示等非运动场景）
    get_current_position_signal = pyqtSignal(object)
    
    def __init__(
        self, 
        message_display: MessageDisplay,
        serial_domain_service: SerialDomainService,
        message_domain_service: MessageDomainService,
        robot_state_service: RobotStateDomainService,
        motion_runner: MotionRunner,
        motion_constructor: MotionConstructor):
        super().__init__(message_display)
        self.serial_domain_service = serial_domain_service
        self.message_domain_service = message_domain_service
        self.robot_state_service = robot_state_service
        self.motion_runner = motion_runner
        self.motion_constructor = motion_constructor
        self._connect_signals()
        self.message_buffer = ""

    def _connect_signals(self):
        """连接串口domain层的数据接收信号"""
        self.serial_domain_service.data_received.connect(self.handle_message)

    def handle_message(self, message_in: str):
        """
        处理接收到的消息
        
        流程：
        1. 拼接缓冲
        2. 检测完整帧（AA55...0D0A）
        3. 解码消息
        4. 更新状态服务
        5. 处理运动消息
        """
        self.message_buffer += message_in
        if "0D0A" in self.message_buffer:
            lines = self.message_buffer.split("0D0A")
            self.message_buffer = lines[-1]
            command_line = lines[0]
            self._display_message(command_line, "接收")
            if command_line.startswith("AA55"):
                try:
                    # 解码消息
                    decoded_message = self.message_domain_service.decode_message(
                        command_line + "0D0A"
                    )
                    # 更新统一的状态服务（单一入口）
                    self.robot_state_service.update_state(decoded_message)
                    
                    # 处理运动消息
                    if decoded_message.control == 0x07 and decoded_message.mode == 0x08:
                        self.handle_motion_message(decoded_message.positions)

                except Exception as e:
                    
                    self._display_message(f"解码消息失败: {str(e)}", "错误")
                    return
            else:
                try:
                    ascii_text = bytes.fromhex(command_line).decode('ascii', errors='replace')
                    # 过滤不可打印字符
                    printable_text = ''.join(c if c.isprintable() or c in '\n\r\t' else f'\\x{ord(c):02x}' for c in ascii_text)
                    if printable_text.strip():  # 如果有可打印内容
                        self._display_message(f"ASCII: {printable_text}", "接收")
                except Exception:
                    pass  # 转换失败则忽略

    def handle_motion_message(self, current_position):
        """
        处理运动消息（获取当前位置的回复）
        
        根据 MotionConstructor 的状态决定行为：
        - 如果有待执行的运动：自动构建轨迹并启动运动
        - 如果没有：只发射信号供UI显示
        
        Args:
            current_position: 当前关节位置（编码器值）
        """
        if self.motion_constructor.has_pending_motion():
            self.motion_constructor.construct_motion_data(current_position)
            self.motion_runner.start_motion()
            self._display_message("轨迹构建完成，开始执行运动", "运动")
        else:
            self.get_current_position_signal.emit(current_position)
