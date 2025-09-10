from ..serial_domain_service import SerialDomainService
from ..message_domain_service import MessageDomainService
from PyQt5.QtCore import QTimer
import time
from PyQt5.QtCore import pyqtSignal, QObject


class DelayCommand:
    def __init__(self, delay_s: int):
        self.delay_s = delay_s
        self.is_delay = True


class MotionRunner(QObject):
    motion_msg_signal = pyqtSignal(str, str)

    def __init__(
        self, 
        serial_domain_service: SerialDomainService, 
        message_domain_service: MessageDomainService):
        super().__init__()
        self.serial_domain_service = serial_domain_service
        self.message_domain_service = message_domain_service
        self.data_list = []
        self.data_index = 0
        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.motion_timeout)

    def add_motion_data(self, positions):
        for position in positions:
            msg_dict = {
                'control': 0x06,
                'mode': 0x08,
                'joint_angles': list(position)
            }
            message = self.message_domain_service.encode_message(**msg_dict)
            self.data_list.append(message)

    def add_gripper_data(self, effector_mode, effector_data):
        msg_dict = {
            'control': 0x00,
            'mode': 0x08,
            'effector_mode': effector_mode,
            'effector_data': effector_data
        }
        message = self.message_domain_service.encode_message(**msg_dict)
        self.data_list.append(message)
        self.data_list.append(DelayCommand(delay_s=1))

    def start_motion(self):
        self.data_index = 0
        self.timer.start()
    
    def stop_motion(self):
        self.data_index = 0
        self.timer.stop()

    def clear_data(self):
        self.data_index = 0
        self.data_list = []

    def motion_timeout(self):
        if self.data_index >= len(self.data_list):
            self.timer.stop()
            return
        current_item = self.data_list[self.data_index]
        if hasattr(current_item, 'is_delay'):
            time.sleep(current_item.delay_s)
        else:
            self.serial_domain_service.send_data(current_item)
            self.motion_msg_signal.emit(current_item, "发送")
        self.data_index += 1
