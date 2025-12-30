from ..communication import SerialDomainService, MessageDomainService
from PyQt5.QtCore import QTimer
import time
from PyQt5.QtCore import pyqtSignal, QObject


class DelayCommand:
    def __init__(self, delay_s: float):
        self.delay_s = delay_s
        self.is_delay = True


class MotionRunner(QObject):
    """运动执行器。
    
    负责按时间步长定时发送运动和夹爪指令。
    
    Attributes:
        motion_msg_signal (pyqtSignal): 运动消息发送信号，携带 (消息内容, 类型)。
        serial_domain_service: 串口领域服务。
        message_domain_service: 消息领域服务。
        data_list (list): 待发送的数据列表。
        data_index (int): 当前发送索引。
        timer (QTimer): 定时器。
    """
    
    motion_msg_signal = pyqtSignal(str, str)

    def __init__(
        self, 
        serial_domain_service: SerialDomainService, 
        message_domain_service: MessageDomainService):
        """初始化运动执行器。"""
        super().__init__()
        self.serial_domain_service = serial_domain_service
        self.message_domain_service = message_domain_service
        self.data_list = []
        self.data_index = 0
        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.motion_timeout)

    def add_motion_data(self, positions):
        """添加运动数据点序列。
        
        Args:
            positions (list | np.ndarray): 关节角度列表序列。
        """
        for position in positions:
            msg_dict = {
                'control': 0x06,
                'mode': 0x08,
                'joint_angles': list(position)
            }
            message = self.message_domain_service.encode_message(**msg_dict)
            self.data_list.append(message)

    def add_gripper_data(self, effector_mode, effector_data, pre_delay=0.0, post_delay=1.0):
        """添加夹爪命令。
        
        Args:
            effector_mode (int): 夹爪模式。
            effector_data (float): 夹爪参数。
            pre_delay (float, optional): 前置等待时间（秒）. Defaults to 0.0.
            post_delay (float, optional): 后置等待时间（秒）. Defaults to 1.0.
        """
        # 添加前置延迟（如果>0）
        if pre_delay > 0:
            self.data_list.append(DelayCommand(delay_s=pre_delay))
        
        # 添加夹爪命令
        msg_dict = {
            'control': 0x00,
            'mode': 0x08,
            'effector_mode': effector_mode,
            'effector_data': effector_data
        }
        message = self.message_domain_service.encode_message(**msg_dict)
        self.data_list.append(message)
        
        # 添加后置延迟（如果>0）
        if post_delay > 0:
            self.data_list.append(DelayCommand(delay_s=post_delay))

    def start_motion(self):
        """开始执行运动序列。"""
        self.data_index = 0
        self.timer.start()
    
    def stop_motion(self):
        """停止执行运动。"""
        self.data_index = 0
        self.timer.stop()

    def clear_data(self):
        """清空所有待发送数据。"""
        self.data_index = 0
        self.data_list = []

    def motion_timeout(self):
        """定时器回调函数，负责单步发送或延迟等待。"""
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
