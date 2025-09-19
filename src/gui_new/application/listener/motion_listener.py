from domain import MotionRunner
from ..commands import MessageDisplay


class MotionListener():

    def __init__(self, motion_runner: MotionRunner, message_display: MessageDisplay):
        self.motion_runner = motion_runner
        self.message_display = message_display
        self._connect_signals()

    def _connect_signals(self):
        self.motion_runner.motion_msg_signal.connect(self.handle_motion_msg)

    def handle_motion_msg(self, message, message_type):
        self.message_display.display_message(str(message), message_type)

