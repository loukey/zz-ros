from PyQt5.QtCore import QObject
from ..commands import MessageDisplay


class BaseService(QObject):
    def __init__(self, message_display: MessageDisplay):
        super().__init__()
        self.message_display = message_display

    def _display_message(self, message: str, message_type: str) -> None:
        self.message_display.display_message(message, message_type)
