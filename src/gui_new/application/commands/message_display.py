from PyQt5.QtCore import QObject, pyqtSignal


class MessageDisplay(QObject):
    message_display_signal = pyqtSignal(str, str)
    clear_requested = pyqtSignal()

    def __init__(self):
        super().__init__()

    def display_message(self, message: str, message_type: str) -> None:
        self.message_display_signal.emit(message, message_type)

    def clear_messages(self) -> None:
        self.clear_requested.emit()
        