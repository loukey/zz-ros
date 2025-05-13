from PyQt5.QtCore import QObject, pyqtSignal


class BaseController(QObject):
    display_requested = pyqtSignal(str, str)
    

    def __init__(self):
        super().__init__()

    def display(self, message, message_type):
        self.display_requested.emit(message, message_type)
