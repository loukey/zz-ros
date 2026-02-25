"""
串口读取器 - Infrastructure层
负责多线程读取串口数据
"""
import time
from PyQt5.QtCore import QObject, pyqtSignal
from typing import Optional
from .native_serial_port import NativeSerialPort


class SerialReader(QObject):
    """串口读取处理类。

    负责在独立线程中读取串口数据，支持多种编码格式。

    Attributes:
        data_received (pyqtSignal): 接收到数据时发送信号，携带十六进制字符串。
    """

    # 信号定义
    data_received = pyqtSignal(str)  # 接收到数据时发送信号

    def __init__(self, serial_port: Optional[NativeSerialPort] = None):
        """初始化串口读取器。

        Args:
            serial_port (NativeSerialPort, optional): 串口对象。
        """
        super().__init__()
        self.serial_port = serial_port
        self.stop_flag = False



    def stop(self) -> None:
        """停止读取。"""
        self.stop_flag = True

    def read_data(self) -> None:
        """读取串口数据 - 在独立线程中运行。"""
        self.stop_flag = False

        while not self.stop_flag:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.1)
                continue

            try:
                data = self.serial_port.read(1024, timeout_ms=100)
                if data:
                    # 固定使用十六进制格式
                    hex_data = data.hex().upper()
                    self.data_received.emit(hex_data)
            except Exception:
                time.sleep(0.1)
                continue
