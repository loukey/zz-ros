"""
串口读取器 - Infrastructure层
负责多线程读取串口数据
"""
import serial
import time
from PyQt5.QtCore import QObject, pyqtSignal
from typing import Optional


class SerialReader(QObject):
    """串口读取处理类
    
    负责在独立线程中读取串口数据，支持多种编码格式
    """
    
    # 信号定义
    data_received = pyqtSignal(str)  # 接收到数据时发送信号
    
    def __init__(self, serial_port: Optional[serial.Serial] = None):
        """
        初始化串口读取器
        
        参数:
            serial_port: 串口对象
        """
        super().__init__()
        self.serial_port = serial_port
        self.stop_flag = False
    

    
    def stop(self) -> None:
        """停止读取"""
        self.stop_flag = True
    
    def read_data(self) -> None:
        """读取串口数据 - 在独立线程中运行"""
        self.stop_flag = False
        
        while not self.stop_flag:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.1)
                continue
                
            try:
                waiting_bytes = self.serial_port.in_waiting
            except OSError:
                time.sleep(0.1)
                continue
            
            if waiting_bytes > 0:
                try:
                    data = self.serial_port.read(waiting_bytes)
                    
                    # 固定使用十六进制格式
                    hex_data = data.hex().upper()
                    self.data_received.emit(hex_data)
                            
                except serial.SerialException:
                    time.sleep(0.1)
                    continue
            else:
                time.sleep(0.01)
            
            time.sleep(0.01) 