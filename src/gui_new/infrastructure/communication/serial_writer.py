"""
串口写入器 - Infrastructure层
负责多线程发送串口数据
"""
import serial
import time
from queue import Queue
from PyQt5.QtCore import QObject
from typing import Union, Optional


class SerialWriter(QObject):
    """串口发送处理类
    
    负责在独立线程中发送串口数据，使用队列管理发送任务
    """
    
    def __init__(self, serial_port: Optional[serial.Serial] = None):
        """
        初始化串口写入器
        
        参数:
            serial_port: 串口对象
        """
        super().__init__()
        self.serial_port = serial_port
        self.send_queue = Queue()
        self.stop_flag = False
    
    def stop(self) -> None:
        """停止发送"""
        self.stop_flag = True
    
    def add_to_queue(self, cmd: Union[str, bytes]) -> None:
        """添加命令到发送队列"""
        self.send_queue.put(cmd)
    
    def send_data(self) -> None:
        """发送串口数据 - 在独立线程中运行"""
        self.stop_flag = False
        
        while not self.stop_flag:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    time.sleep(0.1)
                    continue
                    
                if not self.send_queue.empty():
                    try:
                        cmd = self.send_queue.get()
                        if isinstance(cmd, str):
                            cmd = bytes.fromhex(cmd.replace(' ', ''))
                        self.serial_port.write(cmd)
                        self.serial_port.flush()
                        self.send_queue.task_done()
                    except Exception:
                        # 静默处理错误，不发送信号
                        pass
                else:
                    # 队列为空时短暂休眠，避免CPU空转
                    time.sleep(0.01)
            except Exception:
                # 静默处理错误，不发送信号
                time.sleep(0.1) 