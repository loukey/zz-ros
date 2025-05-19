"""
虚拟串口模拟器包 - 专注于文本命令的处理
"""
from .virtual_serial import VirtualSerial
from .simple_simulator import TextCommandSimulator

__all__ = ['VirtualSerial', 'TextCommandSimulator'] 