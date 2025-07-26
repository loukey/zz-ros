"""
命令相关的数据传输对象
"""
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class SendAnglesDTO:
    """发送角度命令DTO"""
    target_angles: List[float]  # 目标关节角度（弧度）
    curve_type: str = 'S型'  # 曲线类型
    frequency: float = 0.01  # 频率（秒）
    contour_params: Optional[List[List[float]]] = None  # 轮廓参数 [speed, accel, decel]
    encoding_type: str = 'hex'
    run_mode: int = 0x08


@dataclass
class ControlCommandDTO:
    """控制命令DTO"""
    joint_angles: List[float]  # 关节角度
    control: int = 0x06  # 控制字节
    mode: int = 0x08  # 模式字节
    contour_speed: List[float] = None  # 轮廓速度
    contour_acceleration: List[float] = None  # 轮廓加速度
    contour_deceleration: List[float] = None  # 轮廓减速度
    torque: List[float] = None  # 力矩
    effector_mode: int = 0x00  # 执行器模式
    effector_data: float = 0.0  # 执行器数据
    encoding: str = 'hex'


@dataclass
class SerialConfigDTO:
    """串口配置DTO"""
    port: str
    baud_rate: int = 115200
    data_bits: int = 8
    parity: str = 'N'  # N, E, O
    stop_bits: int = 1
    flow_control: Optional[str] = None  # None, 'xonxoff', 'rtscts', 'dsrdtr'


@dataclass
class SerialDataDTO:
    """串口数据DTO"""
    data: str
    encoding: str = 'hex'  # 'hex' or 'string'
    timestamp: Optional[str] = None 