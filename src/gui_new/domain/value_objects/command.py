"""
命令相关值对象
"""
from dataclasses import dataclass
from typing import Optional
from enum import Enum
from .pose import JointAngles
from .trajectory import ContourParameters


class ControlMode(Enum):
    """控制模式"""
    STOP = 0x00               # 停止
    EMERGENCY_STOP = 0x01     # 急停
    POSITION_CONTROL = 0x06   # 位置控制
    GET_POSITION = 0x07       # 获取位置
    RESET = 0x08              # 复位
    PAUSE = 0x05              # 暂停


class RunMode(Enum):
    """运行模式"""
    JOINT_MODE = 0x01         # 关节模式
    TEACH_MODE = 0x0A         # 示教模式
    TRAJECTORY_MODE = 0x08    # 轨迹模式


class EffectorMode(Enum):
    """末端执行器模式"""
    NO_OPERATION = 0x00       # 不进行任何操作
    MANUAL_ENABLE = 0x01      # 手动使能
    SET_POSITION = 0x02       # 设置目标位置
    SET_VELOCITY = 0x03       # 设置速度
    SET_CURRENT = 0x04        # 设置电流
    QUERY_STATUS = 0x05       # 查询抓取状态
    QUERY_POSITION = 0x06     # 查询当前位置
    QUERY_CURRENT = 0x07      # 查询电流


@dataclass(frozen=True)
class SerialCommand:
    """串口命令值对象"""
    control_mode: ControlMode
    run_mode: RunMode
    joint_angles: JointAngles
    contour_params: ContourParameters
    torque: tuple[float, ...]           # 力矩 (6个关节)
    effector_mode: EffectorMode
    effector_data: float
    
    def __post_init__(self):
        if len(self.torque) != 6:
            raise ValueError("必须提供6个关节的力矩")
    
    @classmethod
    def position_command(
        cls, 
        joint_angles: JointAngles,
        contour_params: Optional[ContourParameters] = None,
        effector_mode: EffectorMode = EffectorMode.NO_OPERATION,
        effector_data: float = 0.0
    ) -> 'SerialCommand':
        """创建位置控制命令"""
        return cls(
            control_mode=ControlMode.POSITION_CONTROL,
            run_mode=RunMode.TRAJECTORY_MODE,
            joint_angles=joint_angles,
            contour_params=contour_params or ContourParameters.zero(),
            torque=tuple([0.0] * 6),
            effector_mode=effector_mode,
            effector_data=effector_data
        )
    
    @classmethod
    def get_position_command(cls) -> 'SerialCommand':
        """创建获取位置命令"""
        return cls(
            control_mode=ControlMode.GET_POSITION,
            run_mode=RunMode.TRAJECTORY_MODE,
            joint_angles=JointAngles.zero(),
            contour_params=ContourParameters.zero(),
            torque=tuple([0.0] * 6),
            effector_mode=EffectorMode.NO_OPERATION,
            effector_data=0.0
        )
    
    @classmethod
    def stop_command(cls) -> 'SerialCommand':
        """创建停止命令"""
        return cls(
            control_mode=ControlMode.STOP,
            run_mode=RunMode.TRAJECTORY_MODE,
            joint_angles=JointAngles.zero(),
            contour_params=ContourParameters.zero(),
            torque=tuple([0.0] * 6),
            effector_mode=EffectorMode.NO_OPERATION,
            effector_data=0.0
        )


@dataclass(frozen=True)
class SerialConfig:
    """串口配置值对象"""
    port: str
    baud_rate: int = 115200
    data_bits: int = 8
    parity: str = 'N'           # N, E, O
    stop_bits: int = 1
    flow_control: Optional[str] = None  # None, 'xonxoff', 'rtscts', 'dsrdtr'
    
    def __post_init__(self):
        if self.baud_rate <= 0:
            raise ValueError("波特率必须大于0")
        if self.data_bits not in [5, 6, 7, 8]:
            raise ValueError("数据位必须为5-8")
        if self.parity not in ['N', 'E', 'O']:
            raise ValueError("校验位必须为N、E或O")
        if self.stop_bits not in [1, 1.5, 2]:
            raise ValueError("停止位必须为1、1.5或2") 