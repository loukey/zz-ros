"""
运动操作模式 - Domain Value Object
"""
from enum import Enum


class MotionOperationMode(Enum):
    """运动操作模式枚举。
    
    Attributes:
        NONE: 无操作（仅获取位置）。
        EXECUTE: 执行运动。
        SAVE: 保存轨迹。
        PREVIEW: 预览轨迹（显示曲线）。
    """
    
    NONE = 0      # 无操作（仅获取位置）
    EXECUTE = 1   # 执行运动
    SAVE = 2      # 保存轨迹
    PREVIEW = 3   # 预览轨迹（显示曲线）

