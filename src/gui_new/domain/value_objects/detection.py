"""
检测相关值对象
"""
from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np


@dataclass(frozen=True)
class Point2D:
    """2D点"""
    x: float
    y: float
    
    def to_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)


@dataclass(frozen=True)
class DetectionResult:
    """检测结果值对象"""
    head_center: Point2D        # 头部中心点
    central_center: Point2D     # 中心点  
    real_center: Point2D        # 真实中心点
    angle: float                # 角度（弧度）
    depth: float                # 深度
    real_depth: float           # 真实深度
    confidence: float = 1.0     # 置信度
    
    def __post_init__(self):
        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError("置信度必须在0-1之间")


@dataclass(frozen=True) 
class CameraSettings:
    """相机设置值对象"""
    width: int = 640
    height: int = 480
    fps: int = 30
    exposure: Optional[float] = None
    gain: Optional[float] = None
    
    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("图像尺寸必须大于0")
        if self.fps <= 0:
            raise ValueError("帧率必须大于0")


@dataclass(frozen=True)
class CameraFrame:
    """相机帧数据"""
    image_data: np.ndarray      # 图像数据
    timestamp: float            # 时间戳
    frame_type: str             # 帧类型: 'color', 'depth'
    
    def __post_init__(self):
        if self.image_data.size == 0:
            raise ValueError("图像数据不能为空")
        if self.frame_type not in ['color', 'depth']:
            raise ValueError("帧类型必须为'color'或'depth'") 