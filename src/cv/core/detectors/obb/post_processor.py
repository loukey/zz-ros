from typing import List, Optional
import numpy as np
from .types import OBBDetection, RobotPoseResult
from .geometry import GeometryUtils

class RobotPoseProcessor:
    """机器人姿态后处理器"""
    
    def process(self, detections: List[OBBDetection]) -> Optional[RobotPoseResult]:
        """
        根据检测到的OBB列表，计算机器人姿态
        逻辑：寻找一个Central和一个Head，如果都存在，则计算它们的关系
        """
        central_det = None
        head_det = None
        
        # 筛选置信度最高的 Central 和 Head
        for det in detections:
            if det.class_name == 'central':
                if central_det is None or det.confidence > central_det.confidence:
                    central_det = det
            elif det.class_name == 'head':
                if head_det is None or det.confidence > head_det.confidence:
                    head_det = det
                    
        # 必须同时存在才能计算姿态
        if central_det is None or head_det is None:
            return None
            
        # 1. 计算角度
        final_angle = GeometryUtils.calculate_obb_angle(
            central_det.corners, 
            central_det.center, 
            head_det.center
        )
        
        # 2. 计算 Real Center
        real_center = GeometryUtils.calculate_real_center(
            central_det.corners,
            central_det.center,
            head_det.center
        )
        
        # 3. 计算方向向量 (用于可视化箭头)
        main_vector = GeometryUtils.get_main_direction_vector(
            central_det.corners,
            central_det.center,
            head_det.center
        )
        edge_length = np.linalg.norm(main_vector)
        if edge_length > 0:
            unit_vector = main_vector / edge_length
        else:
            unit_vector = np.array([0, 0])
            
        # 构造可视化辅助点 (箭头起点和终点)
        arrow_start = central_det.center + np.array([0, 20])
        arrow_end = arrow_start + unit_vector * 50
        
        return RobotPoseResult(
            central=central_det,
            head=head_det,
            final_angle=final_angle,
            real_center=real_center,
            direction_vector=unit_vector,
            arrow_start=arrow_start,
            arrow_end=arrow_end
        )

