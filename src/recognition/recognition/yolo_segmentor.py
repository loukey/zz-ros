import numpy as np
import math
from typing import Tuple, Optional, Dict
from ultralytics import YOLO


class YOLOSegmentor:

    def __init__(self, model_path):
        self.model = YOLO(model_path)

    def _calculate_obb_center(self, coords: np.ndarray) -> Tuple[float, float]:
        """计算OBB中心点"""
        corners = coords.reshape(4, 2)
        center_x = corners[:, 0].mean()
        center_y = corners[:, 1].mean()
        return (float(center_x), float(center_y))

    def _calculate_obb_angle(self, coords: np.ndarray) -> float:
        """计算OBB角度 - 直着向上为0度，顺时针为正值"""
        points = coords.reshape(4, 2)
        
        # 计算长边的方向向量
        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]
        
        # 选择较长的边作为主方向
        if np.linalg.norm(edge1) > np.linalg.norm(edge2):
            main_edge = edge1
        else:
            main_edge = edge2
        
        # 计算角度 - 直着向上为0度，顺时针为正值
        angle_rad = math.atan2(main_edge[0], -main_edge[1])
        
        # 标准化到[-180, 180]度
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        elif angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        
        return angle_rad

    def _extract_best_detection(self, obb, target_class_id: int) -> Optional[Dict]:
        """提取指定类别的最佳检测结果"""
        if obb is None or len(obb) == 0:
            return None
        
        # 找到目标类别的检测结果
        class_indices = []
        for i in range(len(obb)):
            if int(obb.cls[i].cpu().numpy()) == target_class_id:
                class_indices.append(i)
        
        if not class_indices:
            return None
        
        # 选择置信度最高的
        best_idx = max(class_indices, key=lambda i: float(obb.conf[i].cpu().numpy()))
        
        coords = obb.xyxyxyxy[best_idx].cpu().numpy()
        confidence = float(obb.conf[best_idx].cpu().numpy())
        center = self._calculate_obb_center(coords)
        angle = self._calculate_obb_angle(coords)
        
        return {
            'center': center,
            'confidence': confidence,
            'angle': angle
        }

    def detect(self, image):
        results = self.model(image)
        
        if not results or results[0].obb is None:
            return {}
        
        obb = results[0].obb
        
        # 提取central和head的检测结果 (假设central=0, head=1)
        central_info = self._extract_best_detection(obb, 0)
        head_info = self._extract_best_detection(obb, 1)
        
        if central_info and head_info:
            return {
                "head_center": head_info["center"],
                "central_center": central_info["center"],
                "angle": central_info["angle"]  # 使用central的角度
            }
        
        return {}
