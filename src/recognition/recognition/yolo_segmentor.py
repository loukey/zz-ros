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

    def _get_adjusted_main_edge(self, coords: np.ndarray, central_center: Tuple[float, float], head_center: Tuple[float, float]) -> np.ndarray:
        """计算调整后的主方向向量（长边指向head方向）"""
        points = coords.reshape(4, 2)
        
        # 计算长边的方向向量
        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]
        
        # 选择较长的边作为主方向
        if np.linalg.norm(edge1) > np.linalg.norm(edge2):
            main_edge = edge1
        else:
            main_edge = edge2
        
        # 计算从central中心到head中心的方向向量
        central_to_head = np.array([head_center[0] - central_center[0], 
                                   head_center[1] - central_center[1]])
        
        # 计算长边方向与central到head方向的点积
        dot_product = np.dot(main_edge, central_to_head)
        
        # 如果点积为负，说明长边方向与head方向相反，需要取反
        if dot_product < 0:
            main_edge = -main_edge
        
        return main_edge

    def _calculate_obb_angle(self, coords: np.ndarray, central_center: Tuple[float, float], head_center: Tuple[float, float]) -> float:
        """计算OBB角度 - 长边指向head方向与垂直向下方向的夹角，左侧为正，右侧为负"""
        main_edge = self._get_adjusted_main_edge(coords, central_center, head_center)
        
        # 计算角度 - 与垂直向下方向的夹角，左侧为正，右侧为负
        # 垂直向下的向量是 [0, 1]（图像坐标系中y向下为正）
        # 使用 -atan2(main_edge[0], main_edge[1]) 来实现左侧为正，右侧为负
        angle_rad = -math.atan2(main_edge[0], main_edge[1])
        
        # 标准化到[-180, 180]度
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        elif angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        
        return angle_rad

    def _extract_best_detection(self, obb, target_class_id: int, head_center: Optional[Tuple[float, float]] = None) -> Optional[Dict]:
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
        
        # 如果是central类别且有head_center，则计算角度和real_center
        if target_class_id == 0 and head_center is not None:
            angle = self._calculate_obb_angle(coords, center, head_center)
            
            # 计算real_center：使用调整后的主方向向量
            main_edge = self._get_adjusted_main_edge(coords, center, head_center)
            edge_length = np.linalg.norm(main_edge)
            
            # 计算单位方向向量
            if edge_length > 0:
                unit_direction = main_edge / edge_length
                # 计算real_center：从central_center沿正方向前进长边长度的1/5
                real_center = (
                    float(center[0] + unit_direction[0] * edge_length / 5),
                    float(center[1] + unit_direction[1] * edge_length / 5)
                )
            else:
                real_center = center
        else:
            angle = 0.0  # 默认角度
            real_center = center  # 默认real_center等于center
        
        return {
            'center': center,
            'confidence': confidence,
            'angle': angle,
            'real_center': real_center
        }

    def detect(self, image):
        results = self.model(image)
        
        if not results or results[0].obb is None:
            return {}
        
        obb = results[0].obb
        
        # 首先提取head的检测结果
        head_info = self._extract_best_detection(obb, 1)  # head类别ID为1
        
        # 然后提取central的检测结果，传入head位置用于角度计算
        head_center = head_info["center"] if head_info else None
        central_info = self._extract_best_detection(obb, 0, head_center)  # central类别ID为0
        
        if central_info and head_info:
            return {
                "head_center": head_info["center"],
                "central_center": central_info["center"],
                "real_center": central_info["real_center"],
                "angle": central_info["angle"]  # 使用central的角度（已根据head方向调整）
            }
        
        return {}
