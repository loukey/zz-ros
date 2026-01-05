import cv2
import numpy as np
import math
from typing import Dict, Tuple, Optional
from .types import OBBDetection, RobotPoseResult

class OBBVisualizer:
    """OBB检测结果可视化器"""
    
    def __init__(self, class_colors: Dict[str, Tuple[int, int, int]] = None):
        self.colors = class_colors or {
            'central': (0, 255, 0),   # 绿色
            'head': (0, 0, 255),      # 红色
        }
        self.default_color = (255, 0, 0)
        
    def draw_detection(self, image: np.ndarray, detection: OBBDetection) -> np.ndarray:
        """绘制单个OBB检测框"""
        vis_img = image
        
        # 1. 绘制旋转矩形
        corners = detection.corners.astype(np.int32)
        color = self.colors.get(detection.class_name, self.default_color)
        cv2.polylines(vis_img, [corners], True, color, 2)
        
        # 2. 绘制中心点
        center_point = (int(detection.center[0]), int(detection.center[1]))
        cv2.circle(vis_img, center_point, 6, color, -1)
        cv2.circle(vis_img, center_point, 6, (255, 255, 255), 2)
        
        # 3. 绘制标签
        label = f"{detection.class_name}: {detection.confidence:.2f}"
        cv2.putText(vis_img, label, 
                   (center_point[0] + 10, center_point[1] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                   
        return vis_img

    def draw_pose(self, image: np.ndarray, pose: RobotPoseResult) -> np.ndarray:
        """绘制机器人姿态信息 (箭头, 角度, real_center)"""
        vis_img = image
        
        # 1. 绘制 real_center
        real_center_point = (int(pose.real_center[0]), int(pose.real_center[1]))
        cv2.circle(vis_img, real_center_point, 6, (255, 255, 0), -1)  # 青色
        cv2.circle(vis_img, real_center_point, 6, (255, 255, 255), 2)
        
        cv2.putText(vis_img, "real_center", 
                   (real_center_point[0] + 10, real_center_point[1] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # 2. 绘制方向箭头
        if pose.arrow_start is not None and pose.arrow_end is not None:
            start_pt = (int(pose.arrow_start[0]), int(pose.arrow_start[1]))
            end_pt = (int(pose.arrow_end[0]), int(pose.arrow_end[1]))
            
            # 黄色箭头
            cv2.arrowedLine(vis_img, start_pt, end_pt, (0, 255, 255), 3, tipLength=0.3)
            
            # 绘制垂直参考线和角度弧线
            self._draw_angle_indicator(vis_img, start_pt, pose.final_angle)
            
        return vis_img
        
    def _draw_angle_indicator(self, image: np.ndarray, start_pt: Tuple[int, int], angle_rad: float):
        """绘制角度指示器（辅助线和弧线）"""
        angle_deg = math.degrees(angle_rad)
        
        # 垂直参考线
        vertical_start = (start_pt[0], start_pt[1] - 30)
        vertical_end = (start_pt[0], start_pt[1] + 30)
        cv2.line(image, vertical_start, vertical_end, (128, 128, 128), 1)
        
        # 角度弧线
        arc_radius = 25
        start_angle = 90  # 垂直向下
        end_angle = 90 + angle_deg
        
        if end_angle < start_angle:
            start_angle, end_angle = end_angle, start_angle
            
        cv2.ellipse(image, start_pt, (arc_radius, arc_radius),
                   0, start_angle, end_angle, (255, 0, 255), 2)
                   
        # 文本
        angle_text_pos = (start_pt[0] + 30, start_pt[1] - 10)
        angle_sign = "+" if angle_deg > 0 else ""
        angle_color = (0, 255, 0) if angle_deg > 0 else (0, 0, 255) if angle_deg < 0 else (255, 255, 255)
        
        cv2.putText(image, f"{angle_sign}{angle_deg:.1f}deg", 
                   angle_text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, angle_color, 2)

