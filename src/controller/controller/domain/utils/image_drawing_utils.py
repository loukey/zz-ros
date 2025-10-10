"""
图像绘制工具类 - Domain层
用于在图像上绘制检测结果
"""
import cv2
import numpy as np
import math
from typing import Dict, Tuple


class ImageDrawingUtils:
    """图像绘制工具类"""
    
    @staticmethod
    def draw_detection_result(image: np.ndarray, detection: Dict) -> np.ndarray:
        """
        在图像上绘制检测结果
        
        Args:
            image: 原始图像（BGR）
            detection: 检测结果字典，包含：
                - head_center: (x, y) 头部中心点
                - central_center: (x, y) 中央中心点
                - real_center: (x, y) 实际中心点
                - angle: float 角度（弧度）
                - depth: float central_center的深度（米）
                - real_depth: float real_center的深度（米）
            
        Returns:
            绘制后的图像（新图像，不修改原图）
        """
        if not detection:
            return image
        
        # 复制图像，避免修改原图
        result_image = image.copy()
        
        try:
            # 提取检测数据
            head_center = detection.get('head_center')
            central_center = detection.get('central_center')
            real_center = detection.get('real_center')
            angle = detection.get('angle', 0.0)
            depth = detection.get('depth', 0.0)
            real_depth = detection.get('real_depth', 0.0)
            
            # 转换为整数坐标
            if head_center:
                head_center = (int(head_center[0]), int(head_center[1]))
                # 绘制head_center（红色圆点）
                cv2.circle(result_image, head_center, 5, (0, 0, 255), -1)
                # 绘制angle文本
                cv2.putText(
                    result_image, 
                    f"angle: {angle:.2f}", 
                    (head_center[0], head_center[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (0, 0, 255), 
                    2
                )
            
            if central_center:
                central_center = (int(central_center[0]), int(central_center[1]))
                # 绘制central_center（绿色圆点）
                cv2.circle(result_image, central_center, 5, (0, 255, 0), -1)
                # 绘制depth文本
                cv2.putText(
                    result_image, 
                    f"depth: {depth:.2f}", 
                    (central_center[0], central_center[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (0, 255, 0), 
                    2
                )
                # 绘制方向箭头
                ImageDrawingUtils.draw_direction_arrow(result_image, central_center, angle)
            
            if real_center:
                real_center = (int(real_center[0]), int(real_center[1]))
                # 绘制real_center（红色圆点）
                cv2.circle(result_image, real_center, 5, (0, 0, 255), -1)
                # 绘制real_depth文本
                cv2.putText(
                    result_image, 
                    f"real_depth: {real_depth:.2f}", 
                    (real_center[0], real_center[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (0, 0, 255), 
                    2
                )
            
        except Exception as e:
            # 如果绘制失败，返回原图
            return image
        
        return result_image
    
    @staticmethod
    def draw_direction_arrow(image: np.ndarray, center: Tuple[int, int], angle: float):
        """
        绘制方向箭头
        
        Args:
            image: 图像（原地修改）
            center: 中心点坐标 (x, y)
            angle: 角度（弧度）- 直着向上为0，顺时针为正
        """
        try:
            # 箭头参数
            arrow_length = 50  # 箭头长度（像素）
            arrow_color = (255, 0, 0)  # 蓝色（BGR格式）
            arrow_thickness = 3
            
            # 计算箭头终点
            # 角度系统：直着向上为0，顺时针为正
            # 向上方向为 (0, -1)，因为图像坐标系Y轴向下
            end_x = center[0] + arrow_length * math.sin(angle)
            end_y = center[1] - arrow_length * math.cos(angle)
            end_point = (int(end_x), int(end_y))
            
            # 绘制箭头
            cv2.arrowedLine(
                image, 
                center, 
                end_point, 
                arrow_color, 
                arrow_thickness, 
                tipLength=0.3
            )
            
        except Exception as e:
            pass

