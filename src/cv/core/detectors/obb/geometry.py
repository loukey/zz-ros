import numpy as np
import math
from typing import Tuple, Optional

class GeometryUtils:
    """纯几何计算工具类"""
    
    @staticmethod
    def calculate_obb_angle(corners: np.ndarray, central_center: np.ndarray, head_center: np.ndarray) -> float:
        """
        计算OBB的旋转角度 - 长边指向head方向与垂直向下方向的夹角
        
        Args:
            corners: OBB的四个角点坐标 (4, 2)
            central_center: central目标的中心点坐标
            head_center: head目标的中心点坐标
            
        Returns:
            旋转角度(弧度) - 与垂直向下方向的夹角，左侧为正，右侧为负
        """
        # 重新排列为4x2的坐标矩阵 (如果传入的是一维数组则reshape)
        if corners.ndim == 1:
            points = corners.reshape(4, 2)
        else:
            points = corners
            
        # 计算两邻边的向量
        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]
        
        # 选择较长的边作为主方向 (假设长边是主要方向)
        if np.linalg.norm(edge1) > np.linalg.norm(edge2):
            main_edge = edge1
        else:
            main_edge = edge2
        
        # 计算从central中心到head中心的方向向量
        central_to_head = head_center - central_center
        
        # 计算长边方向与central到head方向的点积
        dot_product = np.dot(main_edge, central_to_head)
        
        # 如果点积为负，说明长边方向与head方向相反，需要取反
        if dot_product < 0:
            main_edge = -main_edge
        
        # 计算角度 - 与垂直向下方向的夹角，左侧为正，右侧为负
        # 垂直向下的向量是 [0, 1]（图像坐标系中y向下为正）
        # 使用 -atan2(main_edge[0], main_edge[1]) 来实现左侧为正，右侧为负
        # atan2(y, x) 计算的是与x轴的夹角，这里我们想要的是与y轴(垂直向下)的夹角
        angle_rad = -math.atan2(main_edge[0], main_edge[1])
        
        # 标准化到[-pi, pi]
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        elif angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        
        return angle_rad
    
    @staticmethod
    def calculate_real_center(corners: np.ndarray, central_center: np.ndarray, head_center: np.ndarray) -> np.ndarray:
        """
        计算real_center点：从central_center出发，沿着正方向前进长边长度的1/5距离的点
        
        Args:
            corners: 四个角点坐标
            central_center: central目标的中心点坐标
            head_center: head目标的中心点坐标
            
        Returns:
            real_center点的坐标
        """
        if corners.ndim == 1:
            points = corners.reshape(4, 2)
        else:
            points = corners
            
        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]
        
        if np.linalg.norm(edge1) > np.linalg.norm(edge2):
            main_edge = edge1
        else:
            main_edge = edge2
            
        central_to_head = head_center - central_center
        
        if np.dot(main_edge, central_to_head) < 0:
            main_edge = -main_edge
            
        edge_length = np.linalg.norm(main_edge)
        
        if edge_length > 0:
            unit_direction = main_edge / edge_length
            # 计算real_center：从central_center沿正方向前进长边长度的1/5
            real_center = central_center + unit_direction * (edge_length / 5.0)
        else:
            real_center = central_center
            
        return real_center

    @staticmethod
    def get_main_direction_vector(corners: np.ndarray, central_center: np.ndarray, head_center: np.ndarray) -> np.ndarray:
        """获取主方向向量"""
        if corners.ndim == 1:
            points = corners.reshape(4, 2)
        else:
            points = corners
            
        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]
        
        if np.linalg.norm(edge1) > np.linalg.norm(edge2):
            main_edge = edge1
        else:
            main_edge = edge2
            
        central_to_head = head_center - central_center
        
        if np.dot(main_edge, central_to_head) < 0:
            main_edge = -main_edge
            
        return main_edge

