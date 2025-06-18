#!/usr/bin/env python3
"""
相机标定结果使用工具
提供常用的相机几何计算和图像处理功能
"""

import cv2
import numpy as np
import yaml
import os
from typing import Tuple, Optional, List


class CameraCalibrationUtils:
    """相机标定工具类"""
    
    def __init__(self, calibration_file: str = None):
        """
        初始化相机标定工具
        
        Args:
            calibration_file: 标定文件路径 (.yaml 或 .npz)
        """
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_size = None
        
        if calibration_file and os.path.exists(calibration_file):
            self.load_calibration(calibration_file)
    
    def load_calibration(self, file_path: str) -> bool:
        """
        加载标定结果
        
        Args:
            file_path: 标定文件路径
        
        Returns:
            bool: 是否加载成功
        """
        try:
            if file_path.endswith('.yaml'):
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f)
                self.camera_matrix = np.array(data['camera_matrix'])
                self.dist_coeffs = np.array(data['distortion_coefficients'])
                self.image_size = tuple(data['image_size'])
            elif file_path.endswith('.npz'):
                data = np.load(file_path)
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
                self.image_size = tuple(data['image_size'])
            else:
                raise ValueError("不支持的文件格式")
            
            print(f"标定参数加载成功: {file_path}")
            return True
        except Exception as e:
            print(f"加载标定参数失败: {str(e)}")
            return False
    
    def set_calibration_manual(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, image_size: Tuple[int, int]):
        """
        手动设置标定参数
        
        Args:
            camera_matrix: 相机内参矩阵 (3x3)
            dist_coeffs: 畸变系数 (1x5)
            image_size: 图像尺寸 (width, height)
        """
        self.camera_matrix = camera_matrix.copy()
        self.dist_coeffs = dist_coeffs.copy()
        self.image_size = image_size
        print("标定参数手动设置完成")
    
    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        """
        图像去畸变
        
        Args:
            image: 输入图像
        
        Returns:
            去畸变后的图像
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError("请先加载标定参数")
        
        return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
    
    def undistort_points(self, points: np.ndarray) -> np.ndarray:
        """
        点去畸变
        
        Args:
            points: 畸变的像素点 (N x 2)
        
        Returns:
            去畸变后的像素点 (N x 2)
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError("请先加载标定参数")
        
        # 转换为正确格式
        points_reshaped = points.reshape(-1, 1, 2).astype(np.float32)
        undistorted = cv2.undistortPoints(points_reshaped, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
        return undistorted.reshape(-1, 2)
    
    def pixel_to_camera_coords(self, pixel_points: np.ndarray, depth: float) -> np.ndarray:
        """
        像素坐标转相机坐标
        
        Args:
            pixel_points: 像素坐标 (N x 2)
            depth: 深度值 (米)
        
        Returns:
            相机坐标系中的3D点 (N x 3)
        """
        if self.camera_matrix is None:
            raise ValueError("请先加载标定参数")
        
        # 获取相机内参
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # 转换到相机坐标系
        camera_points = []
        for point in pixel_points:
            x_pixel, y_pixel = point
            x_camera = (x_pixel - cx) * depth / fx
            y_camera = (y_pixel - cy) * depth / fy
            z_camera = depth
            camera_points.append([x_camera, y_camera, z_camera])
        
        return np.array(camera_points)
    
    def camera_to_pixel_coords(self, camera_points: np.ndarray) -> np.ndarray:
        """
        相机坐标转像素坐标
        
        Args:
            camera_points: 相机坐标系中的3D点 (N x 3)
        
        Returns:
            像素坐标 (N x 2)
        """
        if self.camera_matrix is None:
            raise ValueError("请先加载标定参数")
        
        # 投影到像素平面
        pixel_points, _ = cv2.projectPoints(
            camera_points.reshape(-1, 1, 3),
            np.zeros(3),  # 无旋转
            np.zeros(3),  # 无平移
            self.camera_matrix,
            self.dist_coeffs
        )
        
        return pixel_points.reshape(-1, 2)
    
    def get_field_of_view(self) -> Tuple[float, float]:
        """
        计算相机视场角
        
        Returns:
            (水平视场角, 垂直视场角) 单位：度
        """
        if self.camera_matrix is None or self.image_size is None:
            raise ValueError("请先加载标定参数")
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        width, height = self.image_size
        
        # 计算视场角
        fov_x = 2 * np.arctan(width / (2 * fx)) * 180 / np.pi
        fov_y = 2 * np.arctan(height / (2 * fy)) * 180 / np.pi
        
        return fov_x, fov_y
    
    def get_optimal_new_camera_matrix(self, alpha: float = 0.0) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
        """
        获取优化的新相机矩阵
        
        Args:
            alpha: 自由缩放参数 (0-1)
                  0: 去畸变后保留所有有效像素
                  1: 保留所有原始像素
        
        Returns:
            (新相机矩阵, 有效区域ROI)
        """
        if self.camera_matrix is None or self.dist_coeffs is None or self.image_size is None:
            raise ValueError("请先加载标定参数")
        
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, 
            self.dist_coeffs, 
            self.image_size, 
            alpha
        )
        
        return new_camera_matrix, roi
    
    def create_undistort_maps(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        创建去畸变映射表（用于快速去畸变）
        
        Returns:
            (map1, map2): 去畸变映射表
        """
        if self.camera_matrix is None or self.dist_coeffs is None or self.image_size is None:
            raise ValueError("请先加载标定参数")
        
        map1, map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            self.camera_matrix,
            self.image_size,
            cv2.CV_32FC1
        )
        
        return map1, map2
    
    def fast_undistort(self, image: np.ndarray, map1: np.ndarray, map2: np.ndarray) -> np.ndarray:
        """
        使用预计算的映射表快速去畸变
        
        Args:
            image: 输入图像
            map1, map2: 预计算的映射表
        
        Returns:
            去畸变后的图像
        """
        return cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
    
    def print_calibration_info(self):
        """打印标定信息"""
        if self.camera_matrix is None:
            print("未加载标定参数")
            return
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        print("="*50)
        print("相机标定参数信息")
        print("="*50)
        print(f"图像尺寸: {self.image_size[0]} x {self.image_size[1]}")
        print(f"焦距 (fx, fy): ({fx:.2f}, {fy:.2f})")
        print(f"主点 (cx, cy): ({cx:.2f}, {cy:.2f})")
        
        if self.image_size:
            fov_x, fov_y = self.get_field_of_view()
            print(f"视场角: {fov_x:.1f}° x {fov_y:.1f}°")
        
        print("\n相机内参矩阵:")
        print(self.camera_matrix)
        print("\n畸变系数:")
        print(self.dist_coeffs)
        print("="*50)


# 使用你的标定结果创建实例
def create_your_camera_utils():
    """使用你的标定结果创建相机工具实例"""
    # 你的标定结果
    camera_matrix = np.array([
        [704.82336131,   0.0,         647.58412328],
        [0.0,           702.96627857, 350.0264086 ],
        [0.0,           0.0,          1.0         ]
    ])
    
    dist_coeffs = np.array([0.025503, -0.082460, -0.004472, 0.002720, 0.052633])
    image_size = (1280, 720)
    
    # 创建工具实例
    camera_utils = CameraCalibrationUtils()
    camera_utils.set_calibration_manual(camera_matrix, dist_coeffs, image_size)
    
    return camera_utils


if __name__ == '__main__':
    # 示例使用
    print("相机标定工具示例")
    
    # 创建工具实例
    utils = create_your_camera_utils()
    
    # 打印标定信息
    utils.print_calibration_info()
    
    # 示例：计算像素坐标对应的相机坐标
    pixel_point = np.array([[640, 360]])  # 图像中心点
    depth = 1.0  # 1米深度
    camera_point = utils.pixel_to_camera_coords(pixel_point, depth)
    print(f"\n像素点 {pixel_point[0]} 在深度 {depth}m 处的相机坐标: {camera_point[0]}")
    
    # 示例：计算视场角
    fov_x, fov_y = utils.get_field_of_view()
    print(f"相机视场角: {fov_x:.1f}° x {fov_y:.1f}°") 
    