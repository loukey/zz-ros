"""
手眼标定变换服务 - Domain层
"""
import numpy as np
from math import atan2, sqrt, sin, cos
from typing import Tuple, List, Optional
from ...value_objects import HandEyeCalibrationConfig
from .kinematic_domain_service import KinematicDomainService


class HandEyeTransformDomainService:
    """
    手眼标定变换服务
    
    职责：
    1. 像素坐标 → 相机坐标系
    2. 相机坐标系 → 基坐标系
    3. 计算目标位姿（包含偏移和姿态调整）
    4. 提供逆运动学求解
    
    注意：这是纯算法层，不依赖任何外部资源
    """
    
    def __init__(
        self,
        config: HandEyeCalibrationConfig,
        kinematic_service: KinematicDomainService
    ):
        """
        Args:
            config: 手眼标定配置（通过DI注入）
            kinematic_service: 运动学服务
        """
        self.config = config
        self.kinematic_service = kinematic_service
    
    def calculate_target_joint_angles(
        self,
        central_center: List[float],      # [u, v] 像素坐标
        depth: float,                     # 深度 (mm)
        real_center: List[float],         # 实际中心点 [u, v]
        real_depth: float,                # 实际深度 (mm)
        angle: float,                     # 零件角度（弧度）
        current_joint_angles: List[float] # 当前关节角度（弧度）
    ) -> Optional[List[float]]:
        """
        计算运动到零件位置所需的目标关节角度
        
        完整流程：
        1. 像素坐标 → 相机坐标系
        2. 正运动学：当前关节角度 → 末端位姿
        3. 相机坐标系 → 基坐标系
        4. 计算零件方向 + 偏移量
        5. 逆运动学：目标位姿 → 目标关节角度
        
        Args:
            central_center: 中心点像素坐标 [u, v]
            depth: 中心点深度 (mm)
            real_center: 实际中心点像素坐标 [u, v]
            real_depth: 实际中心点深度 (mm)
            angle: 零件角度（弧度）
            current_joint_angles: 当前关节角度列表（弧度）
        
        Returns:
            目标关节角度列表（弧度），如果无解则返回 None
        """
        # 1. 像素坐标 → 相机坐标系
        p1_cam = self._pixel_to_camera(central_center[0], central_center[1], depth)
        p2_cam = self._pixel_to_camera(real_center[0], real_center[1], real_depth)
        
        # 2. 正运动学：获取当前末端位姿
        _, end_effector_pos = self.kinematic_service.get_gripper2base(current_joint_angles)
        forward_matrix = self.kinematic_service.gripper2base  # 4x4变换矩阵
        
        # 3. 相机坐标系 → 基坐标系
        T_cam2base = forward_matrix @ self.config.hand_eye_matrix
        p1_base = T_cam2base @ p1_cam
        p2_base = T_cam2base @ p2_cam
        
        # 4. 计算目标位姿
        target_matrix = self._calculate_target_pose(p1_base, p2_base, angle)
        
        # 5. 逆运动学求解
        target_rotation = target_matrix[:3, :3]
        target_position = target_matrix[:3, 3]
        
        target_angles = self.kinematic_service.inverse_kinematic(
            target_rotation,
            target_position,
            initial_theta=current_joint_angles
        )
        
        if not target_angles:
            return None
        
        return target_angles
    
    def _pixel_to_camera(self, u: float, v: float, depth_mm: float) -> np.ndarray:
        """
        像素坐标转换为相机坐标系（齐次坐标）
        
        Args:
            u: 像素 x 坐标
            v: 像素 y 坐标
            depth_mm: 深度（毫米）
        
        Returns:
            相机坐标系下的齐次坐标 [x, y, z, 1]
        """
        intrinsics = self.config.camera_intrinsics
        
        # 深度单位转换：mm → m
        z = depth_mm * 0.001
        
        # 使用针孔相机模型
        x = (u - intrinsics.cx) * z / intrinsics.fx
        y = (v - intrinsics.cy) * z / intrinsics.fy
        
        return np.array([x, y, z, 1.0])
    
    def _calculate_target_pose(
        self,
        p1_base: np.ndarray,
        p2_base: np.ndarray,
        angle: float
    ) -> np.ndarray:
        """
        计算目标位姿矩阵（基坐标系）
        
        Args:
            p1_base: 中心点在基坐标系的位置（齐次坐标）
            p2_base: 实际中心点在基坐标系的位置（齐次坐标）
            angle: 零件角度（弧度）
        
        Returns:
            4x4 目标位姿矩阵
        """
        # 1. 计算零件方向向量
        v_base = p2_base - p1_base
        theta = atan2(v_base[1], v_base[0])
        
        # 2. 构建绕 Z 轴旋转的变换矩阵
        T_target2base = self._get_z_rotation_matrix(theta)
        T_target2base[:3, 3] = p2_base[:3]  # 设置位置
        
        # 3. 添加偏移量
        T_offset = np.eye(4)
        T_offset[:3, 3] = self.config.target_offset.to_array()
        T_target2base = T_target2base @ T_offset
        
        # 4. 调整末端姿态
        adjustment = self.config.end_effector_adjustment
        T_target2base = (
            T_target2base 
            @ self._get_y_rotation_matrix(adjustment.y_rotation)
            @ self._get_x_rotation_matrix(adjustment.x_rotation)
        )
        
        return T_target2base
    
    @staticmethod
    def _get_z_rotation_matrix(angle: float) -> np.ndarray:
        """绕 Z 轴旋转的齐次变换矩阵"""
        c, s = cos(angle), sin(angle)
        return np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ])
    
    @staticmethod
    def _get_y_rotation_matrix(angle: float) -> np.ndarray:
        """绕 Y 轴旋转的齐次变换矩阵"""
        c, s = cos(angle), sin(angle)
        return np.array([
            [ c, 0, s, 0],
            [ 0, 1, 0, 0],
            [-s, 0, c, 0],
            [ 0, 0, 0, 1]
        ])
    
    @staticmethod
    def _get_x_rotation_matrix(angle: float) -> np.ndarray:
        """绕 X 轴旋转的齐次变换矩阵"""
        c, s = cos(angle), sin(angle)
        return np.array([
            [1,  0,  0, 0],
            [0,  c, -s, 0],
            [0,  s,  c, 0],
            [0,  0,  0, 1]
        ])

