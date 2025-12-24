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
        # 获取配置参数
        intrinsics = self.config.camera_intrinsics
        
        # 1. 像素坐标 → 相机坐标系（齐次坐标）
        px1, py1, pz1 = central_center[0], central_center[1], depth
        px2, py2, pz2 = real_center[0], real_center[1], real_depth
        
        # 深度单位转换：mm → m
        pz1 *= 0.001
        pz2 *= 0.001
        
        # 使用针孔相机模型
        px1 = (px1 - intrinsics.cx) * pz1 / intrinsics.fx
        py1 = (py1 - intrinsics.cy) * pz1 / intrinsics.fy
        px2 = (px2 - intrinsics.cx) * pz2 / intrinsics.fx
        py2 = (py2 - intrinsics.cy) * pz2 / intrinsics.fy
        
        p1_cam = np.array([px1, py1, pz1, 1])
        p2_cam = np.array([px2, py2, pz2, 1])
        
        # 2. 正运动学：获取当前末端位姿（4x4变换矩阵）
        forward_matrix = self.kinematic_service.get_gripper2base_rm(current_joint_angles)
        
        # 3. 相机坐标系 → 基坐标系
        T_cam2base = forward_matrix @ self.config.hand_eye_matrix
        print(T_cam2base)
        p1_base = T_cam2base @ p1_cam
        p2_base = T_cam2base @ p2_cam
        v_base = p2_base - p1_base
        print(p1_base)
        # 4. 计算目标位姿
        # 4.1 计算零件方向向量
        theta = np.arctan2(v_base[1], v_base[0])
        print(theta)  
        # 4.2 构建绕 Z 轴旋转的变换矩阵
        length = 0
        T_target2base = self.get_z_rotation_matrix(theta)
        T_target2base[:3, 3] = p1_base[:3]+length * T_target2base[:3, 0]

        
        # 4.3 调整末端姿态
        adjustment = self.config.end_effector_adjustment
        T_target2base = (
            T_target2base 
            @ self.get_z_rotation_matrix(adjustment.z_rotation) 
            @ self.get_y_rotation_matrix(adjustment.y_rotation)
            @ self.get_x_rotation_matrix(adjustment.x_rotation)
        )
        
         # 4.4 添加偏移量
        
        T_offset2target = np.eye(4)
        T_offset2target[:3, 3] = [0, 0, 0]
        T_target2base = T_target2base @ T_offset2target

        T_offset2target[:3, 3] = [0, 0, 0.09]
        T_target2base = T_offset2target @ T_target2base

        # 5. 逆运动学求解
        theta_list = self.kinematic_service.inverse_kinematic(
            T_target2base[:3, :3], 
            T_target2base[:3, 3],
            initial_theta=[1.21489, -0.68856, -1.26255, 2.56762, 0.44942, -1.5708]
        )
        
        return theta_list
    
    def get_z_rotation_matrix(self, angle: float) -> np.ndarray:
        """绕 Z 轴旋转的齐次变换矩阵"""
        c = cos(angle)
        s = sin(angle)
        return np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ])
    
    def get_y_rotation_matrix(self, angle: float) -> np.ndarray:
        """绕 Y 轴旋转的齐次变换矩阵"""
        c = cos(angle)
        s = sin(angle)
        return np.array([
            [ c, 0, s, 0],
            [ 0, 1, 0, 0],
            [-s, 0, c, 0],
            [ 0, 0, 0, 1]
        ])
    
    def get_x_rotation_matrix(self, angle: float) -> np.ndarray:
        """绕 X 轴旋转的齐次变换矩阵"""
        c = cos(angle)
        s = sin(angle)
        return np.array([
            [1,  0,  0, 0],
            [0,  c, -s, 0],
            [0,  s,  c, 0],
            [0,  0,  0, 1]
        ])

