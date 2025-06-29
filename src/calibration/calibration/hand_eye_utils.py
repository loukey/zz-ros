#!/usr/bin/env python3
"""
手眼标定应用工具 - 与hand_eye_calibration.py完全兼容
提供手眼标定结果的加载、使用和验证功能
"""

import numpy as np
import yaml
import cv2
import json
import os
from typing import Tuple, Optional, List, Dict, Union
from datetime import datetime


class HandEyeUtils:
    """
    Eye-in-Hand手眼标定应用工具类
    
    坐标系说明：
    - Base: 机器人基座坐标系
    - Gripper: 机器人末端执行器坐标系  
    - Camera: 相机坐标系
    - Target: 标定板/目标物体坐标系
    
    变换关系：
    - T_gripper_to_base: 机器人正运动学输出的位姿
    - T_gripper_to_cam: 手眼标定求解的固定变换
    - T_base_to_cam = T_gripper_to_base @ T_gripper_to_cam
    """
    
    def __init__(self, hand_eye_file: str = None):
        """
        初始化手眼工具
        
        Args:
            hand_eye_file: 手眼标定文件路径
        """
        self.hand_eye_transform = None  # T_gripper_to_cam
        self.calibration_error = None
        self.calibration_type = None
        self.calibration_date = None
        self.detailed_error = None
        self.num_poses = 0
        
        if hand_eye_file and os.path.exists(hand_eye_file):
            self.load_hand_eye_calibration(hand_eye_file)
    
    def load_hand_eye_calibration(self, file_path: str) -> bool:
        """
        加载手眼标定结果（兼容hand_eye_calibration.py的输出格式）
        
        Args:
            file_path: 标定文件路径
            
        Returns:
            bool: 是否加载成功
        """
        try:
            if file_path.endswith('.yaml'):
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f)
                
                self.hand_eye_transform = np.array(data['hand_eye_transform_matrix'])
                self.calibration_error = data.get('calibration_error', 0.0)
                self.calibration_type = data.get('calibration_type', 'eye_in_hand')
                self.calibration_date = data.get('calibration_date', 'unknown')
                self.detailed_error = data.get('detailed_error', {})
                self.num_poses = data.get('num_poses', 0)
                
            elif file_path.endswith('.npz'):
                data = np.load(file_path)
                self.hand_eye_transform = data['hand_eye_transform']
                self.calibration_error = float(data['calibration_error'])
                self.calibration_type = 'eye_in_hand'
                self.calibration_date = 'unknown'
                self.detailed_error = {}
                self.num_poses = 0
                
            else:
                raise ValueError("不支持的文件格式，请使用.yaml或.npz文件")
            
            # 验证加载的变换矩阵
            if not self._is_valid_transform(self.hand_eye_transform):
                raise ValueError("加载的变换矩阵无效")
            
            print(f"手眼标定结果加载成功: {file_path}")
            print(f"标定类型: {self.calibration_type}")
            print(f"标定误差: {self.calibration_error:.6f}")
            return True
            
        except Exception as e:
            print(f"加载手眼标定结果失败: {str(e)}")
            self.hand_eye_transform = None
            return False
    
    def set_hand_eye_transform(self, transform_matrix: np.ndarray, 
                             error: float = 0.0, 
                             calibration_type: str = 'manual'):
        """
        手动设置手眼变换
        
        Args:
            transform_matrix: 4x4变换矩阵 (T_gripper_to_cam)
            error: 标定误差
            calibration_type: 标定类型
        """
        if not self._is_valid_transform(transform_matrix):
            raise ValueError("输入的变换矩阵无效")
        
        self.hand_eye_transform = transform_matrix.copy()
        self.calibration_error = error
        self.calibration_type = calibration_type
        self.calibration_date = datetime.now().isoformat()
        print("手眼变换手动设置完成")
    
    def robot_pose_to_camera_pose(self, robot_pose: np.ndarray) -> np.ndarray:
        """
        将机器人末端执行器位姿转换为相机在基座坐标系中的位姿
        
        Args:
            robot_pose: 机器人末端执行器位姿 T_gripper_to_base (4x4矩阵)
            
        Returns:
            相机在机器人基座坐标系中的位姿 T_cam_to_base (4x4矩阵)
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # T_cam_to_base = T_gripper_to_base @ T_gripper_to_cam
        camera_pose = robot_pose @ self.hand_eye_transform
        return camera_pose
    
    def camera_pose_to_robot_pose(self, camera_pose: np.ndarray) -> np.ndarray:
        """
        将相机位姿转换为机器人末端执行器位姿
        
        Args:
            camera_pose: 相机位姿 T_cam_to_base (4x4矩阵)
            
        Returns:
            机器人末端执行器位姿 T_gripper_to_base (4x4矩阵)
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # T_gripper_to_base = T_cam_to_base @ T_gripper_to_cam^(-1)
        robot_pose = camera_pose @ np.linalg.inv(self.hand_eye_transform)
        return robot_pose
    
    def transform_point_camera_to_base(self, point_camera: np.ndarray, 
                                     robot_pose: np.ndarray) -> np.ndarray:
        """
        将相机坐标系中的点转换到机器人基座坐标系
        
        Args:
            point_camera: 相机坐标系中的点 (3D或齐次坐标)
            robot_pose: 当前机器人位姿 T_gripper_to_base (4x4矩阵)
            
        Returns:
            机器人基座坐标系中的点 (3D坐标)
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 转换为齐次坐标
        if len(point_camera.shape) == 1:
            if point_camera.shape[0] == 3:
                point_camera_homo = np.append(point_camera, 1.0)
            else:
                point_camera_homo = point_camera
        else:
            # 处理多个点的情况
            if point_camera.shape[1] == 3:
                ones = np.ones((point_camera.shape[0], 1))
                point_camera_homo = np.hstack([point_camera, ones])
            else:
                point_camera_homo = point_camera
        
        # 计算相机在基座坐标系中的位姿
        camera_pose = self.robot_pose_to_camera_pose(robot_pose)
        
        # 转换点到机器人基座坐标系
        if len(point_camera_homo.shape) == 1:
            point_base = camera_pose @ point_camera_homo
            return point_base[:3]
        else:
            # 处理多个点
            point_base = (camera_pose @ point_camera_homo.T).T
            return point_base[:, :3]
    
    def transform_point_base_to_camera(self, point_base: np.ndarray, 
                                     robot_pose: np.ndarray) -> np.ndarray:
        """
        将机器人基座坐标系中的点转换到相机坐标系
        
        Args:
            point_base: 机器人基座坐标系中的点 (3D或齐次坐标)
            robot_pose: 当前机器人位姿 T_gripper_to_base (4x4矩阵)
            
        Returns:
            相机坐标系中的点 (3D坐标)
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 转换为齐次坐标
        if len(point_base.shape) == 1:
            if point_base.shape[0] == 3:
                point_base_homo = np.append(point_base, 1.0)
            else:
                point_base_homo = point_base
        else:
            # 处理多个点的情况
            if point_base.shape[1] == 3:
                ones = np.ones((point_base.shape[0], 1))
                point_base_homo = np.hstack([point_base, ones])
            else:
                point_base_homo = point_base
        
        # 计算相机在基座坐标系中的位姿
        camera_pose = self.robot_pose_to_camera_pose(robot_pose)
        
        # 转换点到相机坐标系
        camera_pose_inv = np.linalg.inv(camera_pose)
        
        if len(point_base_homo.shape) == 1:
            point_camera = camera_pose_inv @ point_base_homo
            return point_camera[:3]
        else:
            # 处理多个点
            point_camera = (camera_pose_inv @ point_base_homo.T).T
            return point_camera[:, :3]
    
    def transform_point_gripper_to_camera(self, point_gripper: np.ndarray) -> np.ndarray:
        """
        将末端执行器坐标系中的点转换到相机坐标系
        
        Args:
            point_gripper: 末端执行器坐标系中的点 (3D或齐次坐标)
            
        Returns:
            相机坐标系中的点 (3D坐标)
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 转换为齐次坐标
        if len(point_gripper.shape) == 1:
            if point_gripper.shape[0] == 3:
                point_gripper_homo = np.append(point_gripper, 1.0)
            else:
                point_gripper_homo = point_gripper
        else:
            if point_gripper.shape[1] == 3:
                ones = np.ones((point_gripper.shape[0], 1))
                point_gripper_homo = np.hstack([point_gripper, ones])
            else:
                point_gripper_homo = point_gripper
        
        # 直接使用手眼变换
        if len(point_gripper_homo.shape) == 1:
            point_camera = self.hand_eye_transform @ point_gripper_homo
            return point_camera[:3]
        else:
            point_camera = (self.hand_eye_transform @ point_gripper_homo.T).T
            return point_camera[:, :3]
    
    def transform_point_camera_to_gripper(self, point_camera: np.ndarray) -> np.ndarray:
        """
        将相机坐标系中的点转换到末端执行器坐标系
        
        Args:
            point_camera: 相机坐标系中的点 (3D或齐次坐标)
            
        Returns:
            末端执行器坐标系中的点 (3D坐标)
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 转换为齐次坐标
        if len(point_camera.shape) == 1:
            if point_camera.shape[0] == 3:
                point_camera_homo = np.append(point_camera, 1.0)
            else:
                point_camera_homo = point_camera
        else:
            if point_camera.shape[1] == 3:
                ones = np.ones((point_camera.shape[0], 1))
                point_camera_homo = np.hstack([point_camera, ones])
            else:
                point_camera_homo = point_camera
        
        # 使用手眼变换的逆
        hand_eye_inv = np.linalg.inv(self.hand_eye_transform)
        
        if len(point_camera_homo.shape) == 1:
            point_gripper = hand_eye_inv @ point_camera_homo
            return point_gripper[:3]
        else:
            point_gripper = (hand_eye_inv @ point_camera_homo.T).T
            return point_gripper[:, :3]
    
    def validate_hand_eye_calibration(self, test_poses: List[np.ndarray], 
                                    test_target_poses: List[np.ndarray]) -> Dict:
        """
        验证手眼标定结果质量
        
        Args:
            test_poses: 测试用的机器人位姿列表 (T_gripper_to_base)
            test_target_poses: 对应的标定板位姿列表 (T_target_to_cam)
            
        Returns:
            验证结果字典
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        if len(test_poses) != len(test_target_poses):
            raise ValueError("位姿数量和观察数量不匹配")
        
        if len(test_poses) < 2:
            raise ValueError("至少需要2个测试位姿")
        
        translation_errors = []
        rotation_errors = []
        
        for i in range(len(test_poses)):
            for j in range(i + 1, len(test_poses)):
                # 计算机器人位姿的相对变换
                T_gripper_rel = np.linalg.inv(test_poses[i]) @ test_poses[j]
                
                # 计算相机观察到的标定板相对变换
                T_target_rel = test_target_poses[j] @ np.linalg.inv(test_target_poses[i])
                
                # 通过手眼变换预测的相对变换
                # 根据方程 A * X = X * B，其中 A = T_gripper_rel, B = T_target_rel, X = T_gripper_to_cam
                T_predicted = np.linalg.inv(self.hand_eye_transform) @ T_gripper_rel @ self.hand_eye_transform
                
                # 计算误差
                T_error = T_target_rel @ np.linalg.inv(T_predicted)
                
                # 平移误差
                translation_error = np.linalg.norm(T_error[:3, 3])
                translation_errors.append(translation_error)
                
                # 旋转误差
                R_error = T_error[:3, :3]
                trace_R = np.trace(R_error)
                trace_R = np.clip(trace_R, -1, 3)
                rotation_error = np.arccos(np.clip((trace_R - 1) / 2, -1, 1))
                rotation_errors.append(np.degrees(rotation_error))
        
        # 统计误差
        return {
            'num_test_pairs': len(translation_errors),
            'translation_error_m': {
                'mean': np.mean(translation_errors),
                'std': np.std(translation_errors),
                'max': np.max(translation_errors),
                'min': np.min(translation_errors)
            },
            'rotation_error_deg': {
                'mean': np.mean(rotation_errors),
                'std': np.std(rotation_errors),
                'max': np.max(rotation_errors),
                'min': np.min(rotation_errors)
            },
            'overall_quality': self._assess_calibration_quality(
                np.mean(translation_errors), np.mean(rotation_errors)
            )
        }
    
    def project_points_to_image(self, points_3d: np.ndarray, 
                              camera_matrix: np.ndarray,
                              dist_coeffs: np.ndarray,
                              robot_pose: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        将3D点投影到图像平面
        
        Args:
            points_3d: 基座坐标系中的3D点 (Nx3)
            camera_matrix: 相机内参矩阵
            dist_coeffs: 相机畸变系数
            robot_pose: 当前机器人位姿
            
        Returns:
            投影后的2D图像点, 是否在相机前方的mask
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 将基座坐标系中的点转换到相机坐标系
        points_camera = self.transform_point_base_to_camera(points_3d, robot_pose)
        
        # 过滤掉在相机后方的点
        valid_mask = points_camera[:, 2] > 0
        
        if np.any(valid_mask):
            # 投影到图像平面
            points_2d, _ = cv2.projectPoints(
                points_camera[valid_mask].reshape(-1, 1, 3),
                np.zeros(3), np.zeros(3),  # 不需要额外的旋转和平移
                camera_matrix, dist_coeffs
            )
            
            # 重新组织结果
            result_2d = np.full((len(points_3d), 2), np.nan)
            result_2d[valid_mask] = points_2d.reshape(-1, 2)
        else:
            result_2d = np.full((len(points_3d), 2), np.nan)
        
        return result_2d, valid_mask
    
    def get_hand_eye_info(self) -> Dict:
        """
        获取手眼标定信息
        
        Returns:
            包含标定信息的字典
        """
        if self.hand_eye_transform is None:
            return {'status': 'not_loaded'}
        
        # 提取位姿信息
        R = self.hand_eye_transform[:3, :3]
        t = self.hand_eye_transform[:3, 3]
        
        # 转换为四元数和欧拉角
        q = self._rotation_matrix_to_quaternion(R)
        euler = self._rotation_matrix_to_euler(R)
        
        info = {
            'status': 'loaded',
            'calibration_type': self.calibration_type,
            'calibration_date': self.calibration_date,
            'calibration_error': self.calibration_error,
            'num_poses': self.num_poses,
            'translation': {
                'x': float(t[0]),
                'y': float(t[1]),
                'z': float(t[2]),
                'norm': float(np.linalg.norm(t))
            },
            'rotation_quaternion': {
                'x': float(q[0]),
                'y': float(q[1]),
                'z': float(q[2]),
                'w': float(q[3])
            },
            'rotation_euler_degrees': {
                'roll': float(np.degrees(euler[0])),
                'pitch': float(np.degrees(euler[1])),
                'yaw': float(np.degrees(euler[2]))
            },
            'transform_matrix': self.hand_eye_transform.tolist()
        }
        
        # 添加详细误差信息
        if self.detailed_error:
            info['detailed_error'] = self.detailed_error
        
        return info
    
    def save_hand_eye_transform(self, file_path: str, 
                              additional_info: Dict = None) -> bool:
        """
        保存手眼变换到文件
        
        Args:
            file_path: 保存路径
            additional_info: 额外信息
            
        Returns:
            是否保存成功
        """
        if self.hand_eye_transform is None:
            print("没有可保存的手眼变换")
            return False
        
        try:
            info = self.get_hand_eye_info()
            
            if additional_info:
                info.update(additional_info)
            
            if file_path.endswith('.yaml'):
                with open(file_path, 'w') as f:
                    yaml.dump(info, f, default_flow_style=False)
            elif file_path.endswith('.json'):
                with open(file_path, 'w') as f:
                    json.dump(info, f, indent=2)
            else:
                raise ValueError("不支持的文件格式")
            
            print(f"手眼变换保存成功: {file_path}")
            return True
            
        except Exception as e:
            print(f"保存手眼变换失败: {str(e)}")
            return False
    
    def _is_valid_transform(self, T: np.ndarray) -> bool:
        """检查变换矩阵是否有效"""
        if T.shape != (4, 4):
            return False
        
        # 检查旋转矩阵
        R = T[:3, :3]
        
        # 旋转矩阵应该是正交的
        if not np.allclose(R @ R.T, np.eye(3), atol=1e-3):
            return False
        
        # 行列式应该为1
        if not np.allclose(np.linalg.det(R), 1.0, atol=1e-3):
            return False
        
        # 检查最后一行
        if not np.allclose(T[3, :], [0, 0, 0, 1]):
            return False
        
        return True
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数"""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        
        return np.array([x, y, z, w])
    
    def _rotation_matrix_to_euler(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转欧拉角 (ZYX顺序)"""
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        
        return np.array([x, y, z])
    
    def _assess_calibration_quality(self, mean_trans_error: float, 
                                  mean_rot_error: float) -> str:
        """评估标定质量"""
        if mean_trans_error < 0.002 and mean_rot_error < 1.0:
            return "excellent"
        elif mean_trans_error < 0.005 and mean_rot_error < 2.0:
            return "good"
        elif mean_trans_error < 0.01 and mean_rot_error < 5.0:
            return "acceptable"
        else:
            return "poor"
    
    def print_hand_eye_info(self):
        """打印手眼标定信息"""
        info = self.get_hand_eye_info()
        
        if info['status'] != 'loaded':
            print("未加载手眼标定结果")
            return
        
        print("="*60)
        print("Eye-in-Hand 手眼标定信息")
        print("="*60)
        print(f"标定类型: {info['calibration_type']}")
        print(f"标定日期: {info['calibration_date']}")
        print(f"使用位姿数: {info['num_poses']}")
        print(f"标定误差: {info['calibration_error']:.6f}")
        
        # 显示详细误差信息
        if 'detailed_error' in info and info['detailed_error']:
            error_info = info['detailed_error']
            print(f"\n详细误差信息:")
            print(f"  平移误差: {error_info.get('translation_error_m', 0):.6f} 米")
            print(f"  旋转误差: {error_info.get('rotation_error_deg', 0):.3f} 度")
            print(f"  位姿对数: {error_info.get('num_pose_pairs', 0)}")
        
        t = info['translation']
        print(f"\n末端执行器到相机的变换:")
        print(f"平移 (x, y, z): ({t['x']:.6f}, {t['y']:.6f}, {t['z']:.6f}) 米")
        print(f"平移距离: {t['norm']:.6f} 米")
        
        q = info['rotation_quaternion']
        print(f"四元数 (x, y, z, w): ({q['x']:.6f}, {q['y']:.6f}, {q['z']:.6f}, {q['w']:.6f})")
        
        e = info['rotation_euler_degrees']
        print(f"欧拉角 (roll, pitch, yaw): ({e['roll']:.2f}°, {e['pitch']:.2f}°, {e['yaw']:.2f}°)")
        
        print(f"\n变换矩阵 T_gripper_to_cam:")
        print(self.hand_eye_transform)
        print("="*60)


def create_sample_hand_eye_transform() -> np.ndarray:
    """
    创建示例手眼变换（用于测试）
    
    Returns:
        4x4变换矩阵
    """
    # 示例：相机相对于末端执行器的位置
    # 假设相机安装在末端执行器前方15cm，上方8cm，右侧3cm
    # 并且相机绕X轴旋转了25度（向下看）
    
    # 平移 (单位：米)
    t = np.array([0.03, 0.08, 0.15])  # x, y, z
    
    # 旋转（绕X轴25度）
    angle = np.radians(25)
    R = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])
    
    # 构建变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    return T


def demo_hand_eye_utils():
    """演示手眼工具的使用"""
    print("="*60)
    print("Hand-Eye Calibration Utils 演示")
    print("="*60)
    
    # 1. 创建示例变换
    print("\n1. 创建示例手眼变换...")
    sample_transform = create_sample_hand_eye_transform()
    
    # 2. 创建工具实例
    hand_eye_utils = HandEyeUtils()
    hand_eye_utils.set_hand_eye_transform(sample_transform, error=0.003)
    
    # 3. 打印信息
    print("\n2. 手眼标定信息:")
    hand_eye_utils.print_hand_eye_info()
    
    # 4. 位姿转换演示
    print("\n3. 位姿转换演示:")
    robot_pose = np.eye(4)
    robot_pose[:3, 3] = [0.5, 0.3, 0.2]  # 机器人在 (0.5, 0.3, 0.2)
    
    camera_pose = hand_eye_utils.robot_pose_to_camera_pose(robot_pose)
    print(f"机器人位姿: {robot_pose[:3, 3]}")
    print(f"相机位姿: {camera_pose[:3, 3]}")
    
    # 5. 坐标转换演示
    print("\n4. 坐标转换演示:")
    
    # 相机坐标系中的点
    point_camera = np.array([0.1, 0.05, 0.5])
    point_base = hand_eye_utils.transform_point_camera_to_base(point_camera, robot_pose)
    print(f"相机坐标系中的点: {point_camera}")
    print(f"基座坐标系中的点: {point_base}")
    
    # 反向转换验证
    point_camera_back = hand_eye_utils.transform_point_base_to_camera(point_base, robot_pose)
    print(f"反向转换验证: {point_camera_back}")
    print(f"转换误差: {np.linalg.norm(point_camera - point_camera_back):.8f}")
    
    # 6. 末端执行器与相机之间的直接转换
    print("\n5. 末端执行器-相机坐标转换:")
    point_gripper = np.array([0.02, -0.01, 0.05])
    point_camera_direct = hand_eye_utils.transform_point_gripper_to_camera(point_gripper)
    print(f"末端执行器坐标系中的点: {point_gripper}")
    print(f"相机坐标系中的点: {point_camera_direct}")
    
    # 反向转换
    point_gripper_back = hand_eye_utils.transform_point_camera_to_gripper(point_camera_direct)
    print(f"反向转换验证: {point_gripper_back}")
    print(f"转换误差: {np.linalg.norm(point_gripper - point_gripper_back):.8f}")
    
    print("\n" + "="*60)
    print("演示完成！")
    print("="*60)


if __name__ == '__main__':
    # 运行演示
    demo_hand_eye_utils() 
    