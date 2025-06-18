#!/usr/bin/env python3
"""
手眼标定辅助工具
提供手眼标定结果的使用和验证功能
"""

import numpy as np
import yaml
import cv2
from typing import Tuple, Optional, List
import os


class HandEyeUtils:
    """
    手眼标定工具类
    
    坐标系说明 (Eye-in-Hand配置)：
    - Base: 机器人基座坐标系
    - Gripper: 机器人末端执行器坐标系  
    - Camera: 相机坐标系
    - Board: 标定板坐标系
    
    变换关系：
    - T_base_to_gripper: 机器人正运动学输出的位姿
    - T_gripper_to_cam: 手眼标定求解的固定变换
    - T_base_to_cam = T_base_to_gripper * T_gripper_to_cam
    """
    
    def __init__(self, hand_eye_file: str = None):
        """
        初始化手眼工具
        
        Args:
            hand_eye_file: 手眼标定文件路径
        """
        self.hand_eye_transform = None
        self.calibration_error = None
        self.calibration_type = None
        
        if hand_eye_file and os.path.exists(hand_eye_file):
            self.load_hand_eye_calibration(hand_eye_file)
    
    def load_hand_eye_calibration(self, file_path: str) -> bool:
        """
        加载手眼标定结果
        
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
                self.calibration_type = data.get('calibration_type', 'unknown')
            elif file_path.endswith('.npz'):
                data = np.load(file_path)
                self.hand_eye_transform = data['hand_eye_transform']
                self.calibration_error = float(data['calibration_error'])
                self.calibration_type = 'eye_in_hand'  # 默认类型
            else:
                raise ValueError("不支持的文件格式")
            
            print(f"手眼标定结果加载成功: {file_path}")
            return True
        except Exception as e:
            print(f"加载手眼标定结果失败: {str(e)}")
            return False
    
    def set_hand_eye_transform(self, transform_matrix: np.ndarray, error: float = 0.0):
        """
        手动设置手眼变换
        
        Args:
            transform_matrix: 4x4变换矩阵
            error: 标定误差
        """
        self.hand_eye_transform = transform_matrix.copy()
        self.calibration_error = error
        self.calibration_type = 'manual'
        print("手眼变换手动设置完成")
    
    def robot_pose_to_camera_pose(self, robot_pose: np.ndarray) -> np.ndarray:
        """
        将机器人位姿转换为相机位姿 (Eye-in-Hand)
        
        Args:
            robot_pose: 机器人末端执行器位姿 (4x4矩阵)
            
        Returns:
            相机在机器人基座坐标系中的位姿 (4x4矩阵)
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 相机位姿 = 机器人位姿 * 手眼变换
        camera_pose = robot_pose @ self.hand_eye_transform
        return camera_pose
    
    def camera_pose_to_robot_pose(self, camera_pose: np.ndarray) -> np.ndarray:
        """
        将相机位姿转换为机器人位姿 (Eye-in-Hand)
        
        Args:
            camera_pose: 相机位姿 (4x4矩阵)
            
        Returns:
            机器人末端执行器位姿 (4x4矩阵)
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 机器人位姿 = 相机位姿 * 手眼变换的逆
        robot_pose = camera_pose @ np.linalg.inv(self.hand_eye_transform)
        return robot_pose
    
    def transform_point_camera_to_robot(self, point_camera: np.ndarray, robot_pose: np.ndarray) -> np.ndarray:
        """
        将相机坐标系中的点转换到机器人基座坐标系
        
        Args:
            point_camera: 相机坐标系中的点 (3D或齐次坐标)
            robot_pose: 当前机器人位姿 (4x4矩阵)
            
        Returns:
            机器人基座坐标系中的点
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 将点转换为齐次坐标
        if point_camera.shape[-1] == 3:
            point_camera_homo = np.hstack([point_camera, 1.0])
        else:
            point_camera_homo = point_camera
        
        # 相机位姿
        camera_pose = self.robot_pose_to_camera_pose(robot_pose)
        
        # 转换点到机器人基座坐标系
        point_robot = camera_pose @ point_camera_homo
        
        return point_robot[:3]
    
    def transform_point_robot_to_camera(self, point_robot: np.ndarray, robot_pose: np.ndarray) -> np.ndarray:
        """
        将机器人基座坐标系中的点转换到相机坐标系
        
        Args:
            point_robot: 机器人基座坐标系中的点 (3D或齐次坐标)
            robot_pose: 当前机器人位姿 (4x4矩阵)
            
        Returns:
            相机坐标系中的点
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        # 将点转换为齐次坐标
        if point_robot.shape[-1] == 3:
            point_robot_homo = np.hstack([point_robot, 1.0])
        else:
            point_robot_homo = point_robot
        
        # 相机位姿
        camera_pose = self.robot_pose_to_camera_pose(robot_pose)
        
        # 转换点到相机坐标系
        point_camera = np.linalg.inv(camera_pose) @ point_robot_homo
        
        return point_camera[:3]
    
    def get_hand_eye_info(self) -> dict:
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
        q = self.rotation_matrix_to_quaternion(R)
        euler = self.rotation_matrix_to_euler(R)
        
        return {
            'status': 'loaded',
            'calibration_type': self.calibration_type,
            'calibration_error': self.calibration_error,
            'translation': {
                'x': float(t[0]),
                'y': float(t[1]),
                'z': float(t[2])
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
    
    def validate_hand_eye_calibration(self, test_poses: List[np.ndarray], 
                                    test_observations: List[np.ndarray]) -> dict:
        """
        验证手眼标定结果
        
        Args:
            test_poses: 测试用的机器人位姿列表
            test_observations: 对应的相机观察结果列表
            
        Returns:
            验证结果字典
        """
        if self.hand_eye_transform is None:
            raise ValueError("请先加载手眼标定结果")
        
        if len(test_poses) != len(test_observations):
            raise ValueError("位姿数量和观察数量不匹配")
        
        errors = []
        
        for i in range(len(test_poses)):
            for j in range(i + 1, len(test_poses)):
                # 计算机器人位姿变化
                robot_rel = np.linalg.inv(test_poses[i]) @ test_poses[j]
                
                # 计算观察到的变化
                obs_rel = test_observations[j] @ np.linalg.inv(test_observations[i])
                
                # 通过手眼变换预测的变化
                predicted_rel = np.linalg.inv(self.hand_eye_transform) @ robot_rel @ self.hand_eye_transform
                
                # 计算误差
                error_matrix = obs_rel @ np.linalg.inv(predicted_rel)
                translation_error = np.linalg.norm(error_matrix[:3, 3])
                
                # 旋转误差（角度）
                trace_R = np.trace(error_matrix[:3, :3])
                rotation_error = np.arccos(np.clip((trace_R - 1) / 2, -1, 1))
                
                errors.append({
                    'translation_error': translation_error,
                    'rotation_error': np.degrees(rotation_error)
                })
        
        # 统计误差
        translation_errors = [e['translation_error'] for e in errors]
        rotation_errors = [e['rotation_error'] for e in errors]
        
        return {
            'num_test_pairs': len(errors),
            'translation_error': {
                'mean': np.mean(translation_errors),
                'std': np.std(translation_errors),
                'max': np.max(translation_errors),
                'min': np.min(translation_errors)
            },
            'rotation_error_degrees': {
                'mean': np.mean(rotation_errors),
                'std': np.std(rotation_errors),
                'max': np.max(rotation_errors),
                'min': np.min(rotation_errors)
            }
        }
    
    def rotation_matrix_to_quaternion(self, R):
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
    
    def rotation_matrix_to_euler(self, R):
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
    
    def print_hand_eye_info(self):
        """打印手眼标定信息"""
        info = self.get_hand_eye_info()
        
        if info['status'] != 'loaded':
            print("未加载手眼标定结果")
            return
        
        print("="*50)
        print("手眼标定信息")
        print("="*50)
        print(f"标定类型: {info['calibration_type']}")
        print(f"标定误差: {info['calibration_error']:.6f}")
        
        t = info['translation']
        print(f"\n平移 (x, y, z): ({t['x']:.6f}, {t['y']:.6f}, {t['z']:.6f}) 米")
        
        q = info['rotation_quaternion']
        print(f"四元数 (x, y, z, w): ({q['x']:.6f}, {q['y']:.6f}, {q['z']:.6f}, {q['w']:.6f})")
        
        e = info['rotation_euler_degrees']
        print(f"欧拉角 (roll, pitch, yaw): ({e['roll']:.2f}°, {e['pitch']:.2f}°, {e['yaw']:.2f}°)")
        
        print(f"\n变换矩阵:")
        print(self.hand_eye_transform)
        print("="*50)


def create_sample_hand_eye_transform():
    """创建示例手眼变换（用于测试）"""
    # 示例：相机相对于末端执行器的位置
    # 假设相机安装在末端执行器前方10cm，上方5cm，右侧2cm
    # 并且相机绕X轴旋转了30度
    
    # 平移
    t = np.array([0.02, 0.05, 0.10])  # x, y, z (米)
    
    # 旋转（绕X轴30度）
    angle = np.radians(30)
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


if __name__ == '__main__':
    # 示例使用
    print("手眼标定工具示例")
    
    # 创建示例变换
    sample_transform = create_sample_hand_eye_transform()
    
    # 创建工具实例
    hand_eye_utils = HandEyeUtils()
    hand_eye_utils.set_hand_eye_transform(sample_transform, error=0.002)
    
    # 打印信息
    hand_eye_utils.print_hand_eye_info()
    
    # 示例：机器人位姿转相机位姿
    robot_pose = np.eye(4)
    robot_pose[:3, 3] = [0.5, 0.3, 0.2]  # 机器人在 (0.5, 0.3, 0.2)
    
    camera_pose = hand_eye_utils.robot_pose_to_camera_pose(robot_pose)
    print(f"\n示例转换:")
    print(f"机器人位姿: {robot_pose[:3, 3]}")
    print(f"相机位姿: {camera_pose[:3, 3]}")
    
    # 示例：坐标转换
    point_camera = np.array([0.1, 0.05, 0.5])  # 相机坐标系中的点
    point_robot = hand_eye_utils.transform_point_camera_to_robot(point_camera, robot_pose)
    print(f"\n坐标转换示例:")
    print(f"相机坐标系中的点: {point_camera}")
    print(f"机器人坐标系中的点: {point_robot}") 