"""
手眼标定配置仓储 - Infrastructure层
负责从文件系统读取配置
"""
import json
import numpy as np
from pathlib import Path
from math import radians
from ament_index_python.packages import get_package_share_directory
from controller.domain.value_objects import (
    HandEyeCalibrationConfig,
    CameraIntrinsics,
    TargetOffset,
    EndEffectorAdjustment
)


class HandEyeCalibrationRepository:
    """
    手眼标定配置仓储
    
    职责：
    - 从 JSON 文件加载配置
    - 转换为 Domain 层的值对象
    - 处理文件不存在等异常
    """
    
    def __init__(self, config_path: str = None):
        """
        Args:
            config_path: 配置文件路径，默认为 controller/config/hand_eye_calibration.json
        """
        if config_path is None:
            # 使用 ROS2 包资源管理方式查找配置文件
            try:
                # ROS2 方式：从 share 目录加载
                package_share_dir = get_package_share_directory('controller')
                config_path = Path(package_share_dir) / 'config' / 'hand_eye_calibration.json'
            except Exception:
                # 如果包未安装，使用相对路径（开发模式）
                current_file = Path(__file__)
                controller_root = current_file.parent.parent.parent
                config_path = controller_root / "config" / "hand_eye_calibration.json"
        
        self.config_path = Path(config_path)
    
    def load(self) -> HandEyeCalibrationConfig:
        """
        加载手眼标定配置
        
        Returns:
            HandEyeCalibrationConfig: 配置值对象
            
        Raises:
            FileNotFoundError: 配置文件不存在
            ValueError: 配置格式错误
        """
        if not self.config_path.exists():
            raise FileNotFoundError(
                f"配置文件不存在: {self.config_path}\n"
                f"请在 controller/config/ 目录下创建 hand_eye_calibration.json"
            )
        
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except json.JSONDecodeError as e:
            raise ValueError(f"配置文件格式错误: {e}")
        
        # 解析手眼标定矩阵
        hand_eye_matrix = np.array(data['hand_eye_matrix'], dtype=float)
        
        # 解析相机内参
        intrinsics_data = data['camera_intrinsics']
        camera_intrinsics = CameraIntrinsics(
            fx=intrinsics_data['fx'],
            fy=intrinsics_data['fy'],
            cx=intrinsics_data['cx'],
            cy=intrinsics_data['cy']
        )
        
        # 解析目标偏移量
        offset_data = data['target_offset']
        target_offset = TargetOffset(
            x=offset_data['x'],
            y=offset_data['y'],
            z=offset_data['z']
        )
        
        # 解析末端姿态调整（角度转弧度）
        adjustment_data = data['end_effector_adjustment']
        end_effector_adjustment = EndEffectorAdjustment(
            z_rotation=radians(adjustment_data['z_rotation_deg']),
            y_rotation=radians(adjustment_data['y_rotation_deg']),
            x_rotation=radians(adjustment_data['x_rotation_deg'])
        )
        
        return HandEyeCalibrationConfig(
            hand_eye_matrix=hand_eye_matrix,
            camera_intrinsics=camera_intrinsics,
            target_offset=target_offset,
            end_effector_adjustment=end_effector_adjustment
        )
    
    def save(self, config: HandEyeCalibrationConfig):
        """
        保存配置到文件（可选功能，用于标定工具）
        
        Args:
            config: 手眼标定配置对象
        """
        # 确保目录存在
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        
        data = {
            "hand_eye_matrix": config.hand_eye_matrix.tolist(),
            "camera_intrinsics": {
                "fx": config.camera_intrinsics.fx,
                "fy": config.camera_intrinsics.fy,
                "cx": config.camera_intrinsics.cx,
                "cy": config.camera_intrinsics.cy
            },
            "target_offset": {
                "x": config.target_offset.x,
                "y": config.target_offset.y,
                "z": config.target_offset.z,
                "comment": "相对于零件位置的偏移量（米）"
            },
            "end_effector_adjustment": {
                "z_rotation_deg": np.degrees(config.end_effector_adjustment.z_rotation),
                "y_rotation_deg": np.degrees(config.end_effector_adjustment.y_rotation),
                "x_rotation_deg": np.degrees(config.end_effector_adjustment.x_rotation),
                "comment": "末端姿态调整（角度）"
            }
        }
        
        with open(self.config_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

