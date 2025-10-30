"""
工具ViewModel
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import List
from controller.application import ToolsApplicationService


class ToolsViewModel(QObject):
    """
    工具ViewModel
    
    职责：
    1. 转发UI的计算请求到Application Service
    2. 转发Application Service的结果信号到UI
    3. 处理获取当前位置和逆运动学计算请求
    """
    
    # 信号（转发给UI）
    calculation_result = pyqtSignal(dict)  # 正运动学结果
    current_angles_received = pyqtSignal(list)  # 当前关节角度
    inverse_result = pyqtSignal(dict)  # 逆运动学结果
    
    def __init__(self, app_service: ToolsApplicationService):
        super().__init__()
        self.app_service = app_service
        
        # 连接Application Service的信号
        self.app_service.calculation_result_signal.connect(
            self.calculation_result.emit
        )
        self.app_service.current_angles_signal.connect(
            self.current_angles_received.emit
        )
        self.app_service.inverse_result_signal.connect(
            self.inverse_result.emit
        )
    
    def calculate_kinematics(self, joint_angles: List[float]):
        """
        请求计算正运动学（转发给Application Service）
        
        Args:
            joint_angles: 6个关节角度（弧度）
        """
        self.app_service.calculate_forward_kinematics(joint_angles)
    
    def get_current_position(self):
        """
        请求获取当前机械臂关节角度
        """
        self.app_service.get_current_joint_angles()
    
    def calculate_inverse_kinematics(self, rotation_matrix: List[List[float]], position: List[float], initial_theta: List[float] = None):
        """
        请求计算逆运动学
        
        Args:
            rotation_matrix: 3x3 旋转矩阵
            position: [x, y, z] 位置（米）
            initial_theta: 初始关节角度（可选）
        """
        self.app_service.calculate_inverse_kinematics(rotation_matrix, position, initial_theta)

