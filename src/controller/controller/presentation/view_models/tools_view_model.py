"""
工具ViewModel
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import List
from controller.application import ToolsApplicationService


class ToolsViewModel(QObject):
    """工具 ViewModel。
    
    职责：
    1. 转发 UI 的计算请求到 Application Service
    2. 转发 Application Service 的结果信号到 UI
    3. 处理获取当前位置和逆运动学计算请求
    
    Attributes:
        calculation_result (pyqtSignal): 正运动学结果信号，携带结果字典。
        current_angles_received (pyqtSignal): 当前关节角度信号，携带角度列表。
        inverse_result (pyqtSignal): 逆运动学结果信号，携带结果字典。
        app_service (ToolsApplicationService): 工具应用服务。
    """
    
    # 信号（转发给UI）
    calculation_result = pyqtSignal(dict)  # 正运动学结果
    current_angles_received = pyqtSignal(list)  # 当前关节角度
    inverse_result = pyqtSignal(dict)  # 逆运动学结果
    
    def __init__(self, app_service: ToolsApplicationService):
        """初始化工具 ViewModel。
        
        Args:
            app_service (ToolsApplicationService): 工具应用服务。
        """
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
        """请求计算正运动学（转发给 Application Service）。
        
        Args:
            joint_angles (List[float]): 6个关节角度（弧度）。
        """
        self.app_service.calculate_forward_kinematics(joint_angles)
    
    def get_current_position(self):
        """请求获取当前机械臂关节角度。"""
        self.app_service.get_current_joint_angles()
    
    def calculate_inverse_kinematics(self, rotation_matrix: List[List[float]], position: List[float], initial_theta: List[float] = None):
        """请求计算逆运动学。
        
        Args:
            rotation_matrix (List[List[float]]): 3x3 旋转矩阵。
            position (List[float]): [x, y, z] 位置（米）。
            initial_theta (List[float], optional): 初始关节角度. Defaults to None.
        """
        self.app_service.calculate_inverse_kinematics(rotation_matrix, position, initial_theta)

