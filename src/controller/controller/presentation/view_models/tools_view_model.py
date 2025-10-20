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
    """
    
    # 信号（转发给UI）
    calculation_result = pyqtSignal(dict)
    
    def __init__(self, app_service: ToolsApplicationService):
        super().__init__()
        self.app_service = app_service
        
        # 连接Application Service的信号
        self.app_service.calculation_result_signal.connect(
            self.calculation_result.emit
        )
    
    def calculate_kinematics(self, joint_angles: List[float]):
        """
        请求计算正运动学（转发给Application Service）
        
        Args:
            joint_angles: 6个关节角度（弧度）
        """
        self.app_service.calculate_forward_kinematics(joint_angles)

