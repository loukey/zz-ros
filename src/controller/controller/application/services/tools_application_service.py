"""
工具应用服务
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import List
from controller.domain import KinematicDomainService


class ToolsApplicationService(QObject):
    """
    工具应用服务
    
    职责：
    1. 接收6个关节角度
    2. 调用KinematicDomainService计算正运动学
    3. 格式化结果并发射信号
    """
    
    # 信号
    calculation_result_signal = pyqtSignal(dict)  # 发送计算结果
    
    def __init__(self, kinematic_service: KinematicDomainService):
        super().__init__()
        self.kinematic_service = kinematic_service
    
    def calculate_forward_kinematics(self, joint_angles: List[float]):
        """
        计算正运动学
        
        Args:
            joint_angles: 6个关节角度（弧度）
            
        Emits:
            calculation_result_signal 包含：
            {
                "quaternion": [x, y, z, w],
                "position": [x, y, z],
                "rotation_matrix": [[...], [...], [...]]  # 3x3
            }
            或错误信息：
            {
                "error": "错误信息"
            }
        """
        try:
            # 获取四元数和位置
            quat, pos = self.kinematic_service.get_gripper2base(joint_angles)
            
            # 获取完整变换矩阵
            transform_matrix = self.kinematic_service.get_gripper2base_rm(joint_angles)
            rotation_matrix = transform_matrix[:3, :3]  # 提取旋转矩阵部分
            
            # 构建结果字典
            result = {
                "quaternion": quat.tolist(),  # [x, y, z, w]
                "position": pos.tolist(),     # [x, y, z]
                "rotation_matrix": rotation_matrix.tolist()  # 3x3 list
            }
            
            # 发射信号
            self.calculation_result_signal.emit(result)
            
        except Exception as e:
            # 错误处理
            error_result = {
                "error": f"计算失败: {str(e)}"
            }
            self.calculation_result_signal.emit(error_result)

