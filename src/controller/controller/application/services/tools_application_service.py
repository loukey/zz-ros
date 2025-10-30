"""
工具应用服务
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import List
from controller.domain import KinematicDomainService, RobotStateDomainService
import numpy as np


class ToolsApplicationService(QObject):
    """
    工具应用服务
    
    职责：
    1. 接收6个关节角度
    2. 调用KinematicDomainService计算正运动学
    3. 格式化结果并发射信号
    4. 获取当前机械臂关节角度
    5. 计算逆运动学
    """
    
    # 信号
    calculation_result_signal = pyqtSignal(dict)  # 发送正运动学计算结果
    current_angles_signal = pyqtSignal(list)  # 发送当前关节角度
    inverse_result_signal = pyqtSignal(dict)  # 发送逆运动学计算结果
    
    def __init__(self, kinematic_service: KinematicDomainService, robot_state_service: RobotStateDomainService):
        super().__init__()
        self.kinematic_service = kinematic_service
        self.robot_state_service = robot_state_service
    
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
    
    def get_current_joint_angles(self):
        """
        获取当前机械臂关节角度
        
        Emits:
            current_angles_signal: List[float] - 6个关节角度（弧度）
        """
        try:
            # 从机械臂状态服务获取当前角度
            current_angles = self.robot_state_service.get_current_angles()
            # 发射信号
            self.current_angles_signal.emit(current_angles)
        except Exception as e:
            # 如果获取失败，发送空列表
            self.current_angles_signal.emit([0.0] * 6)
    
    def calculate_inverse_kinematics(self, rotation_matrix: List[List[float]], position: List[float], initial_theta: List[float] = None):
        """
        计算逆运动学
        
        Args:
            rotation_matrix: 3x3 旋转矩阵
            position: [x, y, z] 位置（米）
            initial_theta: 初始关节角度（用于选择最接近的解）
            
        Emits:
            inverse_result_signal 包含：
            {
                "joint_angles": [θ1, θ2, θ3, θ4, θ5, θ6],  # 弧度
                "joint_angles_deg": [θ1, θ2, θ3, θ4, θ5, θ6]  # 度
            }
            或错误信息：
            {
                "error": "错误信息"
            }
        """
        try:
            # 转换为 numpy 数组
            rm = np.array(rotation_matrix)
            pos = np.array(position)
            
            # 如果没有提供初始角度，使用当前角度
            if initial_theta is None:
                initial_theta = self.robot_state_service.get_current_angles()
            
            # 调用逆运动学求解
            solution = self.kinematic_service.inverse_kinematic(rm, pos, initial_theta)
            
            # 构建结果字典
            result = {
                "joint_angles": solution,  # 弧度
                "joint_angles_deg": [np.rad2deg(angle) for angle in solution]  # 度
            }
            
            # 发射信号
            self.inverse_result_signal.emit(result)
            
        except Exception as e:
            # 错误处理
            error_result = {
                "error": f"逆解计算失败: {str(e)}"
            }
            self.inverse_result_signal.emit(error_result)

