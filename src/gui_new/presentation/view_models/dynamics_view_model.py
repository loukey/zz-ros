"""
动力学视图模型
"""
from typing import List, Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from application.dto.robot_dto import JointAnglesDTO
from shared.config.container import ServiceLocator
import numpy as np


class DynamicsViewModel(BaseViewModel):
    """动力学视图模型"""
    
    # 动力学特有信号
    connection_status_changed = pyqtSignal(bool)
    torque_calculated = pyqtSignal(list)  # 计算出的力矩
    force_calculated = pyqtSignal(list)  # 计算出的力
    dynamics_updated = pyqtSignal(dict)  # 动力学参数更新
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # UI状态
        self.is_connected = False
        self.current_torques = [0.0] * 6  # 当前关节力矩 (Nm)
        self.external_forces = [0.0, 0.0, 0.0]  # 外部力 [Fx, Fy, Fz] (N)
        self.external_moments = [0.0, 0.0, 0.0]  # 外部力矩 [Mx, My, Mz] (Nm)
        
        # 动力学参数
        self.payload_mass = 0.0  # 负载质量 (kg)
        self.payload_center = [0.0, 0.0, 0.0]  # 负载质心 [x, y, z] (mm)
        self.joint_stiffness = [1000.0] * 6  # 关节刚度 (Nm/rad)
        self.joint_damping = [10.0] * 6  # 关节阻尼 (Nm·s/rad)
        
        # 重力向量 (m/s²)
        self.gravity_vector = [0.0, 0.0, -9.81]
    
    def calculate_forward_dynamics(self, joint_angles: List[float], 
                                 joint_velocities: List[float],
                                 joint_torques: List[float]) -> bool:
        """计算正动力学（给定力矩计算加速度）"""
        try:
            # 验证输入数据
            if len(joint_angles) != 6 or len(joint_velocities) != 6 or len(joint_torques) != 6:
                self.emit_error("角度、速度和力矩数据必须包含6个关节")
                return False
            
            # 转换为弧度
            angles_rad = [np.radians(angle) for angle in joint_angles]
            
            # TODO: 当Domain层动力学服务实现后，调用真实的正动力学计算
            # 目前使用简化计算
            accelerations = self._simple_forward_dynamics(
                angles_rad, joint_velocities, joint_torques
            )
            
            self.emit_status(f"正动力学计算完成 - 加速度: {[f'{a:.4f}' for a in accelerations]} rad/s²")
            return True
            
        except Exception as e:
            self.emit_error(f"正动力学计算失败: {str(e)}")
            return False
    
    def calculate_inverse_dynamics(self, joint_angles: List[float],
                                 joint_velocities: List[float],
                                 joint_accelerations: List[float]) -> bool:
        """计算逆动力学（给定运动计算所需力矩）"""
        try:
            # 验证输入数据
            if len(joint_angles) != 6 or len(joint_velocities) != 6 or len(joint_accelerations) != 6:
                self.emit_error("角度、速度和加速度数据必须包含6个关节")
                return False
            
            # 转换为弧度
            angles_rad = [np.radians(angle) for angle in joint_angles]
            
            # TODO: 当Domain层动力学服务实现后，调用真实的逆动力学计算
            # 目前使用简化计算
            torques = self._simple_inverse_dynamics(
                angles_rad, joint_velocities, joint_accelerations
            )
            
            # 更新当前力矩
            self.current_torques = torques
            self.torque_calculated.emit(torques)
            
            self.emit_status(f"逆动力学计算完成 - 力矩: {[f'{t:.4f}' for t in torques]} Nm")
            return True
            
        except Exception as e:
            self.emit_error(f"逆动力学计算失败: {str(e)}")
            return False
    
    def calculate_gravity_compensation(self, joint_angles: List[float]) -> bool:
        """计算重力补偿力矩"""
        try:
            # 验证输入数据
            if len(joint_angles) != 6:
                self.emit_error("角度数据必须包含6个关节")
                return False
            
            # 转换为弧度
            angles_rad = [np.radians(angle) for angle in joint_angles]
            
            # TODO: 当Domain层动力学服务实现后，调用真实的重力补偿计算
            # 目前使用简化计算
            gravity_torques = self._simple_gravity_compensation(angles_rad)
            
            self.torque_calculated.emit(gravity_torques)
            
            self.emit_status(f"重力补偿计算完成 - 力矩: {[f'{t:.4f}' for t in gravity_torques]} Nm")
            return True
            
        except Exception as e:
            self.emit_error(f"重力补偿计算失败: {str(e)}")
            return False
    
    def set_payload_parameters(self, mass: float, center: List[float]) -> bool:
        """设置负载参数"""
        try:
            if mass < 0:
                self.emit_error("负载质量不能为负数")
                return False
            
            if len(center) != 3:
                self.emit_error("负载质心必须包含3个坐标")
                return False
            
            self.payload_mass = mass
            self.payload_center = center.copy()
            
            # 发出参数更新信号
            params = {
                'payload_mass': mass,
                'payload_center': center
            }
            self.dynamics_updated.emit(params)
            
            self.emit_status(f"负载参数已更新 - 质量: {mass}kg, 质心: {center}mm")
            return True
            
        except Exception as e:
            self.emit_error(f"设置负载参数失败: {str(e)}")
            return False
    
    def set_joint_parameters(self, stiffness: List[float], damping: List[float]) -> bool:
        """设置关节参数"""
        try:
            if len(stiffness) != 6 or len(damping) != 6:
                self.emit_error("刚度和阻尼数据必须包含6个关节")
                return False
            
            # 验证参数范围
            for i, (k, d) in enumerate(zip(stiffness, damping)):
                if k <= 0 or d <= 0:
                    self.emit_error(f"第{i+1}关节的刚度和阻尼必须大于0")
                    return False
            
            self.joint_stiffness = stiffness.copy()
            self.joint_damping = damping.copy()
            
            # 发出参数更新信号
            params = {
                'joint_stiffness': stiffness,
                'joint_damping': damping
            }
            self.dynamics_updated.emit(params)
            
            self.emit_status("关节动力学参数已更新")
            return True
            
        except Exception as e:
            self.emit_error(f"设置关节参数失败: {str(e)}")
            return False
    
    def set_external_forces(self, forces: List[float], moments: List[float]) -> bool:
        """设置外部力和力矩"""
        try:
            if len(forces) != 3 or len(moments) != 3:
                self.emit_error("外部力和力矩必须包含3个分量")
                return False
            
            self.external_forces = forces.copy()
            self.external_moments = moments.copy()
            
            # 发出力更新信号
            self.force_calculated.emit(forces + moments)
            
            self.emit_status(f"外部力已设置 - 力: {forces}N, 力矩: {moments}Nm")
            return True
            
        except Exception as e:
            self.emit_error(f"设置外部力失败: {str(e)}")
            return False
    
    def get_dynamics_parameters(self) -> Dict[str, Any]:
        """获取动力学参数"""
        return {
            'payload_mass': self.payload_mass,
            'payload_center': self.payload_center.copy(),
            'joint_stiffness': self.joint_stiffness.copy(),
            'joint_damping': self.joint_damping.copy(),
            'external_forces': self.external_forces.copy(),
            'external_moments': self.external_moments.copy(),
            'gravity_vector': self.gravity_vector.copy(),
            'current_torques': self.current_torques.copy()
        }
    
    def set_connection_status(self, connected: bool):
        """设置连接状态"""
        self.is_connected = connected
        self.connection_status_changed.emit(connected)
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def _simple_forward_dynamics(self, angles: List[float], velocities: List[float], 
                               torques: List[float]) -> List[float]:
        """简化的正动力学计算（实际应使用完整的动力学模型）"""
        # 这是一个简化模型，实际应该使用完整的机器人动力学方程
        # M(q)·q̈ + C(q,q̇)·q̇ + G(q) = τ
        
        accelerations = []
        for i in range(6):
            # 简化计算：τ = I·α + b·ω + k·θ
            inertia = 1.0  # 简化惯量
            accel = (torques[i] - self.joint_damping[i] * velocities[i] - 
                    self.joint_stiffness[i] * angles[i]) / inertia
            accelerations.append(accel)
        
        return accelerations
    
    def _simple_inverse_dynamics(self, angles: List[float], velocities: List[float], 
                               accelerations: List[float]) -> List[float]:
        """简化的逆动力学计算（实际应使用完整的动力学模型）"""
        torques = []
        for i in range(6):
            # 简化计算：τ = I·α + b·ω + k·θ
            inertia = 1.0  # 简化惯量
            torque = (inertia * accelerations[i] + 
                     self.joint_damping[i] * velocities[i] + 
                     self.joint_stiffness[i] * angles[i])
            torques.append(torque)
        
        return torques
    
    def _simple_gravity_compensation(self, angles: List[float]) -> List[float]:
        """简化的重力补偿计算（实际应使用完整的重力模型）"""
        # 这是一个简化模型，实际应该计算各个连杆的重力影响
        gravity_torques = []
        for i, angle in enumerate(angles):
            # 简化计算：重力补偿力矩与关节角度的正弦相关
            base_gravity = 9.81 * (self.payload_mass + 1.0)  # 包含连杆质量
            torque = base_gravity * np.sin(angle) * (6 - i) / 6  # 远端关节影响更大
            gravity_torques.append(torque)
        
        return gravity_torques 