"""
运动学视图模型
"""
from typing import List, Optional, Tuple
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from application.dto.robot_dto import PoseDTO, JointAnglesDTO
from shared.config.container import ServiceLocator
import numpy as np


class KinematicViewModel(BaseViewModel):
    """运动学视图模型"""
    
    # 运动学特有信号
    position_updated = pyqtSignal(list)  # [x, y, z]
    orientation_updated = pyqtSignal(list)  # [rx, ry, rz] 欧拉角
    calculation_completed = pyqtSignal(bool, str)  # 计算完成信号(成功, 消息)
    angles_calculated = pyqtSignal(list)  # 逆运动学计算出的角度
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # UI状态
        self.current_position = [0.0, 0.0, 0.0]  # 当前位置 [x, y, z] (mm)
        self.current_orientation = [0.0, 0.0, 0.0]  # 当前姿态 [rx, ry, rz] (度)
        self.target_position = [250.0, 0.0, 400.0]  # 目标位置
        self.target_orientation = [0.0, 0.0, 0.0]  # 目标姿态
        self.calculated_angles = [0.0] * 6  # 计算得到的关节角度
    
    def calculate_forward_kinematics(self, joint_angles: List[float]) -> bool:
        """计算正运动学"""
        try:
            # 验证角度数据
            if len(joint_angles) != 6:
                self.emit_error("关节角度必须包含6个值")
                return False
            
            # 转换为弧度
            angles_rad = [np.radians(angle) for angle in joint_angles]
            joint_angles_dto = JointAnglesDTO(angles=angles_rad)
            
            # 调用Application服务计算正运动学
            pose_dto = self.robot_service.compute_forward_kinematics(joint_angles_dto)
            
            if pose_dto:
                # 更新当前位置和姿态
                self.current_position = pose_dto.position.copy()
                
                # 将四元数转换为欧拉角（度）
                quaternion = pose_dto.orientation
                euler_angles_rad = self._quaternion_to_euler(quaternion)
                self.current_orientation = [np.degrees(angle) for angle in euler_angles_rad]
                
                # 发出信号
                self.position_updated.emit(self.current_position)
                self.orientation_updated.emit(self.current_orientation)
                self.calculation_completed.emit(True, "正运动学计算成功")
                
                self.emit_status(f"正运动学计算完成 - 位置: [{self.current_position[0]:.2f}, "
                               f"{self.current_position[1]:.2f}, {self.current_position[2]:.2f}]mm")
                return True
            else:
                self.emit_error("正运动学计算失败")
                self.calculation_completed.emit(False, "正运动学计算失败")
                return False
                
        except Exception as e:
            self.emit_error(f"正运动学计算失败: {str(e)}")
            self.calculation_completed.emit(False, f"计算失败: {str(e)}")
            return False
    
    def calculate_inverse_kinematics(self, position: List[float], orientation: List[float]) -> bool:
        """计算逆运动学"""
        try:
            # 验证输入数据
            if len(position) != 3 or len(orientation) != 3:
                self.emit_error("位置和姿态必须分别包含3个值")
                return False
            
            # 将欧拉角转换为四元数
            euler_rad = [np.radians(angle) for angle in orientation]
            quaternion = self._euler_to_quaternion(euler_rad)
            
            # 创建位姿DTO
            pose_dto = PoseDTO(
                position=position.copy(),
                orientation=quaternion
            )
            
            # 调用Application服务计算逆运动学
            joint_angles_dto = self.robot_service.compute_inverse_kinematics(pose_dto)
            
            if joint_angles_dto:
                # 转换为度数
                angles_deg = [np.degrees(angle) for angle in joint_angles_dto.angles]
                self.calculated_angles = angles_deg
                
                # 更新目标位置和姿态
                self.target_position = position.copy()
                self.target_orientation = orientation.copy()
                
                # 发出信号
                self.angles_calculated.emit(angles_deg)
                self.calculation_completed.emit(True, "逆运动学计算成功")
                
                self.emit_status(f"逆运动学计算完成 - 角度: {[f'{a:.2f}°' for a in angles_deg]}")
                return True
            else:
                self.emit_error("逆运动学计算失败 - 无解或超出工作空间")
                self.calculation_completed.emit(False, "逆运动学计算失败")
                return False
                
        except Exception as e:
            self.emit_error(f"逆运动学计算失败: {str(e)}")
            self.calculation_completed.emit(False, f"计算失败: {str(e)}")
            return False
    
    def apply_angles_to_control(self) -> bool:
        """将计算的角度应用到控制系统"""
        try:
            if not self.calculated_angles or len(self.calculated_angles) != 6:
                self.emit_error("没有可用的计算角度")
                return False
            
            # 转换为弧度并更新机器人状态
            angles_rad = [np.radians(angle) for angle in self.calculated_angles]
            joint_angles_dto = JointAnglesDTO(angles=angles_rad)
            
            success, message = self.robot_service.update_joint_angles(joint_angles_dto)
            
            if success:
                self.emit_status("已将计算角度应用到控制系统")
                return True
            else:
                self.emit_error(f"应用角度失败: {message}")
                return False
                
        except Exception as e:
            self.emit_error(f"应用角度失败: {str(e)}")
            return False
    
    def update_current_pose_from_robot(self):
        """从机器人状态更新当前位姿"""
        try:
            robot_state = self.robot_service.get_robot_state()
            
            if robot_state and robot_state.position and robot_state.orientation:
                self.current_position = robot_state.position.copy()
                
                # 转换四元数为欧拉角
                euler_angles_rad = self._quaternion_to_euler(robot_state.orientation)
                self.current_orientation = [np.degrees(angle) for angle in euler_angles_rad]
                
                # 发出更新信号
                self.position_updated.emit(self.current_position)
                self.orientation_updated.emit(self.current_orientation)
                
        except Exception as e:
            self.emit_error(f"更新当前位姿失败: {str(e)}")
    
    def set_target_position(self, position: List[float]):
        """设置目标位置"""
        if len(position) == 3:
            self.target_position = position.copy()
            self.emit_status(f"设置目标位置: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]mm")
        else:
            self.emit_error("目标位置必须包含3个值")
    
    def set_target_orientation(self, orientation: List[float]):
        """设置目标姿态"""
        if len(orientation) == 3:
            self.target_orientation = orientation.copy()
            self.emit_status(f"设置目标姿态: [{orientation[0]:.2f}, {orientation[1]:.2f}, {orientation[2]:.2f}]°")
        else:
            self.emit_error("目标姿态必须包含3个值")
    
    def get_current_position(self) -> List[float]:
        """获取当前位置"""
        return self.current_position.copy()
    
    def get_current_orientation(self) -> List[float]:
        """获取当前姿态"""
        return self.current_orientation.copy()
    
    def get_target_position(self) -> List[float]:
        """获取目标位置"""
        return self.target_position.copy()
    
    def get_target_orientation(self) -> List[float]:
        """获取目标姿态"""
        return self.target_orientation.copy()
    
    def get_calculated_angles(self) -> List[float]:
        """获取计算出的关节角度"""
        return self.calculated_angles.copy()
    
    def _quaternion_to_euler(self, quaternion: List[float]) -> List[float]:
        """四元数转欧拉角（ZYX顺序）"""
        try:
            x, y, z, w = quaternion
            
            # Roll (x-axis rotation)
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)
            
            # Pitch (y-axis rotation)
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
            else:
                pitch = np.arcsin(sinp)
            
            # Yaw (z-axis rotation)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
            
            return [roll, pitch, yaw]
            
        except Exception:
            return [0.0, 0.0, 0.0]
    
    def _euler_to_quaternion(self, euler: List[float]) -> List[float]:
        """欧拉角转四元数（ZYX顺序）"""
        try:
            roll, pitch, yaw = euler
            
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            
            return [x, y, z, w]
            
        except Exception:
            return [0.0, 0.0, 0.0, 1.0] 