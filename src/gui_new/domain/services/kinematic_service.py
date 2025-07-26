"""
运动学服务 - 机器人正逆运动学计算
"""
from typing import List, Optional, Tuple
import numpy as np
from domain.value_objects.pose import JointAngles, Position, Orientation, Pose


class KinematicService:
    """运动学服务"""
    
    def __init__(self):
        # 默认DH参数 (6关节机械臂)
        self.dh_params = {
            'a': [0.0, 425.0, 392.25, 0.0, 0.0, 0.0],  # 连杆长度 (mm)
            'd': [162.5, 0.0, 0.0, 133.3, 99.7, 99.6],  # 偏移 (mm)
            'alpha': [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0],  # 扭转角 (rad)
            'theta_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 角度偏移 (rad)
        }
        
        # 关节限位
        self.joint_limits = {
            'min': [-np.pi, -np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi],
            'max': [np.pi, np.pi/2, np.pi/2, np.pi, np.pi/2, np.pi]
        }
    
    def forward_kinematics(self, joint_angles: JointAngles) -> Optional[Pose]:
        """正运动学计算"""
        try:
            angles = joint_angles.angles
            
            # 构建变换矩阵
            T = np.eye(4)
            
            for i in range(6):
                theta = angles[i] + self.dh_params['theta_offset'][i]
                d = self.dh_params['d'][i] / 1000.0  # mm转m
                a = self.dh_params['a'][i] / 1000.0  # mm转m
                alpha = self.dh_params['alpha'][i]
                
                # DH变换矩阵
                Ti = self._dh_transform(theta, d, a, alpha)
                T = T @ Ti
            
            # 提取位置和姿态
            position = Position(x=T[0, 3]*1000, y=T[1, 3]*1000, z=T[2, 3]*1000)  # m转mm
            
            # 提取旋转矩阵并转换为四元数
            R = T[:3, :3]
            orientation = self._rotation_matrix_to_quaternion(R)
            
            return Pose(position=position, orientation=orientation)
            
        except Exception:
            return None
    
    def inverse_kinematics(self, target_pose: Pose) -> Optional[JointAngles]:
        """逆运动学计算"""
        try:
            # 简化的逆运动学实现
            # 实际应用中需要使用数值方法或解析解
            
            # 这里使用简化的几何方法
            px = target_pose.position.x / 1000.0  # mm转m
            py = target_pose.position.y / 1000.0
            pz = target_pose.position.z / 1000.0
            
            # 简化计算
            joint1 = np.arctan2(py, px)
            r = np.sqrt(px*px + py*py)
            joint2 = np.arctan2(pz - self.dh_params['d'][0]/1000.0, r)
            joint3 = 0.0  # 简化
            joint4 = 0.0
            joint5 = 0.0
            joint6 = 0.0
            
            angles = [joint1, joint2, joint3, joint4, joint5, joint6]
            
            # 验证关节限位
            if self.validate_joint_limits(angles):
                return JointAngles.from_list(angles)
            else:
                return None
                
        except Exception:
            return None
    
    def validate_joint_limits(self, angles: List[float]) -> bool:
        """验证关节限位"""
        try:
            for i, angle in enumerate(angles):
                if not (self.joint_limits['min'][i] <= angle <= self.joint_limits['max'][i]):
                    return False
            return True
        except Exception:
            return False
    
    def compute_jacobian(self, joint_angles: JointAngles) -> Optional[np.ndarray]:
        """计算雅可比矩阵"""
        try:
            # 简化的雅可比计算
            # 实际应用中需要根据DH参数精确计算
            angles = joint_angles.angles
            
            # 6x6雅可比矩阵
            jacobian = np.zeros((6, 6))
            
            # 简化实现
            for i in range(6):
                jacobian[i, i] = 1.0
            
            return jacobian
            
        except Exception:
            return None
    
    def _dh_transform(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """DH变换矩阵"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> Orientation:
        """旋转矩阵转四元数"""
        try:
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
            
            return Orientation(x=x, y=y, z=z, w=w)
            
        except Exception:
            return Orientation(x=0.0, y=0.0, z=0.0, w=1.0) 