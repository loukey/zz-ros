"""
动力学领域服务 - Domain层
实现重力补偿力矩计算
"""
import numpy as np
from math import pi
from domain.utils import KinematicUtils
from domain.value_objects import DHParam


class DynamicDomainService:
    """动力学领域服务 - 计算重力补偿力矩"""
    
    def __init__(self):
        # 连杆质量 (kg)
        self.link_masses = [4.64, 10.755, 3.925, 1.24, 1.24, 1.546]
        
        # 使用动力学DH参数
        dh_param = DHParam()
        self.dh_matrix = dh_param.get_dynamic_dh()
        
        # 连杆质心在本体坐标系下的位置 (m)
        self.link_com_positions = [
            np.array([0, 0, 0]),
            np.array([0.2125, 0, 0.134]),
            np.array([0.1818, 0, 0]),
            np.array([0, 0, 0]),
            np.array([0, 0, 0]),
            np.array([0, 0, 0.05])
        ]
        
        # 重力向量 (m/s²)
        self.g_vector = np.array([0, 0, -9.81])
        
        # 关节数量
        self.n = 6
        
        # 补偿系数（根据实际机械臂调整）
        self.compensation_factor_front = 0.35  # 前3个关节
        self.compensation_factor_rear = 0.25   # 后3个关节
    
    def compute_gravity_compensation(self, joint_angles):
        """
        计算重力补偿力矩
        
        参数:
            joint_angles: 当前关节角度（弧度），6维数组
        
        返回:
            List[float]: 每个关节的重力补偿力矩（Nm），6维数组
        """
        if len(joint_angles) != 6:
            raise ValueError("关节角度必须是6维数组")
        
        q = np.array(joint_angles, dtype=float)
        tau_g = np.zeros(self.n)  # 初始化补偿力矩
        
        # 对每个关节计算重力补偿力矩
        for i in range(self.n):
            # 对每个后续连杆（含自身）累加力矩贡献
            for j in range(i, self.n):
                # 1. 计算第j连杆质心在基坐标系下的位置
                com_base = self._get_link_com_in_base(q, j)
                
                # 2. 计算该质心对第i关节的雅可比列
                jacobian_col = self._get_jacobian_column(q, j, com_base, i)
                
                # 3. 计算重力对该关节的力矩贡献
                tau_g[i] += self.link_masses[j] * self.g_vector.dot(jacobian_col)
        
        # 应用补偿系数（根据实际机械臂调整）
        tau_g[:3] *= self.compensation_factor_front
        tau_g[3:] *= self.compensation_factor_rear
        
        # 返回负值（因为是补偿力矩）
        return (-tau_g).tolist()
    
    def _get_link_com_in_base(self, q, link_index):
        """
        计算第link_index个连杆的质心在基坐标系下的位置
        
        参数:
            q: 当前关节角度数组（单位：弧度）
            link_index: 连杆编号（0~5）
        
        返回:
            np.ndarray: 质心在基坐标系下的3维坐标
        """
        # 从基坐标系累乘到该连杆本体坐标系的齐次变换矩阵
        T = np.eye(4)
        for i in range(link_index + 1):
            a, alpha, d, theta_offset = self.dh_matrix[i]
            theta = q[i] + theta_offset  # 实际关节角度 = 当前角度 + 偏置
            T_i = KinematicUtils.dh2rm(a, alpha, d, theta)
            T = np.dot(T, T_i)  # 累乘变换矩阵
        
        # 质心在本体坐标系下的齐次坐标
        com_local = np.append(self.link_com_positions[link_index], 1)  # [x, y, z, 1]
        
        # 变换到基坐标系
        com_base = np.dot(T, com_local)  # [x, y, z, 1]
        return com_base[:3]  # 返回前三个分量
    
    def _get_jacobian_column(self, q, link_index, com_base, joint_index):
        """
        计算质心对关节的雅可比列（旋转关节）
        
        参数:
            q: 当前关节角度
            link_index: 连杆编号
            com_base: 该连杆质心在基坐标系下的位置
            joint_index: 关节编号
        
        返回:
            np.ndarray: 该质心对joint_index关节的雅可比列（3维向量）
        """
        # 计算joint_index关节的转动轴在基坐标系下的方向
        z_axis = self._get_joint_axis_in_base(q, joint_index)  # 3维向量
        
        # 计算joint_index关节在基坐标系下的位置
        joint_pos = self._get_joint_position_in_base(q, joint_index)  # 3维向量
        
        # 计算质心相对该关节的矢量
        r = com_base - joint_pos
        
        # 旋转关节的雅可比列为 z_axis × r
        return np.cross(z_axis, r)
    
    def _get_joint_axis_in_base(self, q, joint_index):
        """
        返回第joint_index个关节的z轴（转动轴）在基坐标系下的方向
        
        参数:
            q: 当前关节角度
            joint_index: 关节编号
        
        返回:
            np.ndarray: 关节z轴在基坐标系下的方向（3维向量）
        """
        # 从基坐标系累乘到joint_index的变换矩阵
        T = np.eye(4)
        for i in range(joint_index + 1):
            a, alpha, d, theta_offset = self.dh_matrix[i]
            theta = q[i] + theta_offset
            T = np.dot(T, KinematicUtils.dh2rm(a, alpha, d, theta))
        
        # 关节z轴在本体坐标系下为[0, 0, 1]
        z_axis_local = np.array([0, 0, 1, 0])  # 齐次向量，最后一位0表示方向向量
        z_axis_base = np.dot(T, z_axis_local)[:3]
        return z_axis_base
    
    def _get_joint_position_in_base(self, q, joint_index):
        """
        返回第joint_index个关节的原点在基坐标系下的位置
        
        参数:
            q: 当前关节角度
            joint_index: 关节编号
        
        返回:
            np.ndarray: 关节原点在基坐标系下的位置（3维向量）
        """
        # 从基坐标系累乘到joint_index的变换矩阵
        T = np.eye(4)
        for i in range(joint_index + 1):
            a, alpha, d, theta_offset = self.dh_matrix[i]
            theta = q[i] + theta_offset
            T = np.dot(T, KinematicUtils.dh2rm(a, alpha, d, theta))
        
        # 原点在本体坐标系下为[0, 0, 0, 1]
        origin_local = np.array([0, 0, 0, 1])
        origin_base = np.dot(T, origin_local)[:3]
        return origin_base