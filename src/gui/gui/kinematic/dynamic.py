from .base import *
from math import pi


class Dynamic:
    def __init__(self):
        self.link_masses = [4.64,10.755,3.925,1.24,1.24,2.549]
        self.link_com_positions = [
            np.array([0, 0, 0]),
            np.array([0.2125, 0, 0.134]),
            np.array([0.1818, 0, 0]),
            np.array([0, 0, 0]),
            np.array([0, 0, 0]),
            np.array([0, 0, 0.0745])
        ]
        '''
        i | 连杆长度 a_{i-1} | 连杆转角 α_{i-1}   | 连杆偏距 d_i  | 关节角 θ_i
        ---------------------------------------------------------------------
        1 | 0.0             | 0.0                | 0.0       | 0.0
        2 | 0.0             | -π/2 (90°)         | 0.0         | -π/2 (90°)
        3 | 0.425           | pi                | 0.0           | 0.0
        4 | 0.401           | pi                | 0.0856        | π/2 (90°)
        5 | 0.0             | π/2 (90°)          | 0.086        | 0.0
        6 | 0.0             | -π/2 (-90°)        | 0.0725       | 0.0
        '''
        self.DH_matrix = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, -pi/2, 0.0, -pi/2],
            [0.425, pi, 0.0, 0.0],
            [0.401, pi, 0.0856, pi/2],
            [0.0, pi/2, 0.086, 0.0],
            [0.0, -pi/2, 0.0725, 0.0]
        ])
        # 451
        self.g_vector = np.array([0, 0, -9.81])
        self.n = 6

    def get_link_com_in_base(self, q, dh_params, link_com_positions, link_index):
        """
        计算第link_index个连杆的质心在基坐标系下的位置
        q: 当前关节角度数组（单位：弧度）
        dh_params: 每个关节的DH参数列表，每个元素为(a, alpha, d, theta_offset)
        link_com_positions: 每个连杆质心在本体坐标系下的坐标（3维向量）
        link_index: 连杆编号（0~n-1）
        """
        # 1. 计算从基坐标系到该连杆本体坐标系的齐次变换矩阵
        T = np.eye(4)
        for i in range(link_index + 1):
            a, alpha, d, theta_offset = dh_params[i]
            theta = q[i] + theta_offset  # 实际关节角度 = 当前角度 + 偏置
            T_i = dh_to_rotation_matrix(a, alpha, d, theta)
            T = np.dot(T, T_i)  # 累乘变换矩阵

        # 2. 质心在本体坐标系下的齐次坐标
        com_local = np.append(link_com_positions[link_index], 1)  # [x, y, z, 1]

        # 3. 变换到基坐标系
        com_base = np.dot(T, com_local)  # [x, y, z, 1]
        return com_base[:3]  # 返回前三个分量

    # 2. 计算每个连杆质心对每个关节的雅可比矩阵的转动轴
    def get_jacobian_column(self, q, link_index, com_base, joint_index):
        """
        输入：q为当前关节角度，link_index为连杆编号，com_world为该连杆质心在基坐标系下的位置，joint_index为关节编号
        输出：该质心对joint_index关节的雅可比列（3维向量）
        """
        # 计算joint_index关节的转动轴在基坐标系下的方向
        z_axis = self.get_joint_axis_in_base(q, self.DH_matrix, joint_index)  # 3维向量
        # 计算joint_index关节在基坐标系下的位置
        joint_pos = self.get_joint_position_in_base(q, self.DH_matrix, joint_index)  # 3维向量
        # 计算质心相对该关节的矢量
        r = com_base - joint_pos
        # 旋转关节的雅可比列为 z_axis × r
        return np.cross(z_axis, r)

    # 3. 计算重力补偿力矩
    def compute_gravity_compensation(self, q):
        """
        输入：q为当前关节角度
        输出：每个关节的重力补偿力矩（n维向量）
        """
        tau_g = np.zeros(self.n)  # 初始化补偿力矩
        for i in range(self.n):  # 对每个关节
            for j in range(i, self.n):  # 对每个后续连杆（含自身）
                # 1. 计算第j连杆质心在基坐标系下的位置
                com_base = self.get_link_com_in_base(q, self.DH_matrix, self.link_com_positions, j)
                # 2. 计算该质心对第i关节的雅可比列
                J_col = self.get_jacobian_column(q, j, com_base, i)
                # 3. 计算重力对该关节的力矩贡献
                tau_g[i] += self.link_masses[j] * self.g_vector.dot(J_col)

        tau_g[:3] *= 0.35
        tau_g[3:] *= 0.25
        return -tau_g

    def get_joint_axis_in_base(self, q, dh_params, joint_index):
        """
        返回第joint_index个关节的z轴（转动轴）在基坐标系下的方向（3维向量）
        """
        # 从基坐标系累乘到joint_index的变换矩阵
        T = np.eye(4)
        for i in range(joint_index+1):
            a, alpha, d, theta_offset = dh_params[i]
            theta = q[i] + theta_offset
            T = np.dot(T, dh_to_rotation_matrix(a, alpha, d, theta))
        # 关节z轴在本体坐标系下为[0, 0, 1]
        z_axis_local = np.array([0, 0, 1, 0])  # 齐次向量，最后一位0表示方向向量
        z_axis_base = np.dot(T, z_axis_local)[:3]
        return z_axis_base

    def get_joint_position_in_base(self, q, dh_params, joint_index):
        """
        返回第joint_index个关节的原点在基坐标系下的位置（3维向量）
        """
        # 从基坐标系累乘到joint_index的变换矩阵
        T = np.eye(4)
        for i in range(joint_index+1):
            a, alpha, d, theta_offset = dh_params[i]
            theta = q[i] + theta_offset
            T = np.dot(T, dh_to_rotation_matrix(a, alpha, d, theta))
        # 原点在本体坐标系下为[0, 0, 0, 1]
        origin_local = np.array([0, 0, 0, 1])
        origin_base = np.dot(T, origin_local)[:3]
        return origin_base
