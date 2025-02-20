from .base import *
from functools import reduce
from math import acos, degrees


class Kinematic6DOF:
    def __init__(self):
        '''
        i | 连杆长度 a_{i-1} | 连杆转角 α_{i-1}   | 连杆偏距 d_i  | 关节角 θ_i
        ---------------------------------------------------------------------
        1 | 0.0             | 0.0                | 0.0       | 0.0
        2 | 0.0             | -π/2 (90°)         | 0.0         | -π/2 (90°)
        3 | 0.2435          | 0.0                | 0.0           | 0.0
        4 | 0.211           | 0.0                | 0.0865        | π/2 (90°)
        5 | 0.0             | π/2 (90°)          | 0.086        | 0.0
        6 | 0.0             | -π/2 (-90°)        | 0.0725       | 0.0
        '''
        self.theta_list = [0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]
        self.DH_matrix = np.array([
            [0.0, 0.0, 0.0, self.theta_list[0]],
            [0.0, -pi/2, 0.0, self.theta_list[1]],
            [0.425, 0.0, 0.0, self.theta_list[2]],
            [0.401, 0.0, 0.0856, self.theta_list[3]],
            [0.0, pi/2, 0.086, self.theta_list[4]],
            [0.0, -pi/2, 0.0725, self.theta_list[5]]
        ], dtype=float)

        self.rm_list = []
        self.get_rotation_matrix_list()
        self.forward_rm = self.forward_kinematic()

    def get_rotation_matrix_list(self):
        self.rm_list = []
        for i in range(len(self.DH_matrix)):
            self.rm_list.append(dh_to_rotation_matrix(*self.DH_matrix[i]))

    """
    正运动学求解变换矩阵
    """
    def forward_kinematic(self):
        return reduce(lambda x, y: x @ y, self.rm_list, np.eye(4))
    
    def update_dh(self, theta_list):
        for i in range(6):
            self.DH_matrix[i, 3] = theta_list[i]
        self.get_rotation_matrix_list()
        self.forward_rm = self.forward_kinematic()

    """
    逆运动学求解关节角度
    """
    def inverse_kinematic(self, A, B, C, px, py, pz):
        rm = XYZ_euler_angles_to_rotation_matrix(A, B, C)
        T06 = np.eye(4)
        T06[:3, :3] = rm
        T06[:3, 3] = np.array([px, py, pz])

        nx, ny, nz = T06[:3, 0]
        ox, oy, oz = T06[:3, 1]
        ax, ay, az = T06[:3, 2]
        a2 = self.DH_matrix[2, 0]
        a3 = self.DH_matrix[3, 0]
        d1 = self.DH_matrix[0, 2]
        d4 = self.DH_matrix[3, 2]
        d5 = self.DH_matrix[4, 2]
        d6 = self.DH_matrix[5, 2]

        # 添加奇异性判断
        if abs(ax**2 + ay**2 - d4**2) < 1e-6:
            print("Warning: Approaching shoulder singularity!")
        
        # theta1计算
        m = ay * d6 - py
        n = ax * d6 - px
        if m**2 + n**2 - d4**2 < 0:
            raise ValueError("Position unreachable - out of workspace")
        theta1_0 = atan2(m, n) - atan2(-d4, sqrt(m**2 + n**2 - d4**2))
        theta1_1 = atan2(m, n) - atan2(-d4, -sqrt(m**2 + n**2 - d4**2))
        print("get theta1: {} {}".format(degrees(theta1_0), degrees(theta1_1)))
        theta1 = theta1_0 if abs(theta1_0 - self.theta_list[0]) < abs(theta1_1 - self.theta_list[0]) else theta1_1
        print("self.theta_list[0]: {}".format(degrees(self.theta_list[0])))
        print("get theta1: {}".format(degrees(theta1)))
        
        # theta5
        s1 = sin(theta1)
        c1 = cos(theta1)
        theta5_0 = acos(-s1 * ax + c1 * ay)
        theta5_1 = -theta5_0
        theta5 = theta5_0 if abs(theta5_0 - self.theta_list[4]) < abs(theta5_1 - self.theta_list[4]) else theta5_1
        print("get theta5: {} {}".format(degrees(theta5_0), degrees(theta5_1)))

        # theta6
        if abs(sin(theta5)) < 1e-6:
            print("Warning: Wrist singularity detected!")
            theta6 = self.theta_list[5]  # 保持当前角度
        else:
            m1 = -sin(theta1) * nx + cos(theta1) * ny
            n1 = -sin(theta1) * ox + cos(theta1) * oy
            theta6 = atan2(-n1 / sin(theta5), m1 / sin(theta5))
        print("get theta6: {}".format(degrees(theta6)))

        # theta3
        c1 = cos(theta1)
        s1 = sin(theta1)
        s6 = sin(theta6)
        c6 = cos(theta6)
        m2 = d5 * (s6 * (nx * c1 + ny * s1) + c6 * (ox * c1 + oy * s1)) - d6 * (ax * c1 + ay * s1) + px * c1 + py * s1
        n2 = d5 * (oz * c6 + nz * s6) + pz - d1 - az * d6
        theta3_0 = acos((m2**2 + n2**2 - a2**2 - a3**2) / (2 * a2 * a3))
        theta3_1 = -acos((m2**2 + n2**2 - a2**2 - a3**2) / (2 * a2 * a3))
        theta3 = theta3_0 if abs(theta3_0 - self.theta_list[2]) < abs(theta3_1 - self.theta_list[2]) else theta3_1
        print("get theta3: {} {}".format(degrees(theta3_0), degrees(theta3_1)))

        # theta2
        s3 = sin(theta3)
        c3 = cos(theta3)
        s2 = (-(a3 * c3 + a2) * n2 - a3 * s3 * m2) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * c3)
        c2 = (m2 + a3 * s3 * s2) / (a3 * c3 + a2)
        theta2 = atan2(s2, c2)
        print("get theta2: {}".format(degrees(theta2)))

        # theta4
        c5 = cos(theta5)
        s5 = sin(theta5)
        c6 = cos(theta6)
        s6 = sin(theta6)
        c234 = c5 * (c6 * (nx * c1 + ny * s1) - s6 * (ox * c1 + oy * s1)) - s5 * (ax * c1 + ay * s1)
        s234 = -s6 * (nx * c1 + ny * s1) - c6 * (ox * c1 + oy * s1)
        theta4 = atan2(s234, c234) - theta2 - theta3
        print("get theta4: {}".format(degrees(theta4)))

        return [theta1, theta2, theta3, theta4, theta5, theta6]
        
    """
    test kinematic
    """
    def test_kinematic(self, theta_list):
        print("测试角度:", [degrees(t) for t in theta_list])
        self.theta_list = theta_list
        self.update_dh(self.theta_list)
        T06 = self.forward_kinematic()
        target_pos = T06[:3, 3]
        target_rm = T06[:3, :3]
        A, B, C = XYZ_rotation_matrix_to_euler_angles(target_rm)
        print("目标位置:", target_pos)
        
        try:
            solved_angles = self.inverse_kinematic(A, B, C, *target_pos)
            print("解算角度:", [degrees(t) for t in solved_angles])
            
            if self.verify_solution(solved_angles, target_pos):
                print("解算成功！误差在允许范围内")
            else:
                print("警告：解算结果误差过大")
                
        except Exception as e:
            print("解算失败:", str(e))
        
    def verify_solution(self, theta_list, target_pos):
        """验证逆解结果是否正确"""
        # 保存当前状态
        current_theta = self.theta_list.copy()
        
        # 使用解出的角度计算正运动学
        self.update_dh(theta_list)
        current_pos = self.forward_rm[:3, 3]
        
        # 恢复原状态
        self.update_dh(current_theta)
        
        # 计算误差
        error = np.linalg.norm(current_pos - target_pos)
        return error < 1e-3  # 允许的误差阈值
        