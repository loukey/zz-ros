from .base import *
from functools import reduce
from math import acos, degrees
import math


class Kinematic6DOF:
    def __init__(self, theta_list=[0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]):
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
        self.theta_list = theta_list
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
        for dh_matrix in self.DH_matrix:
            self.rm_list.append(dh_to_rotation_matrix(*dh_matrix))

    """
    正运动学求解变换矩阵
    """
    def forward_kinematic(self):
        return reduce(lambda x, y: x @ y, self.rm_list, np.eye(4))
    
    def update_dh(self, theta_list):
        self.DH_matrix[:, 3] = theta_list
        self.get_rotation_matrix_list()
        self.forward_rm = self.forward_kinematic()

    @staticmethod
    def normalize_angle(angle):
        return (angle + pi) % (2 * pi) - pi

    """
    逆运动学求解关节角度
    """
    def inverse_kinematic(self, A, B, C, px, py, pz, initial_theta=None):
        valid_solutions = []
        # rotation matrix
        rm = XYZ_euler_angles_to_rotation_matrix(A, B, C)
        nx, ny, nz = rm[:, 0]
        ox, oy, oz = rm[:, 1]
        ax, ay, az = rm[:, 2]
        a2, a3 = self.DH_matrix[2:4, 0]
        d1, _, _, d4, d5, d6 = self.DH_matrix[:, 2]

        # 添加奇异性判断
        if abs(ax**2 + ay**2 - d4**2) < 1e-6:
            print("Warning: Approaching shoulder singularity!")
            return valid_solutions
        
        # theta1计算
        m = ay * d6 - py
        n = ax * d6 - px
        try:
            delta = m**2 + n**2 - d4**2
            if delta < 0:
                print("Warning: Position unreachable - out of workspace")
                return valid_solutions
            theta1_0 = atan2(m, n) - atan2(-d4, sqrt(m**2 + n**2 - d4**2))
            theta1_1 = atan2(m, n) - atan2(-d4, -sqrt(m**2 + n**2 - d4**2))
            theta1_candidate = [theta1_0, theta1_1]
        except Exception as e:
            theta1_candidate = []

        # theta5
        for theta1 in theta1_candidate:
            s1 = sin(theta1)
            c1 = cos(theta1)
            try:
                theta5_val = acos(-s1 * ax + c1 * ay)
                theta5_candidate = [theta5_val, -theta5_val]

                for theta5 in theta5_candidate:
                    s5 = sin(theta5)
                    c5 = cos(theta5)
                    # theta6
                    if abs(s5) < 1e-6:
                        # 腕部奇异性
                        theta6 = self.theta_list[5]
                    else:
                        m1 = -s1 * nx + c1 * ny
                        n1 = -s1 * ox + c1 * oy
                        theta6 = atan2(-n1 / s5, m1 / s5)
                    # theta3
                    s6 = sin(theta6)
                    c6 = cos(theta6)
                    m2 = d5 * (s6 * (nx * c1 + ny * s1) + c6 * (ox * c1 + oy * s1)) - d6 * (ax * c1 + ay * s1) + px * c1 + py * s1
                    n2 = d5 * (oz * c6 + nz * s6) + pz - d1 - az * d6
                    temp = (m2**2 + n2**2 - a2**2 - a3**2) / (2 * a2 * a3)
                    if temp < -1 or temp > 1:
                        continue
                    theta3_candidate = [acos(temp), -acos(temp)]
                    for theta3 in theta3_candidate:
                        # theta2
                        s3 = sin(theta3)
                        c3 = cos(theta3)
                        s2 = (-(a3 * c3 + a2) * n2 - a3 * s3 * m2) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * c3)
                        c2 = (m2 + a3 * s3 * s2) / (a3 * c3 + a2)
                        theta2 = atan2(s2, c2)
            
                        # theta4
                        c234 = c5 * (c6 * (nx * c1 + ny * s1) - s6 * (ox * c1 + oy * s1)) - s5 * (ax * c1 + ay * s1)
                        s234 = -s6 * (nx * c1 + ny * s1) - c6 * (ox * c1 + oy * s1)
                        theta4 = atan2(s234, c234) - theta2 - theta3

                        candidate = [self.normalize_angle(theta) for theta in [theta1, theta2, theta3, theta4, theta5, theta6]]
                        if self.verify_solution(candidate, (px, py, pz)):
                            valid_solutions.append(candidate)
            except Exception as e:
                continue
        if not valid_solutions:
            raise ValueError("No valid solutions found")
        final_solution = min(valid_solutions, key=lambda sol: np.linalg.norm(np.array(sol) - np.array(self.theta_list)))
        final_solution[2:6] = [-x for x in final_solution[2:6]]
        print("最终解:", final_solution)
        return final_solution
        
    """
    test kinematic
    """
    def test_kinematic(self, theta_list):
        print("测试角度:", [degrees(t) for t in theta_list])
        self.theta_list = theta_list
        self.update_dh(self.theta_list)
        T06 = self.forward_kinematic()
        print(T06)
        target_pos = T06[:3, 3]
        target_rm = T06[:3, :3]
        A, B, C = XYZ_rotation_matrix_to_euler_angles(target_rm)
        print("Euler Angles: {} {} {}".format(A, B, C))
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
        