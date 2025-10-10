from ...value_objects import DHParam
from ...utils import KinematicUtils
from functools import reduce
from math import atan2, acos, sin, cos, pi, sqrt
import numpy as np
from scipy.spatial.transform import Rotation as R


class KinematicDomainService:
    def __init__(self):
        self.dh_param = DHParam()
        self.kinematic_dh = self.dh_param.get_kinematic_dh()
        self.kinematic_utils = KinematicUtils()
        self.gripper2base = self.get_gripper2base(self.kinematic_dh[:, 3])

    def get_kinematic_dh(self):
        return self.kinematic_dh

    def get_gripper2base(self, theta_list=None):
        if theta_list is not None:
            self.kinematic_dh[:, 3] = theta_list
            rm_list = []
            for dh in self.kinematic_dh:
                rm_list.append(self.kinematic_utils.dh2rm(*dh))
            self.gripper2base = reduce(lambda x, y: x @ y, rm_list, np.eye(4))
        quat = R.from_matrix(self.gripper2base[:3, :3]).as_quat()
        return quat, self.gripper2base[:3, 3]

    def inverse_kinematic(self, rm, pos, initial_theta=None):
        if not initial_theta:
            initial_theta = self.kinematic_dh[:, 3]
        valid_solutions = []
        # rotation matrix
        nx, ny, nz = rm[:, 0]
        ox, oy, oz = rm[:, 1]
        ax, ay, az = rm[:, 2]
        px, py, pz = pos
        
        # DH参数
        a3 = 0.425 
        a4 = 0.401 
        d4 = 0.0856 
        d5 = 0.086 
        d6 = 0.2785
        
        # 添加奇异性判断
        if abs(ax**2 + ay**2 - d4**2) < 1e-6:
            return valid_solutions
        
        # theta1计算
        m = ay * d6 - py
        n = ax * d6 - px
        try:
            delta = m**2 + n**2 - d4**2
            
            if delta < 0:
                return valid_solutions
            
            theta1_0 = atan2(m, n) - atan2(-d4, sqrt(m**2 + n**2 - d4**2))
            theta1_1 = atan2(m, n) - atan2(-d4, -sqrt(m**2 + n**2 - d4**2))
            theta1_candidate = [theta1_0, theta1_1]
                
        except Exception as e:
            return valid_solutions

        # theta5
        for i, theta1 in enumerate(theta1_candidate):
            s1 = sin(theta1)
            c1 = cos(theta1)
            # 修改theta5的计算方式
            cos_theta5 = -s1 * ax + c1 * ay
            if abs(cos_theta5) > 1:
                cos_theta5 = 1 if cos_theta5 > 0 else -1
            
            theta5_val = acos(cos_theta5)
            theta5_candidate = [theta5_val, -theta5_val]

            for j, theta5 in enumerate(theta5_candidate):
                s5 = sin(theta5)
                c5 = cos(theta5)
                
                # theta6
                if abs(s5) < 1e-6:
                    # 腕部奇异性
                    theta6 = self.kinematic_dh[5, 3]
                else:
                    m1 = -s1 * nx + c1 * ny
                    n1 = -s1 * ox + c1 * oy
                    theta6 = atan2(-n1 / s5, m1 / s5)
                
                # theta3
                s6 = sin(theta6)
                c6 = cos(theta6)
                m2 = d5 * (s6 * (nx * c1 + ny * s1) + c6 * (ox * c1 + oy * s1)) - d6 * (ax * c1 + ay * s1) + px * c1 + py * s1
                n2 = d5 * (oz * c6 + nz * s6) + pz - az * d6
                
                # 计算到关节3的距离
                r = sqrt(m2**2 + n2**2)
                
                if r > (a3 + a4) * 1.2 or r < abs(a3 - a4) * 0.8:  # 进一步放宽工作空间限制
                    continue
                temp = (m2**2 + n2**2 - a3**2 - a4**2) / (2 * a3 * a4)
                if abs(temp) > 1:
                    temp = 1 if temp > 0 else -1
                theta3_candidate = [acos(temp), -acos(temp)]
                
                for k, theta3 in enumerate(theta3_candidate):
                    # theta2
                    s3 = sin(theta3)
                    c3 = cos(theta3)
                    # 修改theta2的计算方式
                    k1 = a3 + a4 * c3
                    k2 = a4 * s3
                    s2 = (k1 * n2 - k2 * m2) / (k1**2 + k2**2)
                    c2 = (k1 * m2 + k2 * n2) / (k1**2 + k2**2)
                    theta2 = -atan2(s2, c2)
        
                    # theta4
                    if abs(s5) < 1e-6:
                        theta4_candidate = [initial_theta[3]]
                    else:
                        T01 = self.kinematic_utils.dh2rm(self.kinematic_dh[0, 0], self.kinematic_dh[0, 1], self.kinematic_dh[0, 2], theta1)
                        T12 = self.kinematic_utils.dh2rm(self.kinematic_dh[1, 0], self.kinematic_dh[1, 1], self.kinematic_dh[1, 2], theta2)
                        T23 = self.kinematic_utils.dh2rm(self.kinematic_dh[2, 0], self.kinematic_dh[2, 1], self.kinematic_dh[2, 2], theta3)
                        R03 = (T01 @ T12 @ T23)[:3, :3]
                        R36 = R03.T @ rm
                        theta4 = atan2(R36[1, 2], R36[0, 2])
                        theta4_candidate = [-theta4, pi - theta4]
                    for theta4 in theta4_candidate:
                        candidate = [theta1, theta2, theta3, theta4, theta5, theta6]
                        if self.verify_solution(candidate, (px, py, pz)):
                            valid_solutions.append(candidate)
                            
        if not valid_solutions:
            raise ValueError("No valid solutions found")
            
        final_solution = min(valid_solutions, key=lambda sol: np.linalg.norm(np.array(sol) - np.array(initial_theta)))
        return final_solution

    def verify_solution(self, theta_list, target_pos):
        self.get_gripper2base(theta_list)
        current_pos = self.gripper2base[:3, 3]
        error = np.linalg.norm(current_pos - target_pos)
        
        return error < 1e-5 
