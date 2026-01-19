from functools import reduce
from math import atan2, acos, sin, cos, pi, sqrt
import numpy as np
from scipy.spatial.transform import Rotation as R
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo


from dataclasses import dataclass
import numpy as np
from math import pi

@dataclass
class DHParam:
    """机械臂 DH 参数配置。
    
    存储运动学和动力学所需的 DH 参数。
    
    Attributes:
        kinematic_dh (np.array): 运动学 DH 参数表。
        dynamic_dh (np.array): 动力学 DH 参数表。
    """
    
    kinematic_dh: np.ndarray
    dynamic_dh: np.ndarray

    def __init__(self):
        """初始化默认 DH 参数。"""
        self.kinematic_dh = np.array([
            [0.0, 0.0, 0.0,  0.0],
            [0.0, -pi/2, 0.0, -pi/2],
            [0.425, pi, 0.0, 0.0],
            [0.401, pi, 0.0856, pi/2],
            [0.0, pi/2, 0.086, 0.0],
            [0.0, -pi/2, 0.231, 0.0]  
        ], dtype=float)

        self.dynamic_dh = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, -pi/2, 0.0, -pi/2],
            [0.425, pi, 0.0, 0.0],
            [0.401, pi, 0.0856, pi/2],
            [0.0, pi/2, 0.086, 0.0],
            [0.0, -pi/2, 0.0725, 0.0]
        ])

    def get_kinematic_dh(self) -> np.ndarray:
        """获取运动学 DH 参数。
        
        Returns:
            np.ndarray: 运动学 DH 参数矩阵。
        """
        return self.kinematic_dh

    def get_dynamic_dh(self) -> np.ndarray:
        """获取动力学 DH 参数。
        
        Returns:
            np.ndarray: 动力学 DH 参数矩阵。
        """
        return self.dynamic_dh


class KinematicUtils:

    @staticmethod
    def dh2rm(a, alpha, d, theta):
        c_theta = cos(theta)
        s_theta = sin(theta)
        c_alpha = cos(alpha)
        s_alpha = sin(alpha)
    
        return np.array([
            [c_theta, -s_theta, 0, a],
            [s_theta * c_alpha, c_theta * c_alpha, -s_alpha, -s_alpha * d],
            [s_theta * s_alpha, c_theta * s_alpha, c_alpha, c_alpha * d],
            [0, 0, 0, 1]
        ], dtype=np.float64)

    @staticmethod
    def rm2quat(rm):
        return R.from_matrix(rm).as_quat()

    @staticmethod
    def quat2rm(quat):
        return R.from_quat(quat).as_matrix()

    @staticmethod
    def quat2euler(quat):
        return R.from_quat(quat).as_euler('xyz', degrees=False)

    @staticmethod
    def euler2quat(euler):  
        return R.from_euler('xyz', euler, degrees=False).as_quat()

    @staticmethod
    def euler2rm(euler):
        return R.from_euler('xyz', euler, degrees=False).as_matrix()

    @staticmethod
    def rm2euler(rm):
        return R.from_matrix(rm).as_euler('xyz', degrees=False)
    
    @staticmethod
    def normalize_angle(angle):
        return (angle + pi) % (2 * pi) - pi


# class KinematicDomainService:
#     def __init__(self):
#         self.kinematic_dh = np.array([
#             [0.0, 0.0, 0.0,  0.0],
#             [0.0, -pi/2, 0.0, -pi/2],
#             [0.425, pi, 0.0, 0.0],
#             [0.401, pi, 0.0856, pi/2],
#             [0.0, pi/2, 0.086, 0.0],
#             [0.0, -pi/2, 0.231, 0.0]  
#         ], dtype=float)
#         self.kinematic_utils = KinematicUtils()
#         self.gripper2base = self.get_gripper2base(self.kinematic_dh[:, 3])

#     def get_kinematic_dh(self):
#         return self.kinematic_dh

#     def get_gripper2base(self, theta_list=None):
#         if theta_list is not None:
#             self.kinematic_dh[:, 3] = theta_list
#             rm_list = []
#             for dh in self.kinematic_dh:
#                 rm_list.append(self.kinematic_utils.dh2rm(*dh))
#             self.gripper2base = reduce(lambda x, y: x @ y, rm_list, np.eye(4))
#         quat = R.from_matrix(self.gripper2base[:3, :3]).as_quat()
#         return quat, self.gripper2base[:3, 3]

#     def normalize_angle(self, angle):
#         return (angle + pi) % (2 * pi) - pi

#     def inverse_kinematic(self, rm, pos, initial_theta=None):
#         if not initial_theta:
#             initial_theta = self.kinematic_dh[:, 3]
#         valid_solutions = []
#         # rotation matrix
#         nx, ny, nz = rm[:, 0]
#         ox, oy, oz = rm[:, 1]
#         ax, ay, az = rm[:, 2]
#         px, py, pz = pos
        
#         # DH参数
#         a3 = 0.425 
#         a4 = 0.401 
#         d4 = 0.0856 
#         d5 = 0.086 
#         d6 = 0.231
        
#         # 添加奇异性判断
#         if abs(ax**2 + ay**2 - d4**2) < 1e-6:
#             return valid_solutions
        
#         # theta1计算
#         m = ay * d6 - py
#         n = ax * d6 - px
#         try:
#             delta = m**2 + n**2 - d4**2
            
#             if delta < 0:
#                 return valid_solutions
            
#             theta1_0 = atan2(m, n) - atan2(-d4, sqrt(m**2 + n**2 - d4**2))
#             theta1_1 = atan2(m, n) - atan2(-d4, -sqrt(m**2 + n**2 - d4**2))
#             theta1_candidate = [theta1_0, theta1_1]
                
#         except Exception as e:
#             return valid_solutions

#         # theta5
#         for i, theta1 in enumerate(theta1_candidate):
#             s1 = sin(theta1)
#             c1 = cos(theta1)
#             # 修改theta5的计算方式
#             cos_theta5 = -s1 * ax + c1 * ay
#             if abs(cos_theta5) > 1:
#                 cos_theta5 = 1 if cos_theta5 > 0 else -1
            
#             theta5_val = acos(cos_theta5)
#             theta5_candidate = [theta5_val, -theta5_val]

#             for j, theta5 in enumerate(theta5_candidate):
#                 s5 = sin(theta5)
#                 c5 = cos(theta5)
                
#                 # theta6
#                 if abs(s5) < 1e-6:
#                     # 腕部奇异性
#                     theta6 = initial_theta[5]
#                 else:
#                     m1 = -s1 * nx + c1 * ny
#                     n1 = -s1 * ox + c1 * oy
#                     theta6 = atan2(-n1 / s5, m1 / s5)
                
#                 # theta3
#                 s6 = sin(theta6)
#                 c6 = cos(theta6)
#                 m2 = d5 * (s6 * (nx * c1 + ny * s1) + c6 * (ox * c1 + oy * s1)) - d6 * (ax * c1 + ay * s1) + px * c1 + py * s1
#                 n2 = d5 * (oz * c6 + nz * s6) + pz - az * d6
                
#                 # 计算到关节3的距离
#                 r = sqrt(m2**2 + n2**2)
                
#                 if r > (a3 + a4) * 1.2 or r < abs(a3 - a4) * 0.8:  # 进一步放宽工作空间限制
#                     continue
#                 temp = (m2**2 + n2**2 - a3**2 - a4**2) / (2 * a3 * a4)
#                 if abs(temp) > 1:
#                     temp = 1 if temp > 0 else -1
#                 theta3_candidate = [acos(temp), -acos(temp)]
                
#                 for k, theta3 in enumerate(theta3_candidate):
#                     # theta2
#                     s3 = sin(theta3)
#                     c3 = cos(theta3)
#                     # 修改theta2的计算方式
#                     k1 = a3 + a4 * c3
#                     k2 = a4 * s3
#                     s2 = (k1 * n2 - k2 * m2) / (k1**2 + k2**2)
#                     c2 = (k1 * m2 + k2 * n2) / (k1**2 + k2**2)
#                     theta2 = -atan2(s2, c2)
        
#                     # theta4
#                     if abs(s5) < 1e-6:
#                         # 在腕部奇异性时，使用更稳定的计算方式
#                         if initial_theta is not None:
#                             theta4_candidate = [initial_theta[3]]
#                         else:
#                             theta4_candidate = [initial_theta[3]]
#                     else:
#                         T01 = self.kinematic_utils.dh2rm(self.kinematic_dh[0, 0], self.kinematic_dh[0, 1], self.kinematic_dh[0, 2], theta1)
#                         T12 = self.kinematic_utils.dh2rm(self.kinematic_dh[1, 0], self.kinematic_dh[1, 1], self.kinematic_dh[1, 2], theta2)
#                         T23 = self.kinematic_utils.dh2rm(self.kinematic_dh[2, 0], self.kinematic_dh[2, 1], self.kinematic_dh[2, 2], theta3)
#                         R03 = (T01 @ T12 @ T23)[:3, :3]
#                         R36 = R03.T @ rm
#                         theta4 = atan2(R36[1, 2], R36[0, 2])
#                         theta4_candidate = [-theta4, pi - theta4]
#                     for theta4 in theta4_candidate:
#                         candidate = [theta1, theta2, theta3, theta4, theta5, theta6]
#                         if self.verify_solution(candidate, (px, py, pz)):
#                             valid_solutions.append(candidate)
                            
#         if not valid_solutions:
#             raise ValueError("No valid solutions found")
#         valid_solutions = [
#             [self.kinematic_utils.normalize_angle(a) for a in sol] for sol in valid_solutions
#         ]
#         final_solution = min(valid_solutions, key=lambda sol: np.linalg.norm(np.array(sol) - np.array(initial_theta)))
#         return final_solution

#     def verify_solution(self, theta_list, target_pos):
#         self.get_gripper2base(theta_list)
#         current_pos = self.gripper2base[:3, 3]
#         error = np.linalg.norm(current_pos - target_pos)
        
#         return error < 1e-5 


class KinematicDomainService:
    """运动学领域服务。
    
    提供机械臂正逆运动学解算功能，包括DH参数管理、坐标变换矩阵计算等。
    
    Attributes:
        dh_param (DHParam): 机械臂 DH 参数对象。
        kinematic_dh (np.ndarray): 运动学 DH 参数矩阵。
        kinematic_utils (KinematicUtils): 运动学工具类实例。
        gripper2base (np.ndarray): 当前末端到基座的变换矩阵。
    """

    def __init__(self):
        """初始化运动学服务。"""
        self.dh_param = DHParam()
        self.kinematic_dh = self.dh_param.get_kinematic_dh()
        self.kinematic_utils = KinematicUtils()
        self.gripper2base = self.get_gripper2base(self.kinematic_dh[:, 3])

    def get_kinematic_dh(self) -> np.ndarray:
        """获取运动学 DH 参数矩阵。
        
        Returns:
            np.ndarray: DH 参数矩阵。
        """
        return self.kinematic_dh

    def get_gripper2base(self, theta_list: list[float] | np.ndarray = None) -> tuple[np.ndarray, np.ndarray]:
        """计算末端执行器相对于基座的位姿（四元数 + 位置）。
        
        Args:
            theta_list (list[float] | np.ndarray, optional): 关节角度列表（弧度）。如果不传则使用当前内部状态。
        
        Returns:
            tuple: (quat, pos)
                - quat (np.ndarray): 姿态四元数 [x, y, z, w]。
                - pos (np.ndarray): 位置坐标 [x, y, z]。
        """
        if theta_list is not None:
            self.kinematic_dh[:, 3] = theta_list
            rm_list = []
            for dh in self.kinematic_dh:
                rm_list.append(self.kinematic_utils.dh2rm(*dh))
            self.gripper2base = reduce(lambda x, y: x @ y, rm_list, np.eye(4))
        quat = R.from_matrix(self.gripper2base[:3, :3]).as_quat()
        return quat, self.gripper2base[:3, 3]
    
    def get_gripper2base_rm(self, theta_list: list[float]) -> np.ndarray:
        """获取末端执行器相对于基座的变换矩阵。
        
        Args:
            theta_list (list[float]): 关节角度列表（弧度）。
        
        Returns:
            np.ndarray: 4x4 齐次变换矩阵。
        """
        self.kinematic_dh[:, 3] = theta_list
        rm_list = []
        for dh in self.kinematic_dh:
            rm_list.append(self.kinematic_utils.dh2rm(*dh))
        self.gripper2base = reduce(lambda x, y: x @ y, rm_list, np.eye(4))
        return self.gripper2base

    def inverse_kinematic(self, rm: np.ndarray, pos: np.ndarray, initial_theta: list[float] | None = None) -> list[float]:
        """机械臂逆运动学求解。
        
        Args:
            rm (np.ndarray): 目标姿态旋转矩阵 (3x3)。
            pos (np.ndarray): 目标位置坐标 [x, y, z]。
            initial_theta (list[float] | None, optional): 初始关节角度猜测值，用于选择最优解。
        
        Returns:
            list[float]: 最优关节角度解（弧度）。
        
        Raises:
            ValueError: 如果未找到有效的逆运动学解。
        """
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
        d6 = 0.231
        
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
                    theta6 = initial_theta[5]
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
                        # 在腕部奇异性时，使用更稳定的计算方式
                        if initial_theta is not None:
                            theta4_candidate = [initial_theta[3]]
                        else:
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
        
        def angle_distance(a: float, b: float) -> float:
            """计算两个角度的最小差（考虑周期性），返回值在 [0, π]"""
            diff = (a - b + pi) % (2 * pi) - pi
            return abs(diff)
        
        def solution_distance(sol: list[float], ref: list[float]) -> float:
            """计算解与参考值的周期性角度距离"""
            return sum(angle_distance(s, r) ** 2 for s, r in zip(sol, ref)) ** 0.5
        
        def wrap_to_nearest(target: float, reference: float) -> float:
            """将 target 调整到与 reference 最接近的等效角度（加减 2π 的整数倍）"""
            diff = (target - reference + pi) % (2 * pi) - pi
            return reference + diff
        for s in valid_solutions:
            print(s)
        # 选择与 initial_theta 周期性距离最近的解
        final_solution = min(valid_solutions, key=lambda sol: solution_distance(sol, initial_theta))
        
        # 将每个关节角度调整到与 initial_theta 最接近的等效值
        final_solution = [wrap_to_nearest(f, i) for f, i in zip(final_solution, initial_theta)]

        return final_solution

    def verify_solution(self, theta_list: list[float], target_pos: np.ndarray) -> bool:
        """验证逆运动学解的准确性。
        
        Args:
            theta_list (list[float]): 待验证的关节角度解。
            target_pos (np.ndarray): 目标位置坐标。
        
        Returns:
            bool: 如果正解位置与目标位置误差小于 1e-5 则返回 True。
        """
        self.get_gripper2base(theta_list)
        current_pos = self.gripper2base[:3, 3]
        error = np.linalg.norm(current_pos - target_pos)
        
        return error < 1e-5 


'''
[ 0.63116763  0.31389901  0.63909985 -0.30764626] | [ 0.55131439 -0.02752803  0.25536895] | 
[-0.6607127065726779, -1.5615613872776202, -2.0046245686036, 1.1176222192498209, 0.006190027358587147, -3.144355027200458] | 
[-0.6580086573742605, -1.5751421999756665, -1.9246109974502614, 1.6246749437105255, 0.006729766488388356, -3.5578217364827145] 

[ 0.63116763  0.31389901  0.63909985 -0.30764626] | [ 0.54931439 -0.02752803  0.25536895] | 
[-0.6634371066802967, -1.2265668462674435, -1.506571930932049, -2.273867274272604, -0.006788204299675582, -5.872984312539139] | 
[-0.6607127065726779, -1.5615613872776202, -2.0046245686036, 1.1176222192498209, 0.006190027358587147, -3.144355027200458] 
'''


kinematic_domain_service = KinematicDomainService()
kinematic_utils = KinematicUtils()
quat = [0.63116763, 0.31389901, 0.63909985, -0.30764626]
pos = [0.55528, -0.02752803, 0.25536895]
nearest_position = [
-0.6553247556660113, -1.5556075123873467, -1.8462758433296393, 1.9905958255326244, 0.008165271828196874, -3.8649504581211156
    ]
rm = kinematic_utils.quat2rm(quat)
position = kinematic_domain_service.inverse_kinematic(rm, pos, nearest_position)
print(position)
