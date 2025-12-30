from math import cos, sin, pi
import numpy as np
from scipy.spatial.transform import Rotation as R

class KinematicUtils:
    """运动学工具类。
    
    提供基本的运动学变换和角度处理函数。
    """

    @staticmethod
    def dh2rm(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """根据 DH 参数计算变换矩阵。
        
        Args:
            a (float): 连杆长度。
            alpha (float): 连杆扭转角。
            d (float): 连杆偏移。
            theta (float): 关节角。
            
        Returns:
            np.ndarray: 4x4 齐次变换矩阵。
        """
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
    def rm2quat(rm: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数。
        
        Args:
            rm (np.ndarray): 3x3 旋转矩阵或 4x4 变换矩阵。
            
        Returns:
            np.ndarray: 四元数 [x, y, z, w]。
        """
        # 如果是 4x4 矩阵，提取 3x3 部分
        if rm.shape == (4, 4):
            rm = rm[:3, :3]
        return R.from_matrix(rm).as_quat()

    @staticmethod
    def quat2rm(quat: np.ndarray) -> np.ndarray:
        """四元数转旋转矩阵。
        
        Args:
            quat (np.ndarray): 四元数 [x, y, z, w]。
            
        Returns:
            np.ndarray: 3x3 旋转矩阵。
        """
        return R.from_quat(quat).as_matrix()

    @staticmethod
    def quat2euler(quat: np.ndarray) -> np.ndarray:
        """四元数转欧拉角 (XYZ 顺序)。
        
        Args:
            quat (np.ndarray): 四元数 [x, y, z, w]。
            
        Returns:
            np.ndarray: 欧拉角 [roll, pitch, yaw] (弧度)。
        """
        return R.from_quat(quat).as_euler('xyz', degrees=False)

    @staticmethod
    def euler2quat(euler: np.ndarray) -> np.ndarray:
        """欧拉角转四元数 (XYZ 顺序)。
        
        Args:
            euler (np.ndarray): 欧拉角 [roll, pitch, yaw] (弧度)。
            
        Returns:
            np.ndarray: 四元数 [x, y, z, w]。
        """
        return R.from_euler('xyz', euler, degrees=False).as_quat()

    @staticmethod
    def euler2rm(euler: np.ndarray) -> np.ndarray:
        """欧拉角转旋转矩阵 (XYZ 顺序)。
        
        Args:
            euler (np.ndarray): 欧拉角 [roll, pitch, yaw] (弧度)。
            
        Returns:
            np.ndarray: 3x3 旋转矩阵。
        """
        return R.from_euler('xyz', euler, degrees=False).as_matrix()

    @staticmethod
    def rm2euler(rm: np.ndarray) -> np.ndarray:
        """旋转矩阵转欧拉角 (XYZ 顺序)。
        
        Args:
            rm (np.ndarray): 3x3 旋转矩阵。
            
        Returns:
            np.ndarray: 欧拉角 [roll, pitch, yaw] (弧度)。
        """
        # 如果是 4x4 矩阵，提取 3x3 部分
        if rm.shape == (4, 4):
            rm = rm[:3, :3]
        return R.from_matrix(rm).as_euler('xyz', degrees=False)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """将角度归一化到 [-pi, pi] 范围。
        
        Args:
            angle (float): 输入角度（弧度）。
            
        Returns:
            float: 归一化后的角度（弧度）。
        """
        return (angle + pi) % (2 * pi) - pi
