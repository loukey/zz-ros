from math import cos, sin, pi
import numpy as np
from scipy.spatial.transform import Rotation as R

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
