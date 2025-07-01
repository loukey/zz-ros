from math import cos, sin
import numpy as np


def dh_to_rotation_matrix(a, alpha, d, theta):
    c_theta = cos(theta)
    s_theta = sin(theta)
    c_alpha = cos(alpha)
    s_alpha = sin(alpha)
    
    return np.array([
        [c_theta, -s_theta, 0, a],
        [s_theta * c_alpha, c_theta * c_alpha, -s_alpha, -s_alpha * d],
        [s_theta * s_alpha, c_theta * s_alpha, c_alpha, c_alpha * d],
        [0, 0, 0, 1]
    ], dtype=float)
    