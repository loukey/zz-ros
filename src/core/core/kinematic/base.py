import numpy as np
from math import sin, cos, pi, atan2, sqrt


#XYZ/ZYX Euler Angles(rotation metrix to euler angles)
def XYZ_rotation_matrix_to_euler_angles(rotation_matrix):
    A = 0.0
    B = 0.0
    C = 0.0
    if abs(rotation_matrix[2, 0] >= 1.0 - 0.0001):
        if (rotation_matrix[2, 0] < 0):
            A = 0.0
            B = pi / 2
            C = atan2(rotation_matrix[0, 1], rotation_matrix[1, 1])
        else:
            A = 0.0
            B = -pi / 2
            C = atan2(rotation_matrix[0, 1], rotation_matrix[1, 1])

    else:
        B = atan2(-rotation_matrix[2, 0], sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2))
        cb = cos(B)
        A = atan2(rotation_matrix[1, 0] / cb, rotation_matrix[0, 0] / cb)
        C = atan2(rotation_matrix[2, 1] / cb, rotation_matrix[2, 2] / cb)
    
    return A, B, C

def XYZ_euler_angles_to_rotation_matrix(A, B, C):
    ca = cos(A)
    cb = cos(B)
    cc = cos(C)
    sa = sin(A)
    sb = sin(B)
    sc = sin(C)

    return np.array([
        [ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc],
        [sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc],
        [-sb, cb * sc, cb * cc]
    ])

"""
    DH矩阵->变换矩阵
    [ cθᵢ          -sθᵢ          0           aᵢ₋₁            ]
    [ sθᵢ cαᵢ₋₁    cθᵢ cαᵢ₋₁    -sαᵢ₋₁       -sαᵢ₋₁ dᵢ       ]
    [ sθᵢ sαᵢ₋₁    cθᵢ sαᵢ₋₁    cαᵢ₋₁        cαᵢ₋₁ dᵢ        ]
    [ 0            0            0            1               ]
"""
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
