import numpy as np
from math import sin, cos, pi, atan2, sqrt, asin


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

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    Quaternion should be in the format [x, y, z, w].
    """
    x, y, z, w = quaternion

    # 计算 roll (x-axis rotation)
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)

    # 计算 pitch (y-axis rotation)
    t2 = 2.0 * (w * y - z * x)
    # 防止由于数值误差导致 t2 超出[-1, 1]范围
    t2 = max(min(t2, 1.0), -1.0)
    pitch = asin(t2)

    # 计算 yaw (z-axis rotation)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)

    return roll, pitch, yaw

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q