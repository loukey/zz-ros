import numpy as np

# 假设机械臂有6个关节
n = 6

# 机械臂每个连杆的质量（单位：kg）
link_masses = [4.64,10.755,3.925,1.24,1.24,2.549]  # 需替换为实际数值 4.64,10.755,3.925,1.24,1.24,2.549

# 每个连杆质心在本体坐标系下的位置（单位：m，3维向量）
link_com_positions = [
    np.array([0, 0, 0]),  # 连杆1质心
    np.array([0.2125, 0, 0.134]),  # 连杆2质心
    np.array([0.1818, 0, 0]),  # ...
    np.array([0, 0, 0]),
    np.array([0, 0, 0]),
    np.array([0, 0, 0.0745])
]  # 需替换为实际数值

# 重力加速度向量（假设z轴向下）
g_vector = np.array([0, 0, -9.81])

# 机械臂当前关节角度（单位：弧度）
q = np.array([np.deg2rad(0), np.deg2rad(30), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)])  # 实时读取

dh_params = [
    # (a, alpha, d, theta_offset)
    (    0,               0,        0,               0),      # 关节1
    (    0, np.deg2rad(-90),        0, np.deg2rad(-90)),      # 关节2
    (0.425,               0,        0,              0) ,      # 关节3
    (0.401,               0,    0.0856, np.deg2rad(90)),
    (    0,  np.deg2rad(90),    0.086,              0) ,
    (    0, np.deg2rad(-90),   0.0725,              0)
]



def dh_transform(a, alpha, d, theta):
    """
    计算单个DH参数的齐次变换矩阵
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct,    -st,      0,    a],
        [st*ca,ct*ca,   -sa,-sa*d],
        [st*sa,ct*sa,    ca, ca*d],
        [0,       0,      0,    1]
    ])

def get_link_com_in_base(q, dh_params, link_com_positions, link_index):
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
        T_i = dh_transform(a, alpha, d, theta)
        T = np.dot(T, T_i)  # 累乘变换矩阵

    # 2. 质心在本体坐标系下的齐次坐标
    com_local = np.append(link_com_positions[link_index], 1)  # [x, y, z, 1]

    # 3. 变换到基坐标系
    com_base = np.dot(T, com_local)  # [x, y, z, 1]
    return com_base[:3]  # 返回前三个分量

# 2. 计算每个连杆质心对每个关节的雅可比矩阵的转动轴
def get_jacobian_column(q, link_index, com_base, joint_index):
    """
    输入：q为当前关节角度，link_index为连杆编号，com_world为该连杆质心在基坐标系下的位置，joint_index为关节编号
    输出：该质心对joint_index关节的雅可比列（3维向量）
    """
    # 计算joint_index关节的转动轴在基坐标系下的方向
    z_axis = get_joint_axis_in_base(q, dh_params, joint_index)  # 3维向量
    # 计算joint_index关节在基坐标系下的位置
    joint_pos = get_joint_position_in_base(q,dh_params,  joint_index)  # 3维向量
    # 计算质心相对该关节的矢量
    r = com_base - joint_pos
    # 旋转关节的雅可比列为 z_axis × r
    return np.cross(z_axis, r)

# 3. 计算重力补偿力矩
def compute_gravity_compensation(q):
    """
    输入：q为当前关节角度
    输出：每个关节的重力补偿力矩（n维向量）
    """
    tau_g = np.zeros(n)  # 初始化补偿力矩
    for i in range(n):  # 对每个关节
        for j in range(i, n):  # 对每个后续连杆（含自身）
            # 1. 计算第j连杆质心在基坐标系下的位置
            com_base = get_link_com_in_base(q,dh_params, link_com_positions,j)
            # 2. 计算该质心对第i关节的雅可比列
            J_col = get_jacobian_column(q, j, com_base, i)
            # 3. 计算重力对该关节的力矩贡献
            tau_g[i] += link_masses[j] * g_vector.dot(J_col)
    return tau_g

def get_joint_axis_in_base(q, dh_params, joint_index):
    """
    返回第joint_index个关节的z轴（转动轴）在基坐标系下的方向（3维向量）
    """
    # 从基坐标系累乘到joint_index的变换矩阵
    T = np.eye(4)
    for i in range(joint_index+1):
        a, alpha, d, theta_offset = dh_params[i]
        theta = q[i] + theta_offset
        T = np.dot(T, dh_transform(a, alpha, d, theta))
    # 关节z轴在本体坐标系下为[0, 0, 1]
    z_axis_local = np.array([0, 0, 1, 0])  # 齐次向量，最后一位0表示方向向量
    z_axis_base = np.dot(T, z_axis_local)[:3]
    return z_axis_base

def get_joint_position_in_base(q, dh_params, joint_index):
    """
    返回第joint_index个关节的原点在基坐标系下的位置（3维向量）
    """
    # 从基坐标系累乘到joint_index的变换矩阵
    T = np.eye(4)
    for i in range(joint_index+1):
        a, alpha, d, theta_offset = dh_params[i]
        theta = q[i] + theta_offset
        T = np.dot(T, dh_transform(a, alpha, d, theta))
    # 原点在本体坐标系下为[0, 0, 0, 1]
    origin_local = np.array([0, 0, 0, 1])
    origin_base = np.dot(T, origin_local)[:3]
    return origin_base

# 4. 主循环：实时计算并输出补偿力矩

    # 实时读取当前关节角度q
    # q = read_joint_positions()
    # 计算重力补偿力矩
tau_g = compute_gravity_compensation(q)
print(tau_g)
    # 输出到电机控制器
    # send_torque_commands(tau_g)
    # # 等待下一个控制周期
    # wait_for_next_cycle()







# ====== 示例用法 ======
# 假设有3个关节
# dh_params = [
#     # (a, alpha, d, theta_offset)
#     (0,      0, 0.3, 0),      # 关节1
#     (0.2,    0,   0, 0),      # 关节2
#     (0.2,    0,   0, 0)       # 关节3
# ]
# link_com_positions = [
#     np.array([0, 0, 0.15]),   # 连杆1质心
#     np.array([0.1, 0, 0]),    # 连杆2质心
#     np.array([0.1, 0, 0])     # 连杆3质心
# ]
# q = [np.deg2rad(30), np.deg2rad(45), np.deg2rad(60)]  # 当前关节角度
#
# # 计算第2根连杆（编号1）的质心在基坐标系下的位置
# com_base = get_link_com_in_base(q, dh_params, link_com_positions, 1)
# print("连杆2质心在基坐标系下的位置:", com_base)

