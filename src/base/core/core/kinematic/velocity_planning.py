import numpy as np


"""
实现梯形加减速曲线规划

参数:
    angles: 目标关节角度，形状为(6,)的数组，表示六个关节的目标角度
    a_max: 最大角加速度
    v_max: 最大角速度
    v_start: 初始速度，默认为0

返回:
    times: 时间点序列
    velocities: 对应的速度序列
    accelerations: 对应的加速度序列
    positions: 对应的位置序列
"""
def trapezoidal_velocity_planning(angles, a_max, v_max, v_start=0):
    # 确保angles是numpy数组
    angles = np.array(angles)
    
    # 假设当前位置为零点，计算到目标位置的距离
    # 对于多关节，取最大角度变化作为规划依据
    if len(angles.shape) == 0:  # 单个值
        total_distance = np.abs(angles)
    elif len(angles) == 1:  # 单关节
        total_distance = np.abs(angles[0])
    else:  # 多关节
        total_distance = np.max(np.abs(angles))
    
    # 如果总距离为0，则返回空结果
    if total_distance == 0:
        return np.array([0]), np.array([0]), np.array([0]), np.zeros_like(angles).reshape(1, -1)
    
    # 计算加速和减速所需的时间
    t_acc = (v_max - v_start) / a_max  # 加速时间
    t_dec = v_max / a_max  # 减速时间
    
    # 计算加速和减速阶段的距离
    d_acc = v_start * t_acc + 0.5 * a_max * t_acc**2  # 加速距离
    d_dec = 0.5 * v_max * t_dec  # 减速距离
    
    # 计算匀速阶段的距离和时间
    d_const = total_distance - d_acc - d_dec
    
    # 如果没有足够的距离进行完整的加速和减速
    if d_const < 0:
        # 重新计算最大速度
        # 解方程：d_acc + d_dec = total_distance
        # v_peak是能达到的最大速度
        v_peak = np.sqrt(a_max * total_distance + 0.5 * v_start**2)
        t_acc = (v_peak - v_start) / a_max
        t_dec = v_peak / a_max
        t_const = 0
        d_const = 0
    else:
        v_peak = v_max
        t_const = d_const / v_max
    
    # 计算总时间
    total_time = t_acc + t_const + t_dec
    
    # 创建时间序列（采样更多的点以获得平滑的轨迹）
    num_samples = 100  # 采样点数量
    times = np.linspace(0, total_time, num_samples)
    velocities = np.zeros(num_samples)
    accelerations = np.zeros(num_samples)
    positions = np.zeros((num_samples, len(angles) if len(angles.shape) > 0 else 1))
    
    # 计算每个时间点的速度、加速度和位置
    for i, t in enumerate(times):
        if t <= t_acc:  # 加速阶段
            velocities[i] = v_start + a_max * t
            accelerations[i] = a_max
            positions[i] = (v_start * t + 0.5 * a_max * t**2) / total_distance * angles
        elif t <= t_acc + t_const:  # 匀速阶段
            velocities[i] = v_peak
            accelerations[i] = 0
            positions[i] = (d_acc + v_peak * (t - t_acc)) / total_distance * angles
        else:  # 减速阶段
            t_in_dec = t - t_acc - t_const
            velocities[i] = v_peak - a_max * t_in_dec
            accelerations[i] = -a_max
            positions[i] = ((d_acc + d_const) + v_peak * t_in_dec - 0.5 * a_max * t_in_dec**2) / total_distance * angles
    
    return times, velocities, accelerations, positions
