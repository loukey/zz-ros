import numpy as np
from math import pi


"""
实现梯形加减速曲线规划

参数:
    angles: 目标关节角度，形状为(6,)的数组，表示六个关节的目标角度
    v_max: 最大角速度
    t_acc: 加速时间，默认为0.4秒
    v_start: 初始速度，默认为0
    dt: 采样间隔时间，默认为0.01秒

返回:
    times: 时间点序列
    velocities: 对应的速度序列
    accelerations: 对应的加速度序列
    positions: 对应的位置序列
"""
def trapezoidal_velocity_planning(angles, v_max=pi, t_acc=0.4, v_start=0, dt=0.01):
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
    
    # 根据固定加速时间计算加速度
    a_acc = (v_max - v_start) / t_acc  # 加速阶段的加速度
    a_dec = v_max / t_acc  # 减速阶段的加速度（绝对值）
    
    # 计算加速和减速阶段的距离
    d_acc = v_start * t_acc + 0.5 * a_acc * t_acc**2  # 加速距离
    d_dec = 0.5 * v_max * t_acc  # 减速距离
    
    # 计算匀速阶段的距离和时间
    d_const = total_distance - d_acc - d_dec
    
    # 如果没有足够的距离进行完整的加速和减速
    if d_const < 0:
        # 重新计算最大速度
        # 解方程：d_acc + d_dec = total_distance
        # 这里需要解一个二次方程
        # 假设加速和减速时间相等，都为t_acc
        # 则有：v_start*t_acc + 0.5*a*t_acc^2 + 0.5*v_peak*t_acc = total_distance
        # 其中 a = (v_peak - v_start)/t_acc
        # 代入得：v_start*t_acc + 0.5*(v_peak-v_start)*t_acc + 0.5*v_peak*t_acc = total_distance
        # 整理得：v_peak*t_acc = total_distance + 0.5*v_start*t_acc
        v_peak = (total_distance + 0.5 * v_start * t_acc) / t_acc
        a_acc = (v_peak - v_start) / t_acc
        a_dec = v_peak / t_acc
        t_const = 0
        d_const = 0
    else:
        v_peak = v_max
        t_const = d_const / v_max
    
    # 计算总时间
    total_time = t_acc + t_const + t_acc  # 注意减速时间也是t_acc
    
    # 根据采样间隔创建时间序列
    num_samples = int(np.ceil(total_time / dt)) + 1  # 采样点数量
    times = np.linspace(0, total_time, num_samples)
    velocities = np.zeros(num_samples)
    accelerations = np.zeros(num_samples)
    positions = np.zeros((num_samples, len(angles) if len(angles.shape) > 0 else 1))
    
    # 计算每个时间点的速度、加速度和位置
    for i, t in enumerate(times):
        if t <= t_acc:  # 加速阶段
            velocities[i] = v_start + a_acc * t
            accelerations[i] = a_acc
            positions[i] = (v_start * t + 0.5 * a_acc * t**2) / total_distance * angles
        elif t <= t_acc + t_const:  # 匀速阶段
            velocities[i] = v_peak
            accelerations[i] = 0
            positions[i] = (d_acc + v_peak * (t - t_acc)) / total_distance * angles
        else:  # 减速阶段
            t_in_dec = t - t_acc - t_const
            velocities[i] = v_peak - a_dec * t_in_dec
            accelerations[i] = -a_dec
            positions[i] = ((d_acc + d_const) + v_peak * t_in_dec - 0.5 * a_dec * t_in_dec**2) / total_distance * angles
    
    return times, velocities, accelerations, positions


"""
实现S型加减速曲线规划（使用正弦加速度曲线）

参数:
    angles: 目标关节角度，形状为(6,)的数组，表示六个关节的目标角度
    v_max: 最大角速度，默认为pi
    t_acc: 加速时间，默认为0.4秒
    v_start: 初始速度，默认为0
    dt: 采样间隔时间，默认为0.01秒

返回:
    times: 时间点序列
    velocities: 对应的速度序列
    accelerations: 对应的加速度序列
    positions: 对应的位置序列
"""
def s_curve_velocity_planning(angles, v_max=pi, t_acc=0.4, v_start=0, dt=0.01):
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
    
    # 计算S曲线的参数
    # 在S曲线中，加速度是平滑变化的，我们使用正弦函数来模拟
    # 最大加速度是梯形曲线的π/2倍，以保证相同时间内达到相同的速度
    a_max = np.pi/2 * (v_max - v_start) / t_acc
    
    # 计算加速和减速阶段的距离
    # 对于正弦加速度曲线，加速距离为：
    d_acc = v_start * t_acc + (v_max - v_start) * t_acc / 2
    d_dec = v_max * t_acc / 2  # 减速距离
    
    # 计算匀速阶段的距离和时间
    d_const = total_distance - d_acc - d_dec
    
    # 如果没有足够的距离进行完整的加速和减速
    if d_const < 0:
        # 重新计算最大速度
        # 对于S曲线，我们需要解一个更复杂的方程
        # 简化起见，我们使用二分法求解
        v_low = v_start
        v_high = v_max
        
        while v_high - v_low > 1e-6:
            v_mid = (v_low + v_high) / 2
            a_temp = np.pi/2 * (v_mid - v_start) / t_acc
            d_acc_temp = v_start * t_acc + (v_mid - v_start) * t_acc / 2
            d_dec_temp = v_mid * t_acc / 2
            d_total = d_acc_temp + d_dec_temp
            
            if d_total > total_distance:
                v_high = v_mid
            else:
                v_low = v_mid
        
        v_peak = v_low
        t_const = 0
        d_const = 0
    else:
        v_peak = v_max
        t_const = d_const / v_max
    
    # 计算总时间
    total_time = t_acc + t_const + t_acc
    
    # 根据采样间隔创建时间序列
    num_samples = int(np.ceil(total_time / dt)) + 1
    times = np.linspace(0, total_time, num_samples)
    velocities = np.zeros(num_samples)
    accelerations = np.zeros(num_samples)
    positions = np.zeros((num_samples, len(angles) if len(angles.shape) > 0 else 1))
    
    # 使用数值积分计算位置
    # 初始位置为0
    current_position = np.zeros(len(angles) if len(angles.shape) > 0 else 1)
    
    # 计算每个时间点的速度、加速度和位置
    for i, t in enumerate(times):
        if t <= t_acc:  # 加速阶段
            # 正弦加速度曲线
            a_temp = a_max * np.sin(np.pi * t / t_acc)
            velocities[i] = v_start + (v_peak - v_start) * (1 - np.cos(np.pi * t / t_acc)) / 2
            accelerations[i] = a_temp
        elif t <= t_acc + t_const:  # 匀速阶段
            velocities[i] = v_peak
            accelerations[i] = 0
        else:  # 减速阶段
            t_in_dec = t - t_acc - t_const
            # 正弦减速度曲线
            a_temp = -a_max * np.sin(np.pi * t_in_dec / t_acc)
            velocities[i] = v_peak - (v_peak) * (1 - np.cos(np.pi * t_in_dec / t_acc)) / 2
            accelerations[i] = a_temp
        
        # 如果不是第一个点，使用梯形积分法计算位置
        if i > 0:
            dt_actual = times[i] - times[i-1]
            # 梯形积分法：位移 = 上一个位置 + (当前速度 + 上一个速度) * dt / 2
            current_position = current_position + (velocities[i] + velocities[i-1]) * dt_actual / 2 * (angles / total_distance)
        
        # 存储当前位置
        positions[i] = current_position
    
    return times, velocities, accelerations, positions
