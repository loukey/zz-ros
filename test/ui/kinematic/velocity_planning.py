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
        v_max = np.sqrt(2 * a_acc * (total_distance - v_start**2 / (2 * a_acc)))
        
        # 计算加速时间（也是减速时间）
        t_acc_adj = (v_max - v_start) / a_acc
        t_dec_adj = v_max / a_dec
        
        # 总时间
        total_time = t_acc_adj + t_dec_adj
        
        # 三角形速度曲线
        d_const = 0
        t_const = 0
    else:
        # 匀速段的时间
        t_const = d_const / v_max
        
        # 总时间
        total_time = t_acc + t_const + t_acc  # 加速时间 + 匀速时间 + 减速时间
    
    # 创建时间序列
    num_steps = int(total_time / dt) + 1
    times = np.linspace(0, total_time, num_steps)
    
    # 初始化数组
    velocities = np.zeros(num_steps)
    accelerations = np.zeros(num_steps)
    positions = np.zeros((num_steps, angles.size if hasattr(angles, 'size') else 1))
    
    # 计算每个时间点的位置、速度和加速度
    for i, t in enumerate(times):
        # 加速阶段
        if t <= t_acc:
            a = a_acc
            v = v_start + a_acc * t
            p = v_start * t + 0.5 * a_acc * t**2
            
        # 匀速阶段
        elif t <= t_acc + t_const:
            a = 0
            v = v_max
            p = d_acc + v_max * (t - t_acc)
            
        # 减速阶段
        else:
            t_dec = t - (t_acc + t_const)  # 减速段已经过去的时间
            a = -a_dec
            v = v_max - a_dec * t_dec
            p = d_acc + d_const + v_max * t_dec - 0.5 * a_dec * t_dec**2
            
        velocities[i] = v
        accelerations[i] = a
        
        # 对每个关节进行位置调整
        if len(angles.shape) == 0:  # 单个值
            positions[i, 0] = angles * (p / total_distance) if total_distance != 0 else 0
        else:  # 多关节
            for j in range(angles.size):
                positions[i, j] = angles[j] * (p / total_distance) if total_distance != 0 else 0
    
    return times, velocities, accelerations, positions


"""
实现S曲线（七次多项式）速度规划

参数:
    angles: 目标关节角度，形状为(6,)的数组，表示六个关节的目标角度
    v_max: 最大速度
    t_acc: 加速时间，默认为0.5秒
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
    
    # 计算匀速段距离
    # S曲线加速段距离：(v_max + v_start) * t_acc / 2
    d_acc = (v_max + v_start) * t_acc / 2  # 加速距离
    d_dec = v_max * t_acc / 2  # 减速距离（假设减速段初始速度为v_max，结束速度为0）
    
    # 计算匀速阶段的距离
    d_const = total_distance - d_acc - d_dec
    
    # 如果没有足够的距离进行完整的加速和减速
    if d_const < 0:
        # 三角形速度曲线
        # 重新计算最大速度，使加速距离+减速距离=总距离
        # 解方程：(v_max + v_start) * t_acc / 2 + v_max * t_acc / 2 = total_distance
        v_max = (2 * total_distance - v_start * t_acc) / (2 * t_acc)
        
        # 匀速段时间为0
        t_const = 0
    else:
        # 匀速段的时间
        t_const = d_const / v_max
    
    # 总时间
    total_time = 2 * t_acc + t_const  # 加速时间 + 匀速时间 + 减速时间
    
    # 创建时间序列
    num_steps = int(total_time / dt) + 1
    times = np.linspace(0, total_time, num_steps)
    
    # 初始化数组
    velocities = np.zeros(num_steps)
    accelerations = np.zeros(num_steps)
    positions = np.zeros((num_steps, angles.size if hasattr(angles, 'size') else 1))
    
    # 计算每个时间点的位置、速度和加速度
    for i, t in enumerate(times):
        # 加速阶段 - 使用正弦加速曲线
        if t <= t_acc:
            # 正弦加速度曲线：a(t) = a_max * sin(pi*t/t_acc)
            # 最大加速度 a_max 与 v_max 的关系：v_max = a_max * t_acc / pi
            a_max = pi * (v_max - v_start) / t_acc
            a = a_max * np.sin(np.pi * t / t_acc) / 2
            
            # 速度：v(t) = v_start + a_max * t_acc / pi * (1 - cos(pi*t/t_acc))
            v = v_start + (v_max - v_start) * (1 - np.cos(np.pi * t / t_acc)) / 2
            
            # 位置：p(t) = v_start*t + a_max * t_acc^2 / pi^2 * (pi*t/t_acc - sin(pi*t/t_acc))
            p = v_start * t + (v_max - v_start) * (t - t_acc * np.sin(np.pi * t / t_acc) / np.pi) / 2
            
        # 匀速阶段
        elif t <= t_acc + t_const:
            a = 0
            v = v_max
            p = d_acc + v_max * (t - t_acc)
            
        # 减速阶段 - 使用正弦减速曲线
        else:
            t_dec = t - (t_acc + t_const)  # 减速段已经过去的时间
            
            # 类似加速段，但是加速度为负，初速度为v_max，终速度为0
            a_max = pi * v_max / t_acc
            a = -a_max * np.sin(np.pi * t_dec / t_acc) / 2
            
            # 速度
            v = v_max * (1 + np.cos(np.pi * t_dec / t_acc)) / 2
            
            # 位置
            p = d_acc + d_const + v_max * (t_dec - t_acc * np.sin(np.pi * t_dec / t_acc) / np.pi) / 2
            
        velocities[i] = v
        accelerations[i] = a
        
        # 对每个关节进行位置调整
        if len(angles.shape) == 0:  # 单个值
            positions[i, 0] = angles * (p / total_distance) if total_distance != 0 else 0
        else:  # 多关节
            for j in range(angles.size):
                positions[i, j] = angles[j] * (p / total_distance) if total_distance != 0 else 0
    
    return times, velocities, accelerations, positions 