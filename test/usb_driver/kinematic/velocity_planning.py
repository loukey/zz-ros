"""
速度规划模块
实现梯形和S形速度规划算法
"""
import numpy as np
from math import pi

def trapezoidal_velocity_planning(start_angles, target_angles, v_max=[pi/4] * 6, t_acc=[2] * 6, v_start=[0] * 6, dt=0.1):
    """实现梯形加减速曲线规划，专门用于6关节机器人
    
    Args:
        start_angles: 起始关节角度（弧度），形状为(6,)的数组，表示六个关节的起始角度
        target_angles: 目标关节角度（弧度），形状为(6,)的数组，表示六个关节的目标角度
        v_max: 每个关节的最大角速度（弧度/秒），默认为[pi/4] * 6
        t_acc: 每个关节的加速时间（秒），默认为[8] * 6
        v_start: 每个关节的初始速度（弧度/秒），默认为[0] * 6
        dt: 采样间隔时间（秒），默认为0.1秒
    
    Returns:
        times: 时间点序列
        velocities: 对应的速度序列，形状为(num_samples, 6)
        accelerations: 对应的加速度序列，形状为(num_samples, 6)
        positions: 对应的位置序列，形状为(num_samples, 6)
    """
    # 将输入转换为numpy数组以便计算
    start_angles = np.array(start_angles)  # 弧度
    target_angles = np.array(target_angles)  # 弧度
    v_max = np.array(v_max)    # 弧度/秒
    t_acc = np.array(t_acc)    # 秒
    v_start = np.array(v_start)  # 弧度/秒
    
    # 计算每个关节的位移（弧度）
    displacements = target_angles - start_angles
    
    # 找出最大位移，这将决定总运动时间
    max_displacement = np.max(np.abs(displacements))
    max_displacement_idx = np.argmax(np.abs(displacements))
    
    # 使用最大位移关节的参数计算总时间
    max_v_max = v_max[max_displacement_idx]
    max_t_acc = t_acc[max_displacement_idx]
    max_v_start = v_start[max_displacement_idx]
    
    # 计算最大位移关节的加速和减速阶段位移（弧度）
    acc_displacement = max_v_start * max_t_acc + 0.5 * (max_v_max / max_t_acc) * max_t_acc**2
    dec_displacement = max_v_max * max_t_acc + 0.5 * (-max_v_max / max_t_acc) * max_t_acc**2
    
    # 计算匀速阶段的位移（弧度）
    const_displacement = max_displacement - acc_displacement - dec_displacement
    
    # 计算匀速阶段的时间（秒）
    t_const = np.where(const_displacement > 0,
                      const_displacement / max_v_max,
                      0)
    
    # 计算总时间（秒）
    target_time = 2 * max_t_acc + t_const
    
    # 计算采样点数
    num_samples = int(np.ceil(target_time / dt))
    times = np.linspace(0, target_time, num_samples)
    
    # 初始化返回数组
    velocities = np.zeros((num_samples, 6))  # 弧度/秒
    accelerations = np.zeros((num_samples, 6))  # 弧度/秒²
    positions = np.zeros((num_samples, 6))  # 弧度
    
    # 计算每个关节的运动曲线
    for joint in range(6):
        # 获取当前关节的参数
        curr_v_max = v_max[joint]
        curr_t_acc = t_acc[joint]
        curr_v_start = v_start[joint]
        curr_displacement = displacements[joint]  # 注意这里使用实际位移（带符号）
        curr_start_angle = start_angles[joint]
        curr_target_angle = target_angles[joint]
        
        # 计算当前关节的总位移（弧度）
        total_displacement = abs(curr_displacement)
        
        # 计算加速和减速阶段的位移（弧度）
        acc_displacement = curr_v_start * curr_t_acc + 0.5 * (curr_v_max / curr_t_acc) * curr_t_acc**2
        dec_displacement = curr_v_max * curr_t_acc + 0.5 * (-curr_v_max / curr_t_acc) * curr_t_acc**2
        
        # 计算匀速阶段的位移（弧度）
        const_displacement = total_displacement - acc_displacement - dec_displacement
        
        # 初始化 t_acc_actual
        t_acc_actual = curr_t_acc

        # 判断是否需要匀速阶段
        if const_displacement > 0:
            # 有匀速阶段的情况
            t_const = target_time - 2 * curr_t_acc
            acc_acceleration = curr_v_max / curr_t_acc
            dec_acceleration = -curr_v_max / curr_t_acc
        else:
            # 没有匀速阶段的情况，需要重新计算加速度
            # 使用运动学公式：s = v0*t + 0.5*a*t^2
            # 总位移 = 加速位移 + 减速位移
            # 加速时间 = 减速时间 = target_time/2
            t_acc_actual = target_time / 2
            # 解方程：total_displacement = v_start*t_acc_actual + 0.5*a*t_acc_actual^2 + (v_start + a*t_acc_actual)*t_acc_actual + 0.5*(-a)*t_acc_actual^2
            # 化简：total_displacement = 2*v_start*t_acc_actual + a*t_acc_actual^2
            acc_acceleration = (total_displacement - 2 * curr_v_start * t_acc_actual) / (t_acc_actual**2)
            dec_acceleration = -acc_acceleration
            t_const = 0
        
        # 计算每个时间点的运动状态
        for i, t in enumerate(times):
            if t < t_acc_actual:  # 加速阶段
                velocities[i, joint] = curr_v_start + acc_acceleration * t
                accelerations[i, joint] = acc_acceleration
                positions[i, joint] = curr_start_angle + curr_v_start * t + 0.5 * acc_acceleration * t**2
            elif t < t_acc_actual + t_const:  # 匀速阶段
                velocities[i, joint] = curr_v_start + acc_acceleration * t_acc_actual
                accelerations[i, joint] = 0
                positions[i, joint] = curr_start_angle + (curr_v_start * t_acc_actual + 0.5 * acc_acceleration * t_acc_actual**2) + velocities[i, joint] * (t - t_acc_actual)
            else:  # 减速阶段
                t_dec = t - (t_acc_actual + t_const)
                velocities[i, joint] = (curr_v_start + acc_acceleration * t_acc_actual) + dec_acceleration * t_dec
                accelerations[i, joint] = dec_acceleration
                positions[i, joint] = curr_start_angle + (curr_v_start * t_acc_actual + 0.5 * acc_acceleration * t_acc_actual**2) + (curr_v_start + acc_acceleration * t_acc_actual) * t_const + (curr_v_start + acc_acceleration * t_acc_actual) * t_dec + 0.5 * dec_acceleration * t_dec**2
        
        # 根据位移方向调整位置
        if curr_displacement < 0:
            positions[:, joint] = 2 * curr_start_angle - positions[:, joint]
        
        # 确保最终位置精确到达目标位置
        positions[-1, joint] = curr_target_angle
    
    return times, velocities, accelerations, positions

def s_curve_velocity_planning(start_angles, target_angles, v_max=[pi/4] * 6, t_acc=[2] * 6, v_start=[0] * 6, dt=0.1):
    """实现S型加减速曲线规划，专门用于6关节机器人
    
    Args:
        start_angles: 起始关节角度（弧度），形状为(6,)的数组，表示六个关节的起始角度
        target_angles: 目标关节角度（弧度），形状为(6,)的数组，表示六个关节的目标角度
        v_max: 每个关节的最大角速度（弧度/秒），默认为[pi/4] * 6
        t_acc: 每个关节的加速时间（秒），默认为[8] * 6
        v_start: 每个关节的初始速度（弧度/秒），默认为[0] * 6
        dt: 采样间隔时间（秒），默认为0.1秒
    
    Returns:
        times: 时间点序列
        velocities: 对应的速度序列，形状为(num_samples, 6)
        accelerations: 对应的加速度序列，形状为(num_samples, 6)
        positions: 对应的位置序列，形状为(num_samples, 6)
    """
    # 将输入转换为numpy数组以便计算
    start_angles = np.array(start_angles)  # 弧度
    target_angles = np.array(target_angles)  # 弧度
    v_max = np.array(v_max)    # 弧度/秒
    t_acc = np.array(t_acc)    # 秒
    v_start = np.array(v_start)  # 弧度/秒
    
    # 计算每个关节的位移（弧度）
    displacements = target_angles - start_angles
    
    # 找出最大位移，这将决定总运动时间
    max_displacement = np.max(np.abs(displacements))
    max_displacement_idx = np.argmax(np.abs(displacements))
    
    # 使用最大位移关节的参数计算总时间
    max_v_max = v_max[max_displacement_idx]
    max_t_acc = t_acc[max_displacement_idx]
    max_v_start = v_start[max_displacement_idx]
    
    # 计算最大位移关节的加速和减速阶段位移（弧度）
    acc_displacement = max_v_start * max_t_acc + 0.5 * (max_v_max / max_t_acc) * max_t_acc**2
    dec_displacement = max_v_max * max_t_acc + 0.5 * (-max_v_max / max_t_acc) * max_t_acc**2
    
    # 计算匀速阶段的位移（弧度）
    const_displacement = max_displacement - acc_displacement - dec_displacement
    
    # 计算匀速阶段的时间（秒）
    t_const = np.where(const_displacement > 0,
                      const_displacement / max_v_max,
                      0)
    
    # 计算总时间（秒）
    target_time = 2 * max_t_acc + t_const
    
    # 计算采样点数
    num_samples = int(np.ceil(target_time / dt))
    times = np.linspace(0, target_time, num_samples)
    
    # 初始化返回数组
    velocities = np.zeros((num_samples, 6))  # 弧度/秒
    accelerations = np.zeros((num_samples, 6))  # 弧度/秒²
    positions = np.zeros((num_samples, 6))  # 弧度
    
    # 计算每个关节的运动曲线
    for joint in range(6):
        # 获取当前关节的参数
        curr_v_max = v_max[joint]
        curr_t_acc = t_acc[joint]
        curr_v_start = v_start[joint]
        curr_displacement = displacements[joint]
        curr_start_angle = start_angles[joint]
        curr_target_angle = target_angles[joint]
        
        # 计算当前关节的总位移（弧度）
        total_displacement = abs(curr_displacement)
        
        # 计算加速和减速阶段的位移（弧度）
        acc_displacement = curr_v_start * curr_t_acc + 0.5 * (curr_v_max / curr_t_acc) * curr_t_acc**2
        dec_displacement = curr_v_max * curr_t_acc + 0.5 * (-curr_v_max / curr_t_acc) * curr_t_acc**2
        
        # 计算匀速阶段的位移（弧度）
        const_displacement = total_displacement - acc_displacement - dec_displacement

        # 初始化 t_acc_actual
        t_acc_actual = curr_t_acc
        
        # 判断是否需要匀速阶段
        if const_displacement > 0:
            # 有匀速阶段的情况
            t_const = target_time - 2 * curr_t_acc
            acc_acceleration = curr_v_max / curr_t_acc
            dec_acceleration = -curr_v_max / curr_t_acc
        else:
            # 没有匀速阶段的情况，需要重新计算加速度
            t_acc_actual = target_time / 2
            acc_acceleration = (total_displacement - 2 * curr_v_start * t_acc_actual) / (t_acc_actual**2)
            dec_acceleration = -acc_acceleration
            t_const = 0
        
        # 计算每个时间点的运动状态
        for i, t in enumerate(times):
            if t < t_acc_actual:  # 加速阶段
                velocities[i, joint] = curr_v_start + acc_acceleration * t
                accelerations[i, joint] = acc_acceleration
                positions[i, joint] = curr_start_angle + curr_v_start * t + 0.5 * acc_acceleration * t**2
            elif t < t_acc_actual + t_const:  # 匀速阶段
                velocities[i, joint] = curr_v_start + acc_acceleration * t_acc_actual
                accelerations[i, joint] = 0
                positions[i, joint] = curr_start_angle + (curr_v_start * t_acc_actual + 0.5 * acc_acceleration * t_acc_actual**2) + velocities[i, joint] * (t - t_acc_actual)
            else:  # 减速阶段
                t_dec = t - (t_acc_actual + t_const)
                velocities[i, joint] = (curr_v_start + acc_acceleration * t_acc_actual) + dec_acceleration * t_dec
                accelerations[i, joint] = dec_acceleration
                positions[i, joint] = curr_start_angle + (curr_v_start * t_acc_actual + 0.5 * acc_acceleration * t_acc_actual**2) + (curr_v_start + acc_acceleration * t_acc_actual) * t_const + (curr_v_start + acc_acceleration * t_acc_actual) * t_dec + 0.5 * dec_acceleration * t_dec**2
        
        # 根据位移方向调整位置
        if curr_displacement < 0:
            positions[:, joint] = 2 * curr_start_angle - positions[:, joint]
        
        # 确保最终位置精确到达目标位置
        positions[-1, joint] = curr_target_angle
    
    return times, velocities, accelerations, positions

