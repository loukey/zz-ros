import numpy as np
from math import pi
import sys
sys.path.append('/home/za/Codes/zzros')

# 模拟normalize_angle等函数
def normalize_angle(angle):
    return ((angle + pi) % (2 * pi)) - pi

def normalize_angle_difference(start_angle, end_angle):
    return normalize_angle(end_angle - start_angle)

def normalize_angles_array(angles_array):
    if isinstance(angles_array, list):
        return [normalize_angle(angle) for angle in angles_array]
    else:
        angles = np.array(angles_array)
        return np.array([normalize_angle(angle) for angle in angles.flat]).reshape(angles.shape)

# 模拟trapezoidal_velocity_planning函数
def trapezoidal_velocity_planning(angles, v_max=pi/4, t_acc=8, v_start=0, dt=0.01):
    # 确保angles是numpy数组并进行归一化
    angles = np.array(angles)
    angles = normalize_angles_array(angles)
    
    # 分析函数输入
    print('输入角度:', angles)
    
    # 计算总距离（最大角度变化）
    if len(angles.shape) == 0:  # 单个值
        total_distance = np.abs(angles)
    elif len(angles) == 1:  # 单关节
        total_distance = np.abs(angles[0])
    else:  # 多关节
        total_distance = np.max(np.abs(angles))
    
    print('总距离:', total_distance)
    
    # 如果总距离为0，则返回空结果
    if total_distance == 0:
        return np.array([0]), np.array([0]), np.array([0]), np.zeros_like(angles).reshape(1, -1)
    
    # 计算加速和减速参数
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
        v_peak = (total_distance + 0.5 * v_start * t_acc) / t_acc
        a_acc = (v_peak - v_start) / t_acc
        a_dec = v_peak / t_acc
        t_const = 0
        d_const = 0
    else:
        v_peak = v_max
        t_const = d_const / v_max
    
    # 计算总时间
    total_time = t_acc + t_const + t_acc
    
    print('总时间:', total_time)
    
    # 生成时间序列
    num_samples = int(np.ceil(total_time / dt)) + 1
    times = np.linspace(0, total_time, num_samples)
    velocities = np.zeros(num_samples)
    accelerations = np.zeros(num_samples)
    positions = np.zeros((num_samples, len(angles) if len(angles.shape) > 0 else 1))
    
    # 计算每个时间点的速度、加速度和位置
    for i, t in enumerate(times):
        scale_factor = 0
        if t <= t_acc:  # 加速阶段
            velocities[i] = v_start + a_acc * t
            accelerations[i] = a_acc
            scale_factor = (v_start * t + 0.5 * a_acc * t**2) / total_distance
        elif t <= t_acc + t_const:  # 匀速阶段
            velocities[i] = v_peak
            accelerations[i] = 0
            scale_factor = (d_acc + v_peak * (t - t_acc)) / total_distance
        else:  # 减速阶段
            t_in_dec = t - t_acc - t_const
            velocities[i] = v_peak - a_dec * t_in_dec
            accelerations[i] = -a_dec
            scale_factor = ((d_acc + d_const) + v_peak * t_in_dec - 0.5 * a_dec * t_in_dec**2) / total_distance
        
        # 打印scale_factor的值用于调试
        if i % 4 == 0:
            print(f"t={t:.2f}, scale_factor={scale_factor:.4f}")
        
        # 线性插值计算位置
        positions[i] = scale_factor * angles
        
        # 确保位置角度在每次计算后都是归一化的
        if len(angles.shape) > 0 and len(angles) > 1:
            for j in range(len(angles)):
                positions[i, j] = normalize_angle(positions[i, j])
        else:
            positions[i, 0] = normalize_angle(positions[i, 0])
    
    return times, velocities, accelerations, positions

# 测试函数
def debug_test():
    # 模拟用户的问题数据
    start_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_angles = [1.5708, 0.0, 0.0, 0.0, 0.0, 0.0]  # π/2 radians
    
    # 计算角度差
    angles_diff = [end - start for end, start in zip(target_angles, start_angles)]
    print('开始角度:', start_angles)
    print('目标角度:', target_angles)
    print('角度差值:', angles_diff)
    
    print('\n===== 测试 trapezoidal_velocity_planning 函数 =====')
    # 调用函数并查看结果 (使用dt=1.0来复现用户的问题)
    times, velocities, accelerations, positions = trapezoidal_velocity_planning(angles_diff, v_max=pi/4, t_acc=8, v_start=0, dt=1.0)
    
    # 打印关键信息
    print('\n时间点:')
    print(times)
    
    print('\n位置点:')
    for i, pos in enumerate(positions):
        print(f"t={times[i]:.2f}: {pos}")
    
    print('\n===== 检查positions序列中是否有不连续点 =====')
    for i in range(1, len(positions)):
        diff = abs(positions[i][0] - positions[i-1][0])
        if diff > 0.5:  # 检测突变
            print(f'在索引 {i-1} 到 {i} 之间检测到突变:')
            print(f'  时间 {i-1}: {times[i-1]:.2f}s, 位置: {positions[i-1][0]:.6f}')
            print(f'  时间 {i}: {times[i]:.2f}s, 位置: {positions[i][0]:.6f}')
            print(f'  差值: {diff:.6f}')
            print(f'  归一化后差值: {normalize_angle_difference(positions[i-1][0], positions[i][0]):.6f}')
            print(f'  此时scale_factor可能超过或接近1')

if __name__ == "__main__":
    debug_test() 