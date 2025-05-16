import numpy as np
from math import pi
from kinematic import *
from scipy.spatial.transform import Rotation as R, Slerp


def trapezoidal_velocity_planning(start_angles, target_angles, v_max=[pi/4] * 6, acc_max=[pi/8] * 6, v_start=[0] * 6, dt=0.1):
    """实现梯形加减速曲线规划，专门用于6关节机器人
    
    Args:
        start_angles: 起始关节角度（弧度），形状为(6,)的数组，表示六个关节的起始角度
        target_angles: 目标关节角度（弧度），形状为(6,)的数组，表示六个关节的目标角度
        v_max: 每个关节的最大角速度（弧度/秒），默认为[pi/4] * 6
        acc_max: 每个关节的最大角加速度（弧度/秒²），默认为[pi/8] * 6
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
    acc_max = np.array(acc_max)  # 弧度/秒²
    v_start = np.array(v_start)  # 弧度/秒
    
    # 计算每个关节的位移（弧度）
    displacements = target_angles - start_angles
    
    # 计算位移的绝对值
    abs_displacements = np.abs(displacements)
    
    # 特殊情况：所有位移都为0（起始位置等于目标位置）
    if np.all(abs_displacements < 1e-6):
        # 返回单个时间点的结果
        num_samples = 1
        times = np.array([0.0])
        velocities = np.zeros((num_samples, 6))
        accelerations = np.zeros((num_samples, 6))
        positions = np.copy(start_angles).reshape((num_samples, 6))
        return times, velocities, accelerations, positions
    
    # 找出最大位移，这将决定总运动时间
    max_displacement = np.max(abs_displacements)
    max_displacement_idx = np.argmax(abs_displacements)
    
    # 使用最大位移关节的参数计算
    max_v_max = v_max[max_displacement_idx]
    max_acc_max = acc_max[max_displacement_idx]
    max_v_start = v_start[max_displacement_idx]
    
    # 计算加速到最大速度所需的时间
    if max_acc_max < 1e-6:
        t_acc = 0  # 避免除零错误
    else:
        t_acc = (max_v_max - max_v_start) / max_acc_max
    
    # 计算加速段的位移
    # s_acc = v_start * t_acc + 0.5 * acc_max * t_acc^2
    s_acc = max_v_start * t_acc + 0.5 * max_acc_max * t_acc**2
    
    # 计算减速段的位移
    # s_dec = v_max * t_acc - 0.5 * acc_max * t_acc^2
    s_dec = max_v_max * t_acc - 0.5 * max_acc_max * t_acc**2
    
    # 计算匀速阶段的位移
    s_const = max_displacement - s_acc - s_dec
    
    # 计算总运动时间
    if s_const > 0:
        # 有匀速阶段的情况
        if max_v_max < 1e-6:
            t_const = 0  # 避免除零错误
        else:
            t_const = s_const / max_v_max
        
        # 总时间 = 加速时间 + 匀速时间 + 减速时间
        target_time = t_acc + t_const + t_acc
    else:
        # 没有匀速阶段的情况（三角形速度曲线）
        # 需要重新计算加速时间，无法达到最大速度
        # 解方程：s = 2 * (v_start * t_acc_new + 0.5 * acc_max * t_acc_new^2)
        if max_acc_max < 1e-6:
            # 极小情况处理
            t_acc_new = np.sqrt(2 * max_displacement / 1e-5)
        else:
            # 计算新的加速时间
            # a*t^2 + b*t - c = 0的求根公式
            a = max_acc_max
            b = 2 * max_v_start
            c = 2 * max_displacement
            
            t_acc_new = (-b + np.sqrt(b**2 + 4*a*c)) / (2*a)
        
        # 总时间 = 加速时间 + 减速时间
        target_time = 2 * t_acc_new
        t_acc = t_acc_new
        t_const = 0
    
    # 确保最小时间不小于dt
    target_time = max(target_time, dt)
    
    # 计算采样点数
    num_samples = max(int(np.ceil(target_time / dt)), 1)  # 确保至少有一个样本点
    times = np.linspace(0, target_time, num_samples)
    
    # 初始化返回数组
    velocities = np.zeros((num_samples, 6))  # 弧度/秒
    accelerations = np.zeros((num_samples, 6))  # 弧度/秒²
    positions = np.zeros((num_samples, 6))  # 弧度
    
    # 初始化位置为起始角度
    positions[:] = np.tile(start_angles, (num_samples, 1))
    
    # 计算每个关节的运动曲线
    for joint in range(6):
        # 获取当前关节的参数
        curr_v_max = v_max[joint]
        curr_acc_max = acc_max[joint]
        curr_v_start = v_start[joint]
        curr_displacement = displacements[joint]
        curr_start_angle = start_angles[joint]
        curr_target_angle = target_angles[joint]
        
        # 跳过没有位移的关节
        if abs(curr_displacement) < 1e-6:
            continue
        
        # 位移方向
        direction = 1 if curr_displacement >= 0 else -1
        curr_displacement_abs = abs(curr_displacement)
        
        # 根据总运动时间缩放当前关节的速度和加速度
        if curr_displacement_abs > 0:
            if s_const > 0:
                # 有匀速段的情况
                # 计算需要的最大速度和加速度
                if target_time > 2 * t_acc:
                    # 正常情况
                    curr_t_acc = t_acc
                    curr_t_const = target_time - 2 * t_acc
                    
                    # 计算所需的最大速度
                    curr_v_needed = curr_displacement_abs / (t_acc + curr_t_const)
                    
                    # 计算所需的加速度
                    curr_acc_needed = (curr_v_needed - curr_v_start) / t_acc
                    
                    # 限制在最大值范围内
                    curr_actual_acc = min(curr_acc_max, abs(curr_acc_needed)) * direction
                else:
                    # 时间太短，无法正常加速减速
                    curr_t_acc = target_time / 2
                    curr_t_const = 0
                    
                    # 计算能达到的峰值速度
                    curr_v_peak = curr_v_start + curr_displacement_abs / target_time
                    
                    # 计算所需的加速度
                    curr_acc_needed = 2 * (curr_v_peak - curr_v_start) / target_time
                    
                    # 限制在最大值范围内
                    curr_actual_acc = min(curr_acc_max, abs(curr_acc_needed)) * direction
            else:
                # 没有匀速段的情况（三角形速度曲线）
                curr_t_acc = target_time / 2
                curr_t_const = 0
                
                # 计算所需的加速度
                # 解方程：s = 2 * (v_start * t_acc + 0.5 * a * t_acc^2)
                curr_acc_needed = (curr_displacement_abs - 2 * curr_v_start * curr_t_acc) / (curr_t_acc**2)
                
                # 限制在最大值范围内
                curr_actual_acc = min(curr_acc_max, abs(curr_acc_needed)) * direction
            
            # 计算每个时间点的运动状态
            for i, t in enumerate(times):
                if t < curr_t_acc:  # 加速阶段
                    accelerations[i, joint] = curr_actual_acc
                    velocities[i, joint] = curr_v_start + curr_actual_acc * t
                    positions[i, joint] = curr_start_angle + curr_v_start * t + 0.5 * curr_actual_acc * t**2
                elif t < curr_t_acc + curr_t_const:  # 匀速阶段
                    v_reached = curr_v_start + curr_actual_acc * curr_t_acc
                    accelerations[i, joint] = 0
                    velocities[i, joint] = v_reached
                    positions[i, joint] = curr_start_angle + curr_v_start * curr_t_acc + 0.5 * curr_actual_acc * curr_t_acc**2 + v_reached * (t - curr_t_acc)
                else:  # 减速阶段
                    t_dec = t - (curr_t_acc + curr_t_const)
                    v_reached = curr_v_start + curr_actual_acc * curr_t_acc
                    accelerations[i, joint] = -curr_actual_acc
                    velocities[i, joint] = v_reached - curr_actual_acc * t_dec
                    positions[i, joint] = curr_start_angle + curr_v_start * curr_t_acc + 0.5 * curr_actual_acc * curr_t_acc**2 + v_reached * curr_t_const + v_reached * t_dec - 0.5 * curr_actual_acc * t_dec**2
        
        # 确保最终位置精确到达目标位置
        positions[-1, joint] = curr_target_angle
    
    return times, velocities, accelerations, positions

class SCurve():
    def __init__(self, v_max=[pi/2] * 6, acc_max=[pi] * 6, t_j=0.5):
        self.v_max = np.array(v_max)
        self.acc_max = np.array(acc_max)
        self.t_j = t_j

    @staticmethod
    def solve_cubic_numeric(a, b, c, d, tol = 1e-8):
        roots = np.roots([a, b, c, d])
        real_mask = np.isclose(roots.imag, 0, atol=tol)
        real_roots = roots[real_mask].real

        return real_roots
    
    @staticmethod
    def solve_quadratic(a, b, c, tol=1e-8):
        coeffs = [a, b, c]
        roots = np.roots(coeffs)
        real_mask = np.isclose(roots.imag, 0, atol=tol)
        real_roots = roots[real_mask].real
        
        return real_roots
    
    @staticmethod
    def base_method(t, jerk, a_start, v_start, s_start):
        a = jerk * t + a_start
        v = jerk * t**2 / 2 + a_start * t + v_start
        s = jerk * t**3 / 6 + a_start * t**2 / 2 + v_start * t + s_start
        return a, v, s

    def get_boundary_1(self, jerk, a_start, v_start, s_start, v_max, acc_max):
        # boundary stage 6
        # 加加速
        a_acc_1, v_acc_1, s_acc_1 = self.base_method(self.t_j, jerk, a_start, v_start, s_start)
        # 匀加速
        delta_v = v_max - v_acc_1 - acc_max * self.t_j / 2
        t_acc_2 = delta_v / acc_max
        a_acc_2, v_acc_2, s_acc_2 = self.base_method(t_acc_2, 0, a_acc_1, v_acc_1, s_acc_1)
        # 减加速
        a_acc_3, v_acc_3, s_acc_3 = self.base_method(self.t_j, -jerk, a_acc_2, v_acc_2, s_acc_2)
        # 加减速
        a_dec_1, v_dec_1, s_dec_1 = self.base_method(self.t_j, -jerk, a_acc_3, v_acc_3, s_acc_3)
        # 匀减速
        delta_v = v_max - acc_max * self.t_j
        t_dec_2 = delta_v / acc_max
        a_dec_2, v_dec_2, s_dec_2 = self.base_method(t_dec_2, 0, a_dec_1, v_dec_1, s_dec_1)
        # 减减速
        a_dec_3, v_dec_3, s_dec_3 = self.base_method(self.t_j, jerk, a_dec_2, v_dec_2, s_dec_2)
        list_t = [self.t_j, t_acc_2, self.t_j, self.t_j, t_dec_2, self.t_j]
        list_a = [a_acc_1, a_acc_2, a_acc_3, a_dec_1, a_dec_2, a_dec_3]
        list_v = [v_acc_1, v_acc_2, v_acc_3, v_dec_1, v_dec_2, v_dec_3]
        list_s = [s_acc_1, s_acc_2, s_acc_3, s_dec_1, s_dec_2, s_dec_3]
        return s_dec_3, list_t, list_a, list_v, list_s
    
    def get_boundary_2(self, jerk, a_start, v_start, s_start):
        # boundary stage 4
        # 加加速
        a_acc_1, v_acc_1, s_acc_1 = self.base_method(self.t_j, jerk, a_start, v_start, s_start)
        # 减加速
        a_acc_2, v_acc_2, s_acc_2 = self.base_method(self.t_j, -jerk, a_acc_1, v_acc_1, s_acc_1)
        # 加减速
        a_dec_1, v_dec_1, s_dec_1 = self.base_method(self.t_j, -jerk, a_acc_2, v_acc_2, s_acc_2)
        # 匀减速
        t_dec_2 = v_start / a_dec_1
        a_dec_2, v_dec_2, s_dec_2 = self.base_method(t_dec_2, 0, a_dec_1, v_dec_1, s_dec_1)
        # 减减速
        a_dec_3, v_dec_3, s_dec_3 = self.base_method(self.t_j, jerk, a_dec_2, v_dec_2, s_dec_2)
        sum_t = 4 * self.t_j + t_dec_2
        return s_dec_3, sum_t
    
    def get_stage_2(self, jerk, a_start, v_start, s_start, t_k, t_0):
        a_acc_1, v_acc_1, s_acc_1 = self.base_method(self.t_j, jerk, a_start, v_start, s_start)
        a_acc_2, v_acc_2, s_acc_2 = self.base_method(t_k, 0, a_acc_1, v_acc_1, s_acc_1)
        a_acc_3, v_acc_3, s_acc_3 = self.base_method(self.t_j, -jerk, a_acc_2, v_acc_2, s_acc_2)
        a_dec_1, v_dec_1, s_dec_1 = self.base_method(self.t_j, -jerk, a_acc_3, v_acc_3, s_acc_3)
        a_dec_2, v_dec_2, s_dec_2 = self.base_method(t_k, 0, a_dec_1, v_dec_1, s_dec_1)
        a_dec_3, v_dec_3, s_dec_3 = self.base_method(t_0, 0, a_dec_2, v_dec_2, s_dec_2)
        a_dec_4, v_dec_4, s_dec_4 = self.base_method(self.t_j, jerk, a_dec_3, v_dec_3, s_dec_3)
        list_t = [self.t_j, t_k, self.t_j, self.t_j, t_k, t_0, self.t_j]
        list_a = [a_acc_1, a_acc_2, a_acc_3, a_dec_1, a_dec_2, a_dec_3, a_dec_4]
        list_v = [v_acc_1, v_acc_2, v_acc_3, v_dec_1, v_dec_2, v_dec_3, v_dec_4]
        list_s = [s_acc_1, s_acc_2, s_acc_3, s_dec_1, s_dec_2, s_dec_3, s_dec_4]
        return s_dec_4, list_t, list_a, list_v, list_s
    
    def get_stage_3(self, jerk, a_start, v_start, s_start, t_j, t_0):
        a_acc_1, v_acc_1, s_acc_1 = self.base_method(t_j, jerk, a_start, v_start, s_start)
        a_acc_2, v_acc_2, s_acc_2 = self.base_method(t_j, -jerk, a_acc_1, v_acc_1, s_acc_1)
        a_acc_3, v_acc_3, s_acc_3 = self.base_method(t_j, -jerk, a_acc_2, v_acc_2, s_acc_2)
        a_acc_4, v_acc_4, s_acc_4 = self.base_method(t_0, 0, a_acc_3, v_acc_3, s_acc_3)
        a_acc_5, v_acc_5, s_acc_5 = self.base_method(t_j, jerk, a_acc_4, v_acc_4, s_acc_4)
        list_t = [t_j, t_j, t_j, t_0, t_j]
        list_a = [a_acc_1, a_acc_2, a_acc_3, a_acc_4, a_acc_5]
        list_v = [v_acc_1, v_acc_2, v_acc_3, v_acc_4, v_acc_5]
        list_s = [s_acc_1, s_acc_2, s_acc_3, s_acc_4, s_acc_5]
        return s_acc_5, list_t, list_a, list_v, list_s
    
    def get_result(self, param_arr, target_time, dt, max_displacement_idx):
        num_samples = max(int(np.ceil(target_time / dt)), 1) 
        times = np.linspace(0, target_time, num_samples)
        velocities = np.zeros((num_samples, 6))  # 弧度/秒
        accelerations = np.zeros((num_samples, 6))  # 弧度/秒²
        positions = np.zeros((num_samples, 6))  # 弧度
        for i, sample_time in enumerate(times):
            for j in range(1, len(param_arr)):
                if sample_time < param_arr[j, 0]:
                    accelerations[i, max_displacement_idx], velocities[i, max_displacement_idx], positions[i, max_displacement_idx] = \
                        self.base_method(sample_time - param_arr[j-1, 0], param_arr[j, 1], param_arr[j, 2], param_arr[j, 3], param_arr[j, 4])
                    break
        return times, accelerations, velocities, positions
    
    @staticmethod
    def scale_result(accelerations, velocities, positions, max_displacement_idx, displacements, abs_displacements, start_angles, target_angles):
        for j in range(6):
            move_symbol = 0 if displacements[j] == 0 else displacements[j] / abs_displacements[j]
            if j == max_displacement_idx:
                continue
            else:
                scale_factor = abs_displacements[j] / abs_displacements[max_displacement_idx]
                velocities[:, j] = velocities[:, max_displacement_idx] * scale_factor * move_symbol
                accelerations[:, j] = accelerations[:, max_displacement_idx] * scale_factor * move_symbol
                positions[:, j] = positions[:, max_displacement_idx] * scale_factor * move_symbol + start_angles[j]
        positions[:, max_displacement_idx] *= displacements[max_displacement_idx] / abs_displacements[max_displacement_idx]
        positions[:, max_displacement_idx] += start_angles[max_displacement_idx]
        positions[-1, :] = target_angles        
        return accelerations, velocities, positions

    def planning(self, start_angles, target_angles, v_start=[0] * 6, dt=0.1):
        start_angles = np.array(start_angles)
        target_angles = np.array(target_angles)
        displacements = target_angles - start_angles
        abs_displacements = np.abs(displacements)
        if np.all(abs_displacements < 1e-6):
            num_samples = 1
            times = np.array([0.0])
            velocities = np.zeros((num_samples, 6))
            accelerations = np.zeros((num_samples, 6))
            positions = np.copy(start_angles).reshape((num_samples, 6))
            return times, velocities, accelerations, positions
        max_displacement_idx = np.argmax(abs_displacements)
        max_displacement = abs_displacements[max_displacement_idx]
        a_max = self.acc_max[max_displacement_idx]
        v_max = self.v_max[max_displacement_idx]
        max_v_start = v_start[max_displacement_idx]
        jerk = a_max / self.t_j
        s_1, list_t_1, list_a_1, list_v_1, list_s_1 = self.get_boundary_1(jerk, 0, max_v_start, 0, v_max, a_max)
        s_2, t_2 = self.get_boundary_2(jerk, 0, max_v_start, 0)
        if s_1 < max_displacement:
            t_acc_1, t_acc_2, t_acc_3, t_dec_1, t_dec_2, t_dec_3 = list_t_1
            a_acc_1, a_acc_2, a_acc_3, a_dec_1, a_dec_2, a_dec_3 = list_a_1
            v_acc_1, v_acc_2, v_acc_3, v_dec_1, v_dec_2, v_dec_3 = list_v_1
            s_acc_1, s_acc_2, s_acc_3, s_dec_1, s_dec_2, s_dec_3 = list_s_1
            s_const = max_displacement - s_1
            t_const = s_const / v_max
            target_time = sum(list_t_1) + t_const
            param_arr = np.array([
                [0, 0, 0, 0, 0],
                [t_acc_1, jerk, 0, max_v_start, 0],
                [t_acc_1 + t_acc_2, 0, a_acc_1, v_acc_1, s_acc_1],
                [t_acc_1 + t_acc_2 + t_acc_3, -jerk, a_acc_2, v_acc_2, s_acc_2],
                [t_acc_1 + t_acc_2 + t_acc_3 + t_const, 0, a_acc_3, v_acc_3, s_acc_3],
                [t_acc_1 + t_acc_2 + t_acc_3 + t_const + t_dec_1, -jerk, a_acc_3, v_acc_3, s_const],
                [t_acc_1 + t_acc_2 + t_acc_3 + t_const + t_dec_1 + t_dec_2, 0, a_dec_1, v_dec_1, s_dec_1 + s_const],
                [t_acc_1 + t_acc_2 + t_acc_3 + t_const + t_dec_1 + t_dec_2 + t_dec_3, jerk, a_dec_2, v_dec_2, s_dec_2 + s_const],
            ])
            times, accelerations, velocities, positions = self.get_result(param_arr, target_time, dt, max_displacement_idx)
            accelerations, velocities, positions = self.scale_result(accelerations, 
                                                                     velocities,
                                                                     positions,
                                                                     max_displacement_idx, 
                                                                     displacements, 
                                                                     abs_displacements,
                                                                     start_angles, 
                                                                     target_angles)
            return times, accelerations, velocities, positions

        elif s_2 > max_displacement:
            t0 = max_v_start / jerk
            a = 2 * jerk
            b = jerk * t0 / 2
            c = 4 * max_v_start - jerk * t0**2 / 2
            d = -max_displacement
            t_j = self.solve_cubic_numeric(a, b, c, d)
            t_j = t_j[(t_j > 0) & (t_j < self.t_j)][0]
            s_acc_5, list_t_3, list_a_3, list_v_3, list_s_3 = self.get_stage_3(jerk, 0, max_v_start, 0, t_j, t0)
            t_acc_1, t_acc_2, t_dec_1, t_dec_2, t_dec_3 = list_t_3
            a_acc_1, a_acc_2, a_dec_1, a_dec_2, a_dec_3 = list_a_3
            v_acc_1, v_acc_2, v_dec_1, v_dec_2, v_dec_3 = list_v_3
            s_acc_1, s_acc_2, s_dec_1, s_dec_2, s_dec_3 = list_s_3
            target_time = sum(list_t_3)
            param_arr = np.array([
                [0, 0, 0, 0, 0],
                [t_acc_1, jerk, 0, max_v_start, 0],
                [t_acc_1 + t_acc_2, 0, a_acc_1, v_acc_1, s_acc_1],
                [t_acc_1 + t_acc_2 + t_dec_1, -jerk, a_acc_2, v_acc_2, s_acc_2],
                [t_acc_1 + t_acc_2 + t_dec_1 + t_dec_2, 0, a_dec_1, v_dec_1, s_dec_1],
                [t_acc_1 + t_acc_2 + t_dec_1 + t_dec_2 + t_dec_3, jerk, a_dec_2, v_dec_2, s_dec_2],
            ])
            times, accelerations, velocities, positions = self.get_result(param_arr, target_time, dt, max_displacement_idx)
            accelerations, velocities, positions = self.scale_result(accelerations, 
                                                                     velocities,
                                                                     positions,
                                                                     max_displacement_idx, 
                                                                     displacements, 
                                                                     abs_displacements,
                                                                     start_angles, 
                                                                     target_angles)
            return times, accelerations, velocities, positions
        else:
            a_acc_1, v_acc_1, s_acc_1 = self.base_method(self.t_j, jerk, 0, max_v_start, 0)
            t0 = max_v_start / a_acc_1
            s_t0 = a_max * t0**2 / 2 + jerk * self.t_j ** 2 * t0 / 2
            s_dec_3 = jerk * self.t_j ** 3 / 6
            delta_s = max_displacement - s_t0 - s_dec_3 - s_acc_1
            a = a_max / 2
            b = 3 / 2 * a_max * self.t_j + max_v_start
            c = 5 / 6 * a_max * self.t_j ** 2 + max_v_start * self.t_j - delta_s / 2 
            t_k = self.solve_quadratic(a, b, c)
            t_k = t_k[(4 * self.t_j + 2 * t_k + t0 > t_2) & (4 * self.t_j + 2 * t_k + t0 < sum(list_t_1))][0]
            s_dec_4, list_t_2, list_a_2, list_v_2, list_s_2 = self.get_stage_2(jerk, 0, max_v_start, 0, t_k, t0)
            t_acc_1, t_acc_2, t_acc_3, t_dec_1, t_dec_2, t_dec_3, t_dec_4 = list_t_2
            a_acc_1, a_acc_2, a_acc_3, a_dec_1, a_dec_2, a_dec_3, a_dec_4 = list_a_2
            v_acc_1, v_acc_2, v_acc_3, v_dec_1, v_dec_2, v_dec_3, v_dec_4 = list_v_2
            s_acc_1, s_acc_2, s_acc_3, s_dec_1, s_dec_2, s_dec_3, s_dec_4 = list_s_2

            target_time = sum(list_t_2)
            param_arr = np.array([
                [0, 0, 0, 0, 0],
                [t_acc_1, jerk, 0, max_v_start, 0],
                [t_acc_1 + t_acc_2, 0, a_acc_1, v_acc_1, s_acc_1],
                [t_acc_1 + t_acc_2 + t_acc_3, -jerk, a_acc_2, v_acc_2, s_acc_2],
                [t_acc_1 + t_acc_2 + t_acc_3 + t_dec_1, -jerk, a_acc_3, v_acc_3, s_acc_3],
                [t_acc_1 + t_acc_2 + t_acc_3 + t_dec_1 + t_dec_2, 0, a_dec_1, v_dec_1, s_dec_1],
                [t_acc_1 + t_acc_2 + t_acc_3 + t_dec_1 + t_dec_2 + t_dec_3, 0, a_dec_2, v_dec_2, s_dec_2],
                [t_acc_1 + t_acc_2 + t_acc_3 + t_dec_1 + t_dec_2 + t_dec_3 + t_dec_4, jerk, a_dec_3, v_dec_3, s_dec_3],
            ])
            times, accelerations, velocities, positions = self.get_result(param_arr, target_time, dt, max_displacement_idx)
            accelerations, velocities, positions = self.scale_result(accelerations, 
                                                            velocities,
                                                            positions,
                                                            max_displacement_idx, 
                                                            displacements, 
                                                            abs_displacements,
                                                            start_angles, 
                                                            target_angles)
            return times, accelerations, velocities, positions
            
class Linear():
    def __init__(self):
        self.kinematic = Kinematic6DOF()

    def planning(self, start_angles, target_angles, dt=0.1):
        A_start, B_start, C_start, p_start = self.kinematic.get_end_position(start_angles)
        A_target, B_target, C_target, p_target = self.kinematic.get_end_position(target_angles)
        q_start = R.from_euler('xyz', [A_start, B_start, C_start], degrees=False).as_quat()
        q_target = R.from_euler('xyz', [A_target, B_target, C_target], degrees=False).as_quat()
        slerp = Slerp([0, 1], R.from_quat([q_start, q_target]))
        times = np.linspace(0, 1, num=int(1 / dt + 1))
        interp_rots = slerp(times)
        eulers = interp_rots.as_euler('xyz', degrees=False)

        positions = []
        for t, euler in zip(times, eulers):
            p = p_start + t * (p_target - p_start)
            joint_angles = self.kinematic.inverse_kinematic(*euler, *p)
            positions.append(joint_angles)

        return times, positions
