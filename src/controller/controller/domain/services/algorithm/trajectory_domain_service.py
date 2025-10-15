import numpy as np
from math import pi


class SCurve():
    def __init__(self, v_max=[pi/4] * 6, acc_max=[pi/8] * 6, t_j=0.5):
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

    def planning(self, start_angles, target_angles, v_start=[0] * 6, dt=0.01):
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
                [t_acc_1 + t_acc_2 + t_acc_3 + t_const + t_dec_1, -jerk, a_acc_3, v_acc_3, s_acc_3 + s_const],
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
                [t_acc_1 + t_acc_2, -jerk, a_acc_1, v_acc_1, s_acc_1],
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
            