from math import pi
from ruckig import Ruckig, InputParameter, OutputParameter, Result
import numpy as np
from .kinematic_6dof import *
from typing import Mapping, Sequence, List, Tuple 


class RuckigSmooth:
    def __init__(self, v_max=[pi/4] * 6, a_max=[pi/8] * 6, j_max=[pi/16] * 6, dt=0.01):
        self.v_max = np.asarray(v_max,dtype=float)
        self.a_max = np.asarray(a_max,dtype=float)
        self.j_max = np.asarray(j_max,dtype=float)
        self.dt = float(dt)
        self.kinematic_solver = Kinematic6DOF()

    def _limits_dict(self):
        return{"v_max":self.v_max,"a_max":self.a_max,"j_max":self.j_max}

    def rucking_smooth(self, start_position, end_position):
        start_quat, start_pos = self.kinematic_solver.get_end_position(start_position)
        end_quat, end_pos = self.kinematic_solver.get_end_position(end_position)
        quat_list, pos_list = self.sampling(start_quat, start_pos, end_quat, end_pos)
        positions = self.smooth(quat_list, pos_list)
        return positions

    def rucking_smooth_z_axis(self, start_position, distance, axis=-1):
        start_quat, start_pos = self.kinematic_solver.get_end_position(start_position)
        # todo: 按照distance计算终点位置
        p0 = np.asarray(p0, dtype=float).reshape(3)
        q0 = self._q_normalize(q0)

        # 需要的段数（保证每步不超过 ds）
        L = abs(float(distance))
        if L < 1e-12:
            t = np.array([0.0, 1.0]) if include_end else np.array([0.0])
        else:
            n_seg = max(1, int(np.ceil(L / ds)))
            t = np.linspace(0.0, 1.0, n_seg + 1 if include_end else n_seg)

        # 直线方向：基坐标 Z 轴
        d = np.array([0.0, 0.0, distance], dtype=float)

        # 位置线性采样；姿态保持不变
        pos_list = p0[None, :] + t[:, None] * d[None, :]
        quat_list = np.repeat(q0[None, :], len(t), axis=0)

        return pos_list, quat_list

    def _q_slerp(self, q0, q1, t):
        """四元数最短弧 SLERP，t in [0,1]"""
        q0 = self._q_normalize(q0)
        q1 = self._q_normalize(q1)
        dot = np.dot(q0, q1)

        # 如果点积为负，翻转 q1 以走最短路径
        if dot < 0.0:
            q1 = -q1
            dot = -dot

        # 非常接近时退化为线性插值再归一化，避免数值问题
        if dot > 0.9995:
            q = q0 + t * (q1 - q0)
            return self._q_normalize(q)

        theta0 = np.arccos(np.clip(dot, -1.0, 1.0))  # 初始夹角
        sin_theta0 = np.sin(theta0)
        s0 = np.sin((1.0 - t) * theta0) / sin_theta0
        s1 = np.sin(t * theta0) / sin_theta0
        return s0 * q0 + s1 * q1

    def sampling(self, start_quat, start_pos, end_quat, end_pos, sampling_dis=0.002,include_end=True):
        quat_list = []
        pos_list = []
        # todo: 按照起始四元数+位置和终点四元数+位置以及采样间隔0.02m，进行采样
        p0=start_pos
        p1=end_pos
        p0 = np.asarray(p0, dtype=float).reshape(3)
        p1 = np.asarray(p1, dtype=float).reshape(3)

        # 兼容两种四元数顺序：若最后一项绝对值最大，视为 [x,y,z,w]，转成 [w,x,y,z]
        q0=start_quat
        q1=end_quat
        q0 = np.asarray(q0, dtype=float).reshape(4)
        q1 = np.asarray(q1, dtype=float).reshape(4)
        if np.argmax(np.abs(q0)) == 3:  # 可能是 [x,y,z,w]
            q0 = np.roll(q0, 1)
        if np.argmax(np.abs(q1)) == 3:
            q1 = np.roll(q1, 1)

        # 计算需要的采样点数
        L = np.linalg.norm(p1 - p0)
        if L < 1e-9:
            t = np.array([0.0, 1.0]) if include_end else np.array([0.0])
        else:
            n_seg = max(1, int(np.ceil(L / float(sampling_dis))))
            t = np.linspace(0.0, 1.0, n_seg + 1 if include_end else n_seg)

        # 位置线性插值
        pos_list = (p0[None, :] + (p1 - p0)[None, :] * t[:, None])

        # 姿态 SLERP
        quat_list = np.vstack([self._q_slerp(q0, q1, ti) for ti in t])

     
        return quat_list, pos_list

    def smooth(self, quat_list, pos_list):
        positions = []
        for quat, pos in zip(quat_list, pos_list):
            position = self.inverse_kinematic(quat, pos)
            positions.append(position)
        positions = np.array(positions)
        # todo: 基于这个positions列表，规划rucking smooth

        q_wp = self.ensure_2d_array(positions)

        positions, qd, qdd, t_list = self.retime_with_ruckig_waypoints(q_wp, limits=self._limits_dict(), dt=self.dt)
        return positions

    def inverse_kinematic(self, quat, pos):
        rm = R.from_quat(quat).as_matrix()
        return self.kinematic_solver.inverse_kinematic(rm, pos)

    def _q_normalize(self,q):
        q = np.asarray(q, dtype=float)
        n = np.linalg.norm(q)
        if n == 0:
            raise ValueError("zero quaternion")
        return q / n

    def wrap_to_pi(self,q: np.ndarray) -> np.ndarray:
        """将角度包裹到 (-pi, pi]，避免 2π 跳变影响计算。"""
        return (q + np.pi) % (2 * np.pi) - np.pi


    def ensure_2d_array(self,arr: np.ndarray) -> np.ndarray:
        """
        将输入转换为二维数组 (N, dof) 并做基本校验。

        Raises
        ------
        ValueError
            当路标数少于 2（缺少起点或终点）时抛出。
        """
        q = np.asarray(arr, dtype=float)
        if q.ndim == 1:
            q = q[None, :]
        if not (q.ndim == 2 and q.shape[0] >= 2):
            raise ValueError("Q_waypoints 至少需要两个点（起点与终点）且维度为 (N, dof)。")
        return q

    def retime_with_ruckig_waypoints(self,
    q_waypoints: np.ndarray,
    limits: Mapping[str,Sequence[float]],
    dt: float = 0.01,
):
        """
        使用 Ruckig 的“中间路标连续通过”能力进行时间重定时（中间点不停，终点停）。

        Parameters
        ----------
        q_waypoints : np.ndarray
            路标序列，形状 (N, dof)，单位：弧度。
        limits : JointLimits
            关节约束（v/a/jerk 上限）。
        dt : float
            采样周期（秒），例如 0.002 → 500 Hz。

        Returns
        -------
        q_list, qd_list, qdd_list, t_list
            依次为：位置、速度、加速度、时间戳（均为等间隔 dt）。
        """
        try:
            from ruckig import Ruckig, InputParameter, OutputParameter, Result  # pip install ruckig
        except Exception as exc:
            raise RuntimeError("需要安装 python-ruckig（pip install ruckig）。") from exc

        q_wp = self.wrap_to_pi(self.ensure_2d_array(q_waypoints))
        dof = int(q_wp.shape[1])
        num_wp = max(0, q_wp.shape[0] - 2)
        otg = Ruckig(dof, dt, num_wp)
        
        inp = InputParameter(dof)
        out = OutputParameter(dof)

        # 当前状态（通常从静止开始；若不是静止请填真实值）
        inp.current_position = q_wp[0].tolist()
        inp.current_velocity = [0.0] * dof
        inp.current_acceleration = [0.0] * dof

        # 中间路标（不停顿通过）+ 终点（停住）
        if q_wp.shape[0] > 2 and hasattr(inp, "intermediate_positions"):
            # 低版本 Ruckig 可能没有该属性；外部会捕获 AttributeError 并降级。
            inp.intermediate_positions = [q.tolist() for q in q_wp[1:-1]]

        inp.target_position = q_wp[-1].tolist()
        inp.target_velocity = [0.0] * dof
        inp.target_acceleration = [0.0] * dof

        inp.max_velocity=list(limits["v_max"])
        inp.max_acceleration=list(limits["a_max"])
        inp.max_jerk = list(limits["j_max"])

        q_list = []
        qd_list = []
        qdd_list = []
        t_list = []

        t = 0.0
        while True:
            res = otg.update(inp, out)

            q_list.append(np.array(out.new_position).tolist())
            qd_list.append(np.array(out.new_velocity).tolist())
            qdd_list.append(np.array(out.new_acceleration).tolist())
            t_list.append(t)
            t += dt

            if res == Result.Working:
                out.pass_to_input(inp)
            elif res == Result.Finished:
                break
            else:
                raise RuntimeError(f"Ruckig 失败：{res}")

        return q_list, qd_list, qdd_list, t_list
