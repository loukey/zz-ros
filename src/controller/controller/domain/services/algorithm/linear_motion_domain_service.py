from math import pi
import numpy as np
from .kinematic_domain_service import KinematicDomainService
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from scipy.spatial.transform import Rotation as R
    

class LinearMotionDomainService:
    def __init__(self, v_max=[pi/4] * 6, a_max=[pi/8] * 6, j_max=[pi/16] * 6, dt=0.01):
        self.v_max = np.asarray(v_max,dtype=float)
        self.a_max = np.asarray(a_max,dtype=float)
        self.j_max = np.asarray(j_max,dtype=float)
        self.dt = float(dt)
        self.kinematic_solver = KinematicDomainService()
        self.nearest_position = []

    def clamp(self, x, low, high):
        return max(low, min(x, high))

    def linear_motion(self, start_position, end_position):
        self.nearest_position = start_position
        start_quat, start_pos = self.kinematic_solver.get_gripper2base(start_position)
        end_quat, end_pos = self.kinematic_solver.get_gripper2base(end_position)
        quat_list, pos_list, n_seg = self.sampling(start_quat, start_pos, end_quat, end_pos)
        positions = self.smooth(quat_list, pos_list, n_seg)
        return positions

    def linear_motion_z_axis(self, start_position, distance, direction, ds=0.002, include_end=True):
        
        start_quat, start_pos = self.kinematic_solver.get_gripper2base(start_position)
        # todo: 按照distance计算终点位置
        p0 = np.asarray(start_pos, dtype=float).reshape(3)
        q0 = self._q_normalize(start_quat)
        
        direction = np.asarray(direction, dtype=float)
        norm = np.linalg.norm(direction)
        if norm < 1e-9:
            raise ValueError("Direction vector is zero or too small")
        direction = direction / norm

        # 需要的段数（保证每步不超过 ds）
        L = abs(float(distance))
        if L < 1e-12:
            t = np.array([0.0, 1.0]) if include_end else np.array([0.0])
        else:
            n_seg = max(1, int(np.ceil(L / ds)))
            t = np.linspace(0.0, 1.0, n_seg + 1 if include_end else n_seg)

        # 直线方向：xiangliang
        d = direction * float(distance)

        # 位置线性采样；姿态保持不变
        pos_list = p0[None, :] + t[:, None] * d[None, :]
        quat_list = np.repeat(q0[None, :], len(t), axis=0)

        return pos_list, quat_list, n_seg

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
        q1=q0
        q0 = np.asarray(q0, dtype=float).reshape(4)
        q1 = np.asarray(q1, dtype=float).reshape(4)
        # if np.argmax(np.abs(q0)) == 3:  # 可能是 [x,y,z,w]
        #     q0 = np.roll(q0, 1)
        # if np.argmax(np.abs(q1)) == 3:
        #     q1 = np.roll(q1, 1)

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

     
        return quat_list, pos_list, n_seg

    def smooth(self, quat_list, pos_list, n_seg):
        positions = []
        for quat, pos in zip(quat_list, pos_list):
            position = self.inverse_kinematic(quat, pos)
            positions.append(position)
        positions = np.array(positions)
        # todo: 基于这个positions列表，规划rucking smooth

        q_wp = self.ensure_2d_array(positions)
        grid_n = self.clamp(6*n_seg,300,3000)    
        t_list, positions, qd, qdd= self.toppra_time_parameterize(q_wp, self.v_max, self.a_max, self.dt, grid_n)
        return positions

    def inverse_kinematic(self, quat, pos):
        rm = R.from_quat(quat).as_matrix()
        inverse_position = self.kinematic_solver.inverse_kinematic(rm, pos, initial_theta=self.nearest_position)
        self.nearest_position = inverse_position
        return inverse_position

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

    def toppra_time_parameterize(self,
    waypoints: np.ndarray,
    v_max: np.ndarray | float,
    a_max: np.ndarray | float,
    dt: float = 0.01,
    grid_n: int = 400,
    ):
        waypoints = np.asarray(waypoints, dtype=float)
        if waypoints.ndim != 2 or waypoints.shape[1] != 6 or waypoints.shape[0] < 2:
            raise ValueError("waypoints 必须是 (N,6) 且 N>=2")

        dof = waypoints.shape[1]

        # 1) 几何路径：用样条在路径参数 s∈[0,1] 上插值
        breaks = np.linspace(0.0, 1.0, waypoints.shape[0])
        path = ta.SplineInterpolator(breaks, waypoints)  # 官方示例同款 API

        # 2) 速度/加速度约束（上下界格式）
        v_max = np.full(dof, float(v_max)) if np.isscalar(v_max) else np.asarray(v_max, dtype=float)
        a_max = np.full(dof, float(a_max)) if np.isscalar(a_max) else np.asarray(a_max, dtype=float)
        if v_max.shape != (dof,) or a_max.shape != (dof,):
            raise ValueError("v_max/a_max 需为标量或 shape=(6,)")

        v_bounds = np.column_stack((-np.abs(v_max), np.abs(v_max)))  # (2,6) -> [v_min; v_max]
        a_bounds = np.column_stack((-np.abs(a_max), np.abs(a_max)))  # (2,6) -> [a_min; a_max]

        pc_vel = constraint.JointVelocityConstraint(v_bounds)
        pc_acc = constraint.JointAccelerationConstraint(a_bounds)

        # 3) TOPPRA 主算法 + 常用参数化器（常用且满足边界/约束）
        gridpoints = np.linspace(0, path.duration, int(grid_n))
        instance = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints, parametrizer="ParametrizeConstAccel")
 
        # 4) 求解时间参数化，得到可按时间采样的 jnt_traj
        jnt_traj = instance.compute_trajectory(sd_start=0.0, sd_end=0.0)
        if jnt_traj is None:
            raise RuntimeError("TOPPRA 求解失败：给定约束下不可行，或路径异常。")

        # 5) 按 dt 采样
        T = float(jnt_traj.duration)
        M = max(2, int(np.ceil(T / dt)) + 1)
        t = np.linspace(0.0, T, M)

        q   = jnt_traj.eval(t)
        qd  = jnt_traj.eval(t)
        qdd = jnt_traj.eval(t)
        return t.tolist(), q.tolist(), qd.tolist(), qdd.tolist()
