from math import pi
import numpy as np
from scipy.spatial.transform import Rotation as R
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.interpolate import interp1d
from .kinematic_domain_service import KinematicDomainService


class CurveMotionDomainService:
    """曲线运动规划服务。
    
    提供基于弧长参数化的空间曲线规划、姿态平滑插值以及基于 TOPPRA 的时间参数化功能。
    
    Attributes:
        v_max (np.ndarray): 各关节最大速度 (rad/s)。
        a_max (np.ndarray): 各关节最大加速度 (rad/s²)。
        j_max (np.ndarray): 各关节最大加加速度 (rad/s³)。
        dt (float): 时间步长 (s)。
        kinematic_solver (KinematicDomainService): 运动学求解器实例。
    """

    def __init__(
        self, 
        kinematic_solver: KinematicDomainService,
        v_max=[pi / 4] * 6, 
        a_max=[pi / 8] * 6, 
        j_max=[pi / 16] * 6, 
        dt=0.01,
    ):
        """初始化曲线运动服务。

        Args:
            kinematic_solver: 运动学求解器服务实例。
            v_max (list[float], optional): 最大速度列表. Defaults to [pi/4]*6.
            a_max (list[float], optional): 最大加速度列表. Defaults to [pi/8]*6.
            j_max (list[float], optional): 最大加加速度列表. Defaults to [pi/16]*6.
            dt (float, optional): 采样时间步长. Defaults to 0.01.
        """
        self.v_max = np.asarray(v_max, dtype=float)
        self.a_max = np.asarray(a_max, dtype=float)
        self.j_max = np.asarray(j_max, dtype=float)
        self.dt = float(dt)
        self.kinematic_solver = kinematic_solver
        self.nearest_position = None

    def clamp(self, x, low, high):
        """将值 x 限制在 [low, high] 区间内。"""
        return max(low, min(x, high))

    def curve_motion(
            self,
            pos_fun,  # callable u -> (3,)
            u0: float,
            u1: float,
            start_position: list[float],
            end_position: list[float] | None,
            ds: float = 0.002,
            include_end: bool = True,
            orientation_mode: str = "slerp",
            tool_axis: str = "z",
            up_hint: np.ndarray = np.array([0, 0, 1.0])
    ) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
        """执行曲线运动规划。
        
        Args:
            pos_fun (callable): 位置函数，接受参数 u 返回 (3,) 坐标。
            u0 (float): 参数起始值。
            u1 (float): 参数终止值。
            start_position (list[float]): 起点关节角度。
            end_position (list[float] | None): 终点关节角度。
            ds (float, optional): 采样弧长步长. Defaults to 0.002.
            include_end (bool, optional): 是否包含终点. Defaults to True.
            orientation_mode (str, optional): 姿态插值模式 ('fixed'|'slerp'|'tangent'). Defaults to "slerp".
            tool_axis (str, optional): 工具轴方向 ('x'|'y'|'z'). Defaults to "z".
            up_hint (np.ndarray, optional): 向上向量提示，用于切向跟随模式. Defaults to [0, 0, 1.0].
        
        Returns:
            tuple: (t_list, positions, qd, qdd)
                - t_list: 时间戳列表
                - positions: 关节角度轨迹列表
                - qd: 关节速度轨迹列表
                - qdd: 关节加速度轨迹列表
        
        Raises:
            ValueError: 如果 orientation_mode 无效或 slerp 模式下未提供 end_position。
        """
        # 起点姿态来自起点关节角的正解
        start_quat, start_pos_fk = self.kinematic_solver.get_gripper2base(start_position)
        # 1) 等弧长采样
        pos_list, u_eq, n_seg = self.sample_parametric_equal_arclen(pos_fun, u0, u1, ds, include_end)
        
        # 2) 姿态生成
        if orientation_mode == "fixed":
            quat_list = np.repeat(self._q_normalize(start_quat)[None, :], len(pos_list), axis=0)

        elif orientation_mode == "slerp":
            if end_position is None:
                raise ValueError("slerp 模式需要提供 end_position 以确定终止姿态。")
            end_quat, _ = self.kinematic_solver.get_gripper2base(end_position)
            q0 = self._q_normalize(start_quat)
            q1 = self._q_normalize(end_quat)
            tau = np.linspace(0, 1, len(pos_list))
            quat_list = np.vstack([self._q_slerp(q0, q1, ti) for ti in tau])
            
        elif orientation_mode == "tangent":
            quat_list = self._quat_follow_tangent(pos_list, tool_axis=tool_axis, up_hint=up_hint)
        else:
            raise ValueError("orientation_mode 必须是 {'fixed','slerp','tangent'}")

        # 3) 原有平滑/IK/TOPPRA 管线复用
        t_list, positions, qd, qdd = self.smooth(quat_list, pos_list, n_seg)
        return t_list, positions, qd, qdd

    def make_pos_fun_spline(
        self, 
        start_position: list[float],
        end_position: list[float],
        mid_points: list, 
        bc_type: str = "natural") -> tuple[callable, np.ndarray]:
        """构造基于三次样条的位置函数。
        
        Args:
            start_position (list[float]): 起点关节角度。
            end_position (list[float]): 终点关节角度。
            mid_points (list): 中间点坐标列表 (N, 3)。
            bc_type (str, optional): 边界条件类型. Defaults to "natural".
        
        Returns:
            tuple: (pos_fun, s)
                - pos_fun: 位置函数 callable
                - s: 累积弦长数组
        
        Raises:
            ValueError: 如果点集形状不正确。
        """
        _, start_pos = self.kinematic_solver.get_gripper2base(start_position)
        _, end_pos = self.kinematic_solver.get_gripper2base(end_position)
        points = np.vstack([start_pos, mid_points, end_pos])
        P = np.asarray(points, dtype=float)
        if P.ndim != 2 or P.shape[0] < 2 or P.shape[1] not in (2, 3):
            raise ValueError("points 形状必须是 (N,3) 或 (N,2)，且 N>=2")
        if P.shape[1] == 2:
            P = np.hstack([P, np.zeros((P.shape[0], 1))])  # 2D 自动补 z=0

        # --- 弦长参数化，避免参数拥挤 ---
        d = np.linalg.norm(np.diff(P, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        if s[-1] == 0:
            raise ValueError("所有点重合，无法构造曲线")
        t = s / s[-1]  # 归一化到 [0,1]

        # 分别构造 x,y,z 的样条
        cs_x = CubicSpline(t, P[:, 0], bc_type=bc_type)
        cs_y = CubicSpline(t, P[:, 1], bc_type=bc_type)
        cs_z = CubicSpline(t, P[:, 2], bc_type=bc_type)

        def _eval(u):
            u = np.asarray(u, dtype=float)
            # 可选：夹紧到 [0,1]，避免数值越界
            u = np.clip(u, 0.0, 1.0)
            X = np.stack([cs_x(u), cs_y(u), cs_z(u)], axis=-1)  # (..., 3)
            if X.ndim == 1:  # 标量 u -> (3,)
                return X
            return X  # 数组 u -> (M,3)

        return _eval, s


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

    def sample_parametric_equal_arclen(
            self,
            pos_fun,  # callable: u -> (3,) numpy array
            u0: float,
            u1: float,
            ds: float = 0.002,
            include_end: bool = True,
            dense: int = 4000
    ) -> tuple[np.ndarray, np.ndarray, int]:
        """对参数曲线 r(u) 做等弧长采样。
        
        Args:
            pos_fun (callable): 参数曲线函数 r(u)。
            u0 (float): 参数起始值。
            u1 (float): 参数终止值。
            ds (float, optional): 目标弧长采样间隔. Defaults to 0.002.
            include_end (bool, optional): 结果是否必须包含终点. Defaults to True.
            dense (int, optional): 初始密采样的点数. Defaults to 4000.
        
        Returns:
            tuple: (pos_list, u_eq, n_seg)
                - pos_list: 等弧长采样后的坐标列表 (M, 3)
                - u_eq: 对应的参数值列表 (M,)
                - n_seg: 分段数
        """
        # 1) 先在参数上做均匀密采样，估算弧长
        u_dense = np.linspace(u0, u1, max(1000, int(abs(u1 - u0) * dense)))
        XYZ = np.vstack([np.asarray(pos_fun(ui), dtype=float).reshape(3) for ui in u_dense])
        seg = np.linalg.norm(np.diff(XYZ, axis=0), axis=1)
        s_cum = np.concatenate([[0.0], np.cumsum(seg)])
        L = float(s_cum[-1])
        if L < 1e-12:
            # 退化：几乎零长
            pos_list = XYZ[[0, -1], :] if include_end else XYZ[[0], :]
            u_eq = np.array([u_dense[0], u_dense[-1]]) if include_end else np.array([u_dense[0]])
            return pos_list, u_eq, 0.0

        # 2) 弧长 -> u 的反查，用插值近似
        s2u = interp1d(s_cum, u_dense, kind="linear", bounds_error=False, fill_value=(u_dense[0], u_dense[-1]))

        # 3) 目标等弧长序列
        n_seg = max(1, int(np.ceil(L / ds)))
        s_target = np.linspace(0.0, L, n_seg + 1 if include_end else n_seg)
        u_eq = s2u(s_target)

        # 4) 得到等弧长位置
        pos_list = np.vstack([np.asarray(pos_fun(ui), dtype=float).reshape(3) for ui in u_eq])
        return pos_list, u_eq, n_seg

    def _quat_follow_tangent(self, pos_list, tool_axis="z", up_hint=np.array([0, 0, 1.0])):
        pos = np.asarray(pos_list, dtype=float)
        v = np.gradient(pos, axis=0)  # 中心差分
        v_norm = np.linalg.norm(v, axis=1, keepdims=True) + 1e-12
        t = v / v_norm  # 切向

        up = np.tile(up_hint / (np.linalg.norm(up_hint) + 1e-12), (len(pos), 1))
        # up 与切向几乎共线时换备用 up
        collinear = (np.abs(np.sum(up * t, axis=1)) > 0.98)
        if np.any(collinear):
            alt = np.array([1.0, 0, 0]) if abs(up_hint[0]) < 0.9 else np.array([0, 1.0, 0])
            up[collinear] = alt

        n = np.cross(up, t);
        n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
        b = np.cross(t, n);
        b /= (np.linalg.norm(b, axis=1, keepdims=True) + 1e-12)

        if tool_axis == "z":
            Rm = np.stack([n, b, t], axis=2)  # 列向量
        elif tool_axis == "x":
            Rm = np.stack([t, n, b], axis=2)
        elif tool_axis == "y":
            Rm = np.stack([n, t, b], axis=2)
        else:
            raise ValueError("tool_axis must be in {'x','y','z'}")

        quat_list = np.array([R.from_matrix(Rm[i]).as_quat() for i in range(Rm.shape[0])])
        return quat_list

    def smooth(self, quat_list, pos_list, n_seg):
        positions = []

        pos_list = np.array(pos_list)
        quat_list = np.array(quat_list)
        for quat, pos in zip(quat_list, pos_list):
            position = self.inverse_kinematic(quat, pos)
            positions.append(position)

        positions = np.array(positions)
        # todo: 基于这个positions列表，规划rucking smooth
        q_wp = self.ensure_2d_array(positions)
        grid_n = self.clamp(6 * n_seg, 300, 3000)
        t_list, positions, qd, qdd = self.toppra_time_parameterize(q_wp, self.v_max, self.a_max, self.dt, grid_n)
        return t_list, positions, qd, qdd

    def inverse_kinematic(self, quat, pos):
        rm = R.from_quat(quat).as_matrix()
        inverse_position = self.kinematic_solver.inverse_kinematic(rm, pos, initial_theta=self.nearest_position)
        self.nearest_position = inverse_position
        return inverse_position

    def _q_normalize(self, q):
        q = np.asarray(q, dtype=float)
        n = np.linalg.norm(q)
        if n == 0:
            raise ValueError("zero quaternion")
        return q / n

    def wrap_to_pi(self, q: np.ndarray) -> np.ndarray:
        """将角度包裹到 (-pi, pi]，避免 2π 跳变影响计算。"""
        return (q + np.pi) % (2 * np.pi) - np.pi

    def ensure_2d_array(self, arr: np.ndarray) -> np.ndarray:
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
                                 ) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
        """使用 TOPPRA 对路径进行时间参数化。
        
        Args:
            waypoints (np.ndarray): 路径点数组 (N, dof)。
            v_max (np.ndarray | float): 最大速度约束。
            a_max (np.ndarray | float): 最大加速度约束。
            dt (float, optional): 输出轨迹的时间步长. Defaults to 0.01.
            grid_n (int, optional): TOPPRA 离散化网格点数. Defaults to 400.
        
        Returns:
            tuple: (t, q, qd, qdd)
                - t: 时间戳列表
                - q: 关节位置列表
                - qd: 关节速度列表
                - qdd: 关节加速度列表
        
        Raises:
            ValueError: 如果 waypoints 维度不正确或约束格式错误。
            RuntimeError: 如果 TOPPRA 求解失败。
        """
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

        q = jnt_traj.eval(t)
        qd = jnt_traj.evald(t)
        qdd = jnt_traj.evaldd(t)
        return t.tolist(), q.tolist(), qd.tolist(), qdd.tolist()

