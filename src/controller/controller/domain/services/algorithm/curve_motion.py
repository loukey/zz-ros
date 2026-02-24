from math import pi
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline
from scipy.interpolate import interp1d
from .kinematic_domain_service import KinematicDomainService
from ...utils import KinematicUtils


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
        v_max=None,
        a_max=None,
        j_max=None,
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
        self.v_max = np.asarray(v_max if v_max is not None else [pi / 4] * 6, dtype=float)
        self.a_max = np.asarray(a_max if a_max is not None else [pi / 8] * 6, dtype=float)
        self.j_max = np.asarray(j_max if j_max is not None else [pi / 16] * 6, dtype=float)
        self.dt = float(dt)
        self.kinematic_solver = kinematic_solver
        self.nearest_position = None

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
        """
        # 起点姿态来自起点关节角的正解
        start_quat, start_pos_fk = self.kinematic_solver.get_gripper2base(start_position)
        # 1) 等弧长采样
        pos_list, u_eq, n_seg = self.sample_parametric_equal_arclen(pos_fun, u0, u1, ds, include_end)

        # 2) 姿态生成
        if orientation_mode == "fixed":
            quat_list = np.repeat(KinematicUtils.q_normalize(start_quat)[None, :], len(pos_list), axis=0)

        elif orientation_mode == "slerp":
            if end_position is None:
                raise ValueError("slerp 模式需要提供 end_position 以确定终止姿态。")
            end_quat, _ = self.kinematic_solver.get_gripper2base(end_position)
            q0 = KinematicUtils.q_normalize(start_quat)
            q1 = KinematicUtils.q_normalize(end_quat)
            tau = np.linspace(0, 1, len(pos_list))
            quat_list = np.vstack([KinematicUtils.q_slerp(q0, q1, ti) for ti in tau])

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

    def smooth(self, quat_list: np.ndarray, pos_list: np.ndarray, n_seg: int) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
        positions = []

        pos_list = np.array(pos_list)
        quat_list = np.array(quat_list)
        for quat, pos in zip(quat_list, pos_list):
            position = self.inverse_kinematic(quat, pos)
            if position is None:
                continue
            positions.append(position)

        positions = np.array(positions)
        q_wp = KinematicUtils.ensure_waypoints_2d(positions)
        grid_n = KinematicUtils.clamp(6 * n_seg, 300, 3000)
        t_list, positions, qd, qdd = KinematicUtils.toppra_time_parameterize(q_wp, self.v_max, self.a_max, self.dt, grid_n)
        return t_list, positions, qd, qdd

    def inverse_kinematic(self, quat: np.ndarray, pos: np.ndarray) -> list[float] | None:
        """单点逆运动学求解（利用上一位置作为初值）。"""
        rm = KinematicUtils.quat2rm(quat)
        try:
            inverse_position = self.kinematic_solver.inverse_kinematic(rm, pos, initial_theta=self.nearest_position)
        except ValueError:
            return None
        self.nearest_position = inverse_position
        return inverse_position
