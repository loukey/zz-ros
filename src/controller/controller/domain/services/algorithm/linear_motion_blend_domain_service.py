import numpy as np
from math import pi
from dataclasses import dataclass
from typing import List, Tuple, Optional, Union
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from scipy.spatial.transform import Rotation as R
from .kinematic_domain_service import KinematicDomainService


@dataclass
class ProjectionResult:
    seg_idx: int
    alpha: float


@dataclass
class Piece:
    type: str                  # 'line' or 'arc'
    start: np.ndarray
    end: np.ndarray
    # arc 专用
    center: Optional[np.ndarray] = None
    axis: Optional[np.ndarray] = None
    arc_angle: Optional[float] = None
    radius: Optional[float] = None


class LinearMotionBlendDomainService:

    def __init__(self, kinematic_solver: KinematicDomainService):
        self.kinematic_solver = kinematic_solver
        # TOPPRA 参数
        self.v_max = np.asarray([pi/4] * 6, dtype=float)
        self.a_max = np.asarray([pi/8] * 6, dtype=float)
        self.dt = 0.01
        # 逆运动学初始猜测值
        self.nearest_position = []

    # =========================================================
    # 0) 基础工具：向量、夹角、Rodrigues 旋转
    # =========================================================
    def _norm(self, v: np.ndarray) -> float:
        return float(np.linalg.norm(v))

    def _unit(self, v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
        n = self._norm(v)
        if n < eps:
            return v.copy()
        return v / n

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def _angle_between(self, u: np.ndarray, v: np.ndarray, eps: float = 1e-12) -> float:
        """返回 u 与 v 的夹角 [0, pi]"""
        nu, nv = self._norm(u), self._norm(v)
        if nu < eps or nv < eps:
            return 0.0
        c = float(np.dot(u, v) / (nu * nv))
        c = self._clamp(c, -1.0, 1.0)
        return float(np.arccos(c))

    def _rodrigues_rotate(self, v: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
        """
        Rodrigues 旋转：将向量 v 绕单位轴 axis 旋转 angle(弧度)
        """
        axis = self._unit(axis)
        c = np.cos(angle)
        s = np.sin(angle)
        return v * c + np.cross(axis, v) * s + axis * (np.dot(axis, v)) * (1 - c)


    # =========================================================
    # 1) 四元数工具：normalize + slerp
    #    四元数格式统一为 [x, y, z, w]
    # =========================================================

    def quat_normalize(self, q: np.ndarray, eps: float = 1e-12) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        n = float(np.linalg.norm(q))
        if n < eps:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)  # 单位四元数
        return q / n

    def quat_slerp(self, q0: np.ndarray, q1: np.ndarray, t: float, eps: float = 1e-12) -> np.ndarray:
        """
        四元数球面线性插值 SLERP
        - q0, q1: [x,y,z,w]
        - t: 0~1
        """
        q0 = self.quat_normalize(q0, eps)
        q1 = self.quat_normalize(q1, eps)

        # 最短弧：点积为负则翻转 q1
        dot = float(np.dot(q0, q1))
        if dot < 0.0:
            q1 = -q1
            dot = -dot

        dot = self._clamp(dot, -1.0, 1.0)

        # 很接近时用线性插值避免数值问题
        if dot > 0.9995:
            q = (1.0 - t) * q0 + t * q1
            return self.quat_normalize(q, eps)

        theta = np.arccos(dot)
        sin_theta = np.sin(theta)
        if abs(sin_theta) < eps:
            return q0

        w0 = np.sin((1.0 - t) * theta) / sin_theta
        w1 = np.sin(t * theta) / sin_theta
        q = w0 * q0 + w1 * q1
        return self.quat_normalize(q, eps)


    # =========================================================
    # 2) 点投影到“原始折线”以获得段索引与 alpha（用于姿态 SLERP）
    # =========================================================
    def project_point_to_polyline_segment(self, p: np.ndarray, poly: np.ndarray, eps: float = 1e-12) -> ProjectionResult:
        """
        把点 p 投影到折线 poly 上，返回最近点所在段 seg_idx 以及段内比例 alpha。
        seg_idx: 0..N-2, 表示段 [P[i], P[i+1]]
        alpha: 0..1, 表示最近点 = (1-alpha)*P[i] + alpha*P[i+1]
        """
        P = np.asarray(poly, dtype=float)
        p = np.asarray(p, dtype=float)

        if P.shape[0] < 2:
            return ProjectionResult(seg_idx=0, alpha=0.0)

        best_d2 = float("inf")
        best_i = 0
        best_a = 0.0

        for i in range(P.shape[0] - 1):
            a = P[i]
            b = P[i + 1]
            ab = b - a
            lab2 = float(np.dot(ab, ab))
            if lab2 < eps:
                alpha = 0.0
                proj = a
            else:
                alpha = float(np.dot(p - a, ab) / lab2)
                alpha = self._clamp(alpha, 0.0, 1.0)
                proj = a + alpha * ab

            d2 = float(np.dot(p - proj, p - proj))
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
                best_a = alpha

        return ProjectionResult(seg_idx=best_i, alpha=best_a)


    # =========================================================
    # 3) 构建“直线段 + 圆弧段”的几何分段（多拐点 fillet blending）
    # =========================================================
    def build_blended_pieces_fillet(
        self,
        points: np.ndarray,
        radii: Union[float, np.ndarray],
        min_turn_angle_deg: float = 2.0,
        eps: float = 1e-9
    ) -> List[Piece]:
        """
        输入原始路点 points (N,3)，在每个中间点做圆弧 fillet blending，
        输出按顺序拼接的几何段 pieces（line/arc）。

        radii:
        - 标量：所有拐点用同一个半径
        - shape(N,)：radii[i] 对应拐点 i 的半径（首尾忽略）
        - shape(N-2,)：对应拐点 1..N-2
        """
        P = np.asarray(points, dtype=float)
        N = P.shape[0]
        if N < 2:
            raise ValueError("points 至少 2 个点")

        # 解析半径数组到 shape(N,)
        if np.isscalar(radii):
            r_corner = np.full(N, float(radii), dtype=float)
        else:
            r_arr = np.asarray(radii, dtype=float).reshape(-1)
            if len(r_arr) == N:
                r_corner = r_arr
            elif len(r_arr) == N - 2:
                r_corner = np.zeros(N, dtype=float)
                r_corner[1:-1] = r_arr
            else:
                raise ValueError("radii 长度必须为 标量 / N / (N-2)")

        min_turn = np.deg2rad(min_turn_angle_deg)

        pieces: List[Piece] = []
        current = P[0].copy()

        for i in range(1, N - 1):
            Pm1, Pi, Pp1 = P[i - 1], P[i], P[i + 1]
            v1 = Pi - Pm1
            v2 = Pp1 - Pi
            L1, L2 = self._norm(v1), self._norm(v2)

            # 段太短：不做圆角，直接连到 Pi
            if L1 < eps or L2 < eps:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue

            d1 = self._unit(Pm1 - Pi)  # 指向前一段
            d2 = self._unit(Pp1 - Pi)  # 指向后一段
            theta = self._angle_between(d1, d2, eps=eps)

            # 近共线或近 180°：不做圆角
            if theta < min_turn or abs(np.pi - theta) < min_turn:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue

            r_des = float(max(0.0, r_corner[i]))
            if r_des < eps:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue

            tan_half = np.tan(theta * 0.5)
            if not np.isfinite(tan_half) or tan_half < eps:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue

            # 切点距离 t = r * tan(theta/2)
            t_des = r_des * tan_half

            # t 必须小于两侧段长，否则缩小半径（k 防止贴边）
            k = 0.9
            t_max = k * min(L1, L2)
            if t_des > t_max:
                t_use = t_max
                r_use = t_use / tan_half
            else:
                t_use = t_des
                r_use = r_des

            T1 = Pi + d1 * t_use
            T2 = Pi + d2 * t_use

            # 圆心：在角平分线上
            bis = d1 + d2
            if self._norm(bis) < eps:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue
            b = self._unit(bis)

            sin_half = np.sin(theta * 0.5)
            if abs(sin_half) < eps:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue

            h = r_use / sin_half
            C = Pi + b * h

            v_start = T1 - C
            v_end = T2 - C
            axis = np.cross(v_start, v_end)
            if self._norm(axis) < eps:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue
            axis = self._unit(axis)

            # 弧角（最短旋转）
            cross_mag = self._norm(np.cross(self._unit(v_start), self._unit(v_end)))
            dot_val = self._clamp(float(np.dot(self._unit(v_start), self._unit(v_end))), -1.0, 1.0)
            arc_angle = float(np.arctan2(cross_mag, dot_val))  # [0, pi]

            radius = 0.5 * (self._norm(v_start) + self._norm(v_end))

            # 拼接：line current->T1 + arc T1->T2
            if self._norm(T1 - current) > 1e-12:
                pieces.append(Piece("line", current, T1.copy()))
            pieces.append(Piece("arc", T1.copy(), T2.copy(), center=C.copy(), axis=axis.copy(),
                                arc_angle=arc_angle, radius=radius))

            current = T2.copy()

        # 末尾连接到终点
        if self._norm(P[-1] - current) > 1e-12:
            pieces.append(Piece("line", current, P[-1].copy()))

        return pieces


    # =========================================================
    # 4) 对 pieces 按弧长步长 step 等距采样 → 得到 pos_list
    # =========================================================
    def sample_pieces_by_step(
        self,
        pieces: List[Piece],
        step: float,
        include_last: bool = True,
        eps: float = 1e-12
    ) -> Tuple[np.ndarray, List[int]]:
        """
        对 pieces 做等距采样（每 step 一点），返回：
        - pos: (M,3) 采样点列（相邻段拼接处去重）
        - n_piece: 每个 piece 内部采样点数量（用于调试；不是你要的 n_seg）
        """
        if step <= 0:
            raise ValueError("step 必须 > 0")

        pts: List[np.ndarray] = []
        n_piece: List[int] = []

        def append_pt(p: np.ndarray):
            if len(pts) == 0 or self._norm(pts[-1] - p) > 1e-10:
                pts.append(p)

        for pc in pieces:
            if pc.type == "line":
                a, b = pc.start, pc.end
                L = self._norm(b - a)
                if L < eps:
                    append_pt(a.copy())
                    n_piece.append(1)
                    continue

                # 这里的策略：保证端点都有；中间点尽量接近 step
                n = int(np.floor(L / step)) + 1
                n = max(n, 2)
                t_list = np.linspace(0.0, 1.0, n, endpoint=True)

                local = 0
                for t in t_list:
                    p = (1 - t) * a + t * b
                    append_pt(p)
                    local += 1
                n_piece.append(local)

            elif pc.type == "arc":
                T1, T2 = pc.start, pc.end
                C, axis = pc.center, pc.axis
                arc_angle = float(pc.arc_angle)
                r = float(pc.radius)

                if r < eps or arc_angle < eps:
                    append_pt(T1.copy())
                    n_piece.append(1)
                    continue

                arc_len = r * arc_angle
                n = int(np.floor(arc_len / step)) + 1
                n = max(n, 2)
                a_list = np.linspace(0.0, arc_angle, n, endpoint=True)

                v0 = T1 - C
                local = 0
                for ang in a_list:
                    v = self._rodrigues_rotate(v0, axis, float(ang))
                    p = C + v
                    append_pt(p)
                    local += 1
                n_piece.append(local)

            else:
                raise ValueError(f"未知 piece 类型: {pc.type}")

        if include_last and len(pieces) > 0:
            append_pt(pieces[-1].end.copy())

        return np.vstack(pts), n_piece

    def inverse_kinematic(self, quat, pos):
        """单点逆运动学求解（利用上一位置作为初值）。
        
        Args:
            quat: 四元数 [x, y, z, w]。
            pos: 位置 [x, y, z]。
            
        Returns:
            list[float]: 关节角度列表。
        """
        rm = R.from_quat(quat).as_matrix()
        inverse_position = self.kinematic_solver.inverse_kinematic(rm, pos, initial_theta=self.nearest_position if self.nearest_position else None)
        self.nearest_position = inverse_position
        return inverse_position

    def ensure_2d_array(self, arr: np.ndarray) -> np.ndarray:
        """将输入转换为二维数组 (N, dof) 并做基本校验。
        
        Args:
            arr (np.ndarray): 输入数组。
            
        Returns:
            np.ndarray: 二维数组 (N, 6)。
            
        Raises:
            ValueError: 当路标数少于 2 时抛出。
        """
        arr = np.asarray(arr, dtype=float)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.shape[0] < 2:
            raise ValueError("至少需要 2 个路点")
        return arr

    def toppra_time_parameterize(self,
        waypoints: np.ndarray,
        v_max: np.ndarray | float = np.asarray([pi/4] * 6, dtype=float),
        a_max: np.ndarray | float = np.asarray([pi/8] * 6, dtype=float),
        dt: float = 0.01,
        grid_n: int = 800
        ) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
        """使用 TOPPRA 进行时间参数化。
        
        Args:
            waypoints (np.ndarray): 路径点 (N, dof)。
            v_max (np.ndarray | float): 最大速度。
            a_max (np.ndarray | float): 最大加速度。
            dt (float, optional): 时间步长. Defaults to 0.01.
            grid_n (int, optional): 网格点数. Defaults to 800.
            
        Returns:
            tuple: (t, q, qd, qdd).
            
        Raises:
            ValueError: 如果参数形状不正确。
            RuntimeError: 如果求解失败。
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

        q   = jnt_traj.eval(t)
        qd  = jnt_traj.evald(t)
        qdd = jnt_traj.evaldd(t)
        return t.tolist(), q.tolist(), qd.tolist(), qdd.tolist()

    # =========================================================
    # 5) 主接口：blend + 等距采样 + 姿态（四元数）输出
    #    你要的 n_seg = 最终点数（TOPP-RA 输入点数）
    # =========================================================
    def move_with_blend(
        self,
        input_positions,
        step: float = 0.02,
        radii: Union[float, np.ndarray] = 0.03,
        min_turn_angle_deg: float = 2.0,
        include_last: bool = True,
    ) -> Tuple[List[List[float]], List[List[float]], int]:
        """
        radii:jiaorongbanjing
        输出：
        - quat_list: List[[x,y,z,w]]
        - pos_list : List[[x,y,z]]
        - n_seg    : int = len(pos_list) = len(quat_list)
        """
        print(input_positions)
        quat_wp = []
        pos_wp = []
        for position in input_positions:
            quat, pos = self.kinematic_solver.get_gripper2base(position)
            quat_wp.append(quat)
            pos_wp.append(pos)
        P = np.asarray(pos_wp, dtype=float)
        Q = np.asarray(quat_wp, dtype=float)
        if P.ndim != 2 or P.shape[1] != 3:
            raise ValueError("pos_wp 必须是 (N,3)")
        if Q.ndim != 2 or Q.shape[1] != 4:
            raise ValueError("quat_wp 必须是 (N,4) 且格式为 [x,y,z,w]")
        if P.shape[0] != Q.shape[0]:
            raise ValueError("pos_wp 与 quat_wp 点数必须一致")
        if P.shape[0] < 2:
            raise ValueError("至少需要 2 个路点")

        # 归一化路点四元数
        Qn = np.vstack([self.quat_normalize(Q[i]) for i in range(Q.shape[0])])

        # 1) 生成几何拼接分段（line + arc）
        pieces = self.build_blended_pieces_fillet(
            points=P,
            radii=radii,
            min_turn_angle_deg=min_turn_angle_deg
        )

        # 2) 沿拼接路径等距采样（step=2cm）
        pos_samp, _n_piece_debug = self.sample_pieces_by_step(
            pieces=pieces,
            step=step,
            include_last=include_last
        )

        # 3) 姿态：将每个采样点投影回“原始折线”段上 → SLERP
        quat_samp = []
        for p in pos_samp:
            pr = self.project_point_to_polyline_segment(p, P)
            i, a = pr.seg_idx, pr.alpha
            qs = self.quat_slerp(Qn[i], Qn[i + 1], a)
            quat_samp.append(qs)

        pos_list = pos_samp.tolist()
        quat_list = [q.tolist() for q in quat_samp]
        positions = []
        for quat, pos in zip(quat_list, pos_list):
            position = self.inverse_kinematic(quat, pos)
            positions.append(position)
        positions = np.array(positions)
        # todo: 基于这个positions列表，规划rucking smooth

        q_wp = self.ensure_2d_array(positions)
        n_seg = len(pos_list)  # ✅ 按你定义：最终 TOPP-RA 输入点数
        t_list, positions, qd, qdd= self.toppra_time_parameterize(q_wp, self.v_max, self.a_max, self.dt, n_seg)
        return t_list, positions, qd, qdd
