import numpy as np
from math import pi
from dataclasses import dataclass
from typing import List, Tuple, Optional, Union
from .kinematic_domain_service import KinematicDomainService
from .trajectory_domain_service import SCurve
from ...utils import KinematicUtils

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

    def __init__(
        self, 
        kinematic_solver: KinematicDomainService,
        s_curve: SCurve,
        ):
        self.kinematic_solver = kinematic_solver
        self.s_curve = s_curve
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

    def _tangency_cost(self, C: np.ndarray, T1: np.ndarray, T2: np.ndarray, u_in: np.ndarray, u_out: np.ndarray, eps: float = 1e-9) -> float:
        """计算切线代价：切点方向与入射/出射方向的点积之和。"""
        v1 = T1 - C
        v2 = T2 - C

        if self._norm(v1) < eps or self._norm(v2) < eps:
            return float("inf")

        v1u = self._unit(v1)
        v2u = self._unit(v2)

        return abs(float(np.dot(v1u, u_in))+abs(float(np.dot(v2u, u_out))))

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
        k_radii:float = 0.1,
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

            u_in  = self._unit(Pi - Pm1)   # A -> B（入射方向）
            u_out = self._unit(Pp1 - Pi)   # B -> C（出射方向）

            phi = self._angle_between(u_in, u_out, eps=eps)  # 0..pi

            # 几乎直行 或 近 180° 掉头：不做圆角
            if phi < min_turn or abs(np.pi - phi) < min_turn:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue

            # === 以下是圆角几何（与方向定义必须保持一致） ===
            tan_half = np.tan(phi * 0.5)
            if not np.isfinite(tan_half) or tan_half < eps:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue
            
            zone_min = 0.005
            zone_max = 0.15
            Lmin = min(L1,L2)
            # 期望切点距离
            t_des = k_radii * Lmin
            t_use = float(np.clip(t_des, zone_min, zone_max))

            r_use = t_use / tan_half

            # === 正确的切点（非常关键） ===
            T1 = Pi - u_in  * t_use   # 从 B 沿 AB 方向“回退”
            T2 = Pi + u_out * t_use   # 从 B 沿 BC 方向“前进”

            # === 内侧角平分线 ===
            bis = u_out - u_in
            if self._norm(bis) < eps:
                if self._norm(Pi - current) > 1e-12:
                    pieces.append(Piece("line", current, Pi.copy()))
                    current = Pi.copy()
                continue
            b = self._unit(bis)

            sin_half = np.sin(phi * 0.5)
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
        eps: float = 1e-12,
        err_max: float = 0.0005,
        min_pts_arc: int = 8
    ) -> Tuple[np.ndarray, List[int]]:
        """
        对 pieces 做等距采样（每 step 一点），返回：
        - pos: (M,3) 采样点列（相邻段拼接处去重）
        - n_piece: 每个 piece 内部采样点数量（用于调试；不是你要的 n_seg）
        """
        if step <= 0:
            raise ValueError("step 必须 > 0")
        
        if err_max <= 0:
            raise ValueError("err_max 必须 > 0")

        min_pts_arc = int(min_pts_arc)
        if min_pts_arc < 2:
            raise ValueError("min_pts_arc 必须 > =2")

        pts: List[np.ndarray] = []
        n_piece: List[int] = []

        def append_pt(p: np.ndarray):
            if len(pts) == 0 or self._norm(pts[-1] - p) > 1e-10:
                pts.append(p)
        
        def _arc_n_from_chord_error(r: float, arc_angle: float) -> int:
            """
            由最大弦误差 err_max 推出圆弧所需点数 n
            弦误差 e = r*(1-cos(dtheta/2)) <= err_max
            => dtheta <= 2*acos(1-err_max/r)
            """
            r = float(abs(r))
            ang = float(abs(arc_angle))
            if r < eps or ang < eps:
                return 2

            x = 1.0 - (err_max / r)
            x = float(np.clip(x, -1.0, 1.0))
            dtheta = 2.0 * float(np.arccos(x))
            dtheta = max(dtheta, 1e-6)

            n = int(np.ceil(ang / dtheta)) + 1
            return max(n, min_pts_arc)

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

                n = _arc_n_from_chord_error(r, arc_angle)
                
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

    def inverse_kinematic(self, quat, pos, similar_position=None):
        """单点逆运动学求解（利用上一位置作为初值）。

        Args:
            quat: 四元数 [x, y, z, w]。
            pos: 位置 [x, y, z]。
            similar_position: 相似位置（用于逆运动学初始猜测）。

        Returns:
            list[float] | None: 关节角度列表，无解时返回 None。
        """
        rm = KinematicUtils.quat2rm(quat)
        try:
            inverse_position = self.kinematic_solver.inverse_kinematic(rm, pos, initial_theta=similar_position)
        except ValueError:
            return None
        return inverse_position

    # =========================================================
    # 5) 主接口：blend + 等距采样 + 姿态（四元数）输出
    #    你要的 n_seg = 最终点数（TOPP-RA 输入点数）
    # =========================================================
    def move_with_blend(
        self,
        input_positions,
        step: float = 0.005,
        k_radii: float = 0.25,
        min_turn_angle_deg: float = 2.0,
        include_last: bool = True,
    ) -> Tuple[List[List[float]], List[List[float]], int]:
        """
        radii: 圆角融合半径
        输出：
        - quat_list: List[[x,y,z,w]]
        - pos_list : List[[x,y,z]]
        - n_seg    : int = len(pos_list) = len(quat_list)
        """
        
        self.nearest_position = input_positions[0]
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
        Qn = np.vstack([KinematicUtils.q_normalize(Q[i]) for i in range(Q.shape[0])])

        # 1) 生成几何拼接分段（line + arc）
        pieces = self.build_blended_pieces_fillet(
            points=P,
            k_radii=k_radii,
            min_turn_angle_deg=min_turn_angle_deg
        )

        # 2) 沿拼接路径等距采样（step=2cm）
        pos_samp, _n_piece_debug = self.sample_pieces_by_step(
            pieces=pieces,
            step=step,
            include_last=include_last,
            err_max=0.0005,
            min_pts_arc=8
        )

        # 3) 姿态：将每个采样点投影回“原始折线”段上 → SLERP
        quat_samp = []
        for p in pos_samp:
            pr = self.project_point_to_polyline_segment(p, P)
            i, a = pr.seg_idx, pr.alpha
            qs = KinematicUtils.q_slerp(Qn[i], Qn[i + 1], a)
            quat_samp.append(qs)

        pos_list = pos_samp.tolist()
        quat_list = [q.tolist() for q in quat_samp]
        
        s_curve_positions = np.zeros((0, 6))
        for position_index in range(len(input_positions) - 1):
            start_position = input_positions[position_index]
            end_position = input_positions[position_index + 1]
            _, _, _, positions = self.s_curve.planning(start_position, end_position)
            s_curve_positions = np.concatenate((s_curve_positions, positions), axis=0)

        # 对齐长度：对每一列分别进行线性插值重采样
        if len(s_curve_positions) != len(quat_list):
            if len(s_curve_positions) > 1:
                old_indices = np.linspace(0, 1, len(s_curve_positions))
                new_indices = np.linspace(0, 1, len(quat_list))
                new_s_curve_positions = np.zeros((len(quat_list), 6))
                for j in range(6):
                    new_s_curve_positions[:, j] = np.interp(new_indices, old_indices, s_curve_positions[:, j])
                s_curve_positions = new_s_curve_positions
            else:
                # 极端情况处理
                s_curve_positions = np.tile(s_curve_positions[0], (len(quat_list), 1))
        positions = []
        
        for i in range(len(quat_list)):
            position = self.inverse_kinematic(quat_list[i], pos_list[i], s_curve_positions[i].tolist())
            if position is None:
                continue
            positions.append(position)
        positions = np.array(positions)

        q_wp = KinematicUtils.ensure_waypoints_2d(positions)
        n_seg = len(pos_list)
        grid_n = KinematicUtils.clamp(6 * n_seg, 300, 3000)
        t_list, positions, qd, qdd = KinematicUtils.toppra_time_parameterize(q_wp, self.v_max, self.a_max, self.dt, grid_n)
        
        return t_list, positions, qd, qdd
