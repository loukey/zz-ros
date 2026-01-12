import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Union


# =========================================================
# 0) 基础工具：向量、夹角、Rodrigues 旋转
# =========================================================

def _norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))

def _unit(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = _norm(v)
    if n < eps:
        return v.copy()
    return v / n

def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def _angle_between(u: np.ndarray, v: np.ndarray, eps: float = 1e-12) -> float:
    """返回 u 与 v 的夹角 [0, pi]"""
    nu, nv = _norm(u), _norm(v)
    if nu < eps or nv < eps:
        return 0.0
    c = float(np.dot(u, v) / (nu * nv))
    c = _clamp(c, -1.0, 1.0)
    return float(np.arccos(c))

def _rodrigues_rotate(v: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """
    Rodrigues 旋转：将向量 v 绕单位轴 axis 旋转 angle(弧度)
    """
    axis = _unit(axis)
    c = np.cos(angle)
    s = np.sin(angle)
    return v * c + np.cross(axis, v) * s + axis * (np.dot(axis, v)) * (1 - c)


# =========================================================
# 1) 四元数工具：normalize + slerp
#    四元数格式统一为 [x, y, z, w]
# =========================================================

def quat_normalize(q: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    if n < eps:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)  # 单位四元数
    return q / n

def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float, eps: float = 1e-12) -> np.ndarray:
    """
    四元数球面线性插值 SLERP
    - q0, q1: [x,y,z,w]
    - t: 0~1
    """
    q0 = quat_normalize(q0, eps)
    q1 = quat_normalize(q1, eps)

    # 最短弧：点积为负则翻转 q1
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = _clamp(dot, -1.0, 1.0)

    # 很接近时用线性插值避免数值问题
    if dot > 0.9995:
        q = (1.0 - t) * q0 + t * q1
        return quat_normalize(q, eps)

    theta = np.arccos(dot)
    sin_theta = np.sin(theta)
    if abs(sin_theta) < eps:
        return q0

    w0 = np.sin((1.0 - t) * theta) / sin_theta
    w1 = np.sin(t * theta) / sin_theta
    q = w0 * q0 + w1 * q1
    return quat_normalize(q, eps)


# =========================================================
# 2) 点投影到“原始折线”以获得段索引与 alpha（用于姿态 SLERP）
# =========================================================

@dataclass
class ProjectionResult:
    seg_idx: int
    alpha: float

def project_point_to_polyline_segment(p: np.ndarray, poly: np.ndarray, eps: float = 1e-12) -> ProjectionResult:
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
            alpha = _clamp(alpha, 0.0, 1.0)
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

def build_blended_pieces_fillet(
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
        L1, L2 = _norm(v1), _norm(v2)

        # 段太短：不做圆角，直接连到 Pi
        if L1 < eps or L2 < eps:
            if _norm(Pi - current) > 1e-12:
                pieces.append(Piece("line", current, Pi.copy()))
                current = Pi.copy()
            continue
        if np.isscalar(radii):
            r_des = float(radii)
        else:
            r_des = float(radii[i])

        if r_des <= 0:
            pieces.append(Piece("line", current, Pi.copy()))
            current = Pi.copy()
            continue
        u_in  = _unit(Pi - Pm1)   # A -> B（入射方向）
        u_out = _unit(Pp1 - Pi)   # B -> C（出射方向）

        phi = _angle_between(u_in, u_out, eps=eps)  # 0..pi

        # 几乎直行 或 近 180° 掉头：不做圆角
        if phi < min_turn or abs(np.pi - phi) < min_turn:
            if _norm(Pi - current) > 1e-12:
                pieces.append(Piece("line", current, Pi.copy()))
                current = Pi.copy()
            continue

        # === 以下是圆角几何（与方向定义必须保持一致） ===
        tan_half = np.tan(phi * 0.5)
        if not np.isfinite(tan_half) or tan_half < eps:
            if _norm(Pi - current) > 1e-12:
                pieces.append(Piece("line", current, Pi.copy()))
                current = Pi.copy()
            continue

        # 期望切点距离
        t_des = r_des * tan_half

        # 不允许切点超过两侧段长
        k = 0.9
        t_max = k * min(L1, L2)
        t_use = min(t_des, t_max)
        r_use = t_use / tan_half

        # === 正确的切点（非常关键） ===
        T1 = Pi - u_in  * t_use   # 从 B 沿 AB 方向“回退”
        T2 = Pi + u_out * t_use   # 从 B 沿 BC 方向“前进”

        # === 内侧角平分线 ===
        bis = u_in + u_out
        if _norm(bis) < eps:
            if _norm(Pi - current) > 1e-12:
                pieces.append(Piece("line", current, Pi.copy()))
                current = Pi.copy()
            continue
        b = _unit(bis)

        sin_half = np.sin(phi * 0.5)
        if abs(sin_half) < eps:
            if _norm(Pi - current) > 1e-12:
                pieces.append(Piece("line", current, Pi.copy()))
                current = Pi.copy()
            continue

        h = r_use / sin_half
        C = Pi + b * h

        v_start = T1 - C
        v_end = T2 - C
        axis = np.cross(v_start, v_end)
        if _norm(axis) < eps:
            if _norm(Pi - current) > 1e-12:
                pieces.append(Piece("line", current, Pi.copy()))
                current = Pi.copy()
            continue
        axis = _unit(axis)

        # 弧角（最短旋转）
        cross_mag = _norm(np.cross(_unit(v_start), _unit(v_end)))
        dot_val = _clamp(float(np.dot(_unit(v_start), _unit(v_end))), -1.0, 1.0)
        arc_angle = float(np.arctan2(cross_mag, dot_val))  # [0, pi]

        radius = 0.5 * (_norm(v_start) + _norm(v_end))

        # 拼接：line current->T1 + arc T1->T2
        if _norm(T1 - current) > 1e-12:
            pieces.append(Piece("line", current, T1.copy()))
        pieces.append(Piece("arc", T1.copy(), T2.copy(), center=C.copy(), axis=axis.copy(),
                            arc_angle=arc_angle, radius=radius))

        current = T2.copy()

    # 末尾连接到终点
    if _norm(P[-1] - current) > 1e-12:
        pieces.append(Piece("line", current, P[-1].copy()))

    return pieces


# =========================================================
# 4) 对 pieces 按弧长步长 step 等距采样 → 得到 pos_list
# =========================================================

def sample_pieces_by_step(
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
        if len(pts) == 0 or _norm(pts[-1] - p) > 1e-10:
            pts.append(p)

    for pc in pieces:
        if pc.type == "line":
            a, b = pc.start, pc.end
            L = _norm(b - a)
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
                v = _rodrigues_rotate(v0, axis, float(ang))
                p = C + v
                append_pt(p)
                local += 1
            n_piece.append(local)

        else:
            raise ValueError(f"未知 piece 类型: {pc.type}")

    if include_last and len(pieces) > 0:
        append_pt(pieces[-1].end.copy())

    return np.vstack(pts), n_piece


# =========================================================
# 5) 主接口：blend + 等距采样 + 姿态（四元数）输出
#    你要的 n_seg = 最终点数（TOPP-RA 输入点数）
# =========================================================

def blend_and_sample_cartesian_pose(
    pos_wp: np.ndarray,
    quat_wp: np.ndarray,
    step: float = 0.02,
    radii: Union[float, np.ndarray] = 0.03,
    min_turn_angle_deg: float = 2.0,
    include_last: bool = True,
) -> Tuple[List[List[float]], List[List[float]], int]:
    """
    输出：
    - quat_list: List[[x,y,z,w]]
    - pos_list : List[[x,y,z]]
    - n_seg    : int = len(pos_list) = len(quat_list)
      （这就是你说的“最终输入给 TOPP-RA 的采样数量”）
    """
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
    Qn = np.vstack([quat_normalize(Q[i]) for i in range(Q.shape[0])])

    # 1) 生成几何拼接分段（line + arc）
    pieces = build_blended_pieces_fillet(
        points=P,
        radii=radii,
        min_turn_angle_deg=min_turn_angle_deg
    )

    # 2) 沿拼接路径等距采样（step=2cm）
    pos_samp, _n_piece_debug = sample_pieces_by_step(
        pieces=pieces,
        step=step,
        include_last=include_last
    )

    # 3) 姿态：将每个采样点投影回“原始折线”段上 → SLERP
    quat_samp = []
    for p in pos_samp:
        pr = project_point_to_polyline_segment(p, P)
        i, a = pr.seg_idx, pr.alpha
        qs = quat_slerp(Qn[i], Qn[i + 1], a)
        quat_samp.append(qs)

    pos_list = pos_samp.tolist()
    quat_list = [q.tolist() for q in quat_samp]
    n_seg = len(pos_list)  # ✅ 按你定义：最终 TOPP-RA 输入点数

    return quat_list, pos_list, n_seg


# =========================================================
# 6) 示例
# =========================================================
if __name__ == "__main__":
    pos_wp = np.array([
        [-0.122,  0.648,  0.219],
        [0.286, 0.264,  0.209],
        [0.597, 0.047, 0.189]
    ], dtype=float)

    quat_wp = np.array([
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.2, 0.98],
        [0.0, 0.0, 0.4, 0.92]

    ], dtype=float)
    quat_wp = np.vstack([quat_normalize(q) for q in quat_wp])

    quat_list, pos_list, n_seg = blend_and_sample_cartesian_pose(
        pos_wp=pos_wp,
        quat_wp=quat_wp,
        step=0.02,       # 2cm
        radii=0.03,      # 3cm 圆角半径
        min_turn_angle_deg=2.0,
        include_last=True
    )

    print("n_seg =", n_seg)
    print("len(pos_list) =", len(pos_list))
    print("len(quat_list) =", len(quat_list))
    print(f"Total points (n_seg) = {n_seg}")
    print("-" * 60)

    for i, (p, q) in enumerate(zip(pos_list, quat_list)):
        print(
            f"[{i:04d}] "
            f"pos = ({p[0]: .6f}, {p[1]: .6f}, {p[2]: .6f}) | "
            f"quat = ({q[0]: .6f}, {q[1]: .6f}, {q[2]: .6f}, {q[3]: .6f})"
        )