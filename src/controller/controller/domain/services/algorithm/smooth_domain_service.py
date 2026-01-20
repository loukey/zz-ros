"""
轨迹平滑领域服务 - Domain/Algorithm层
负责所有轨迹的平滑处理算法
"""
import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
from scipy.interpolate import interp1d
from typing import List, Tuple, Any
from math import pi
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo


class SmoothDomainService:
    """轨迹平滑服务。
    
    提供多种平滑算法：
    1. Cubic Spline + Savitzky-Golay 滤波（默认，旧版）
    2. TOPPRA 时间参数化平滑（可选，旧版测试方法）
    
    适用场景：
    - 示教轨迹播放
    - 多点轨迹规划
    - 任何需要轨迹平滑的场景
    """
    
    def __init__(self):
        """初始化平滑服务。"""
        pass
    
    def smooth_trajectory(
        self, 
        angles_list: List[List[float]], 
        method: str = "spline_savgol",
        **kwargs: Any
    ) -> List[List[float]]:
        """平滑轨迹（统一接口）。
        
        Args:
            angles_list (List[List[float]]): 原始轨迹 [[θ1,...,θ6], ...]。
            method (str, optional): 平滑方法 ("spline_savgol" 或 "toppra"). Defaults to "spline_savgol".
            **kwargs: 各算法的特定参数。
        
        Returns:
            List[List[float]]: 平滑后的轨迹点列表。
            
        Raises:
            ValueError: 如果指定了未知的平滑方法。
        """
        if method == "spline_savgol":
            return self.spline_then_savgol(angles_list, **kwargs)
        elif method == "toppra":
            return self.toppra_smooth(angles_list, **kwargs)
        else:
            raise ValueError(f"未知的平滑方法: {method}")
    
    def spline_then_savgol(
        self,
        angles_list: List[List[float]],
        upsample: int = 5,
        sg_window: int = 211,
        sg_poly: int = 3
    ) -> List[List[float]]:
        """Cubic Spline + Savitzky-Golay 滤波（参考旧版）。
        
        流程：
        1. Savitzky-Golay 平滑原始数据
        2. Cubic Spline 上采样插值
        3. 重采样回原始点数
        4. 再次 Savitzky-Golay 平滑
        
        Args:
            angles_list (List[List[float]]): 原始轨迹 N×6。
            upsample (int, optional): 上采样倍数. Defaults to 5.
            sg_window (int, optional): Savgol 滤波窗口（必须是奇数）. Defaults to 211.
            sg_poly (int, optional): Savgol 多项式阶数. Defaults to 3.
        
        Returns:
            List[List[float]]: 平滑后的轨迹列表。
        """
        arr = np.asarray(angles_list, dtype=float)
        N, D = arr.shape
        
        # 确保窗口大小不超过数据点数
        sg_window = min(sg_window, N if N % 2 == 1 else N - 1)
        if sg_window < sg_poly + 2:
            sg_window = sg_poly + 2
            if sg_window % 2 == 0:
                sg_window += 1
        
        # 1. Savitzky–Golay平滑（先平滑）
        smoothed = savgol_filter(
            arr,
            window_length=sg_window,
            polyorder=sg_poly,
            axis=0
        )
        
        # 2. 三次样条插值（再插值）
        t = np.arange(N)
        t_new = np.linspace(0, N - 1, N * upsample)
        interp = np.empty((len(t_new), D), dtype=float)
        
        for j in range(D):
            cs = CubicSpline(t, smoothed[:, j])
            interp[:, j] = cs(t_new)
        
        # 3. 重采样回原始点数
        indices = np.linspace(0, len(t_new) - 1, N).astype(int)
        resampled = interp[indices, :]
        
        # 4. 再次平滑
        resampled = savgol_filter(
            resampled,
            window_length=sg_window,
            polyorder=sg_poly,
            axis=0
        )
        
        return resampled.tolist()
    
    def toppra_smooth(self,
        waypoints: np.ndarray,
        v_max: np.ndarray | float,
        a_max: np.ndarray | float,
        dt: float = 0.01,
        grid_n: int = 800
    ) -> List[List[float]]:
        """使用 TOPPRA 进行平滑处理。
        
        Args:
            waypoints (np.ndarray): 路径点 (N, dof)。
            v_max (np.ndarray | float): 最大速度。
            a_max (np.ndarray | float): 最大加速度。
            dt (float, optional): 时间步长. Defaults to 0.01.
            grid_n (int, optional): 网格数. Defaults to 800.
        
        Returns:
            list: 平滑后的关节位置列表。
            
        Raises:
            ValueError: 参数错误。
            RuntimeError: 求解失败。
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
        return q.tolist()

    def remove_redundant_points(self, q_teach: List[List[float]], eps: float = 1e-4) -> List[List[float]]:
        """改进版去除重复点：保留“任意一个关节角变化超过阈值”的点。
        
        Args:
            q_teach (List[List[float]]): 示教点列表。
            eps (float, optional): 变化阈值. Defaults to 1e-4.
        
        Returns:
            List[List[float]]: 去重后的点列表。
        """
        q_teach = np.asarray(q_teach, dtype=float)
        dq = np.abs(np.diff(q_teach, axis=0))  # 相邻关节角绝对差
        moving_mask = np.any(dq > eps, axis=1)  # 只要任意一轴动了就认为在动
        mask = np.hstack([[True], moving_mask])  # 保留第一个点
        return q_teach[mask].tolist()
    # ------------------------------------------------------------
    def resample_equal_arclen(self, q_points: List[List[float]], step: float = 0.002) -> List[List[float]]:
        """按弧长重采样，使路径在关节空间中分布均匀。
        
        原理：
        - 计算相邻点的欧氏距离（弧长）
        - 将弧长积分得到累计s
        - 对s均匀取样后，用插值函数生成新点
        
        Args:
            q_points (List[List[float]]): 原始点列表。
            step (float, optional): 重采样步长. Defaults to 0.002.
        
        Returns:
            List[List[float]]: 重采样后的点列表。
        """
        q_points = np.asarray(q_points, dtype=float)

        # 计算相邻点差值与长度（即弧长增量）
        diffs = np.diff(q_points, axis=0)
        ds = np.linalg.norm(diffs, axis=1)

        # 累计弧长 s[0]=0, s[-1]=总弧长
        s = np.hstack([[0], np.cumsum(ds)])
        s_total = s[-1]


        # 如果路径几乎没有长度（所有点都相同）
        if s_total < 1e-8:
            # 直接返回重复的同一个点
            return np.repeat(q_points[0:1], 2, axis=0)

        num_points = int(np.ceil(s_total / max(step, 1e-12))) + 1
        num_points = max(2, num_points)
        # 均匀取样 num_points 个弧长点
        s_uniform = np.linspace(0, s_total, num_points)

        # 构造弧长到关节角的插值函数（每个维度自动插值）
        kind = 'cubic' if q_points.shape[0] >= 4 else 'linear'
        interp_fun = interp1d(s, q_points, axis=0, kind=kind)

        # 根据均匀弧长取样，生成重采样轨迹
        q_eq = interp_fun(s_uniform)

        return q_eq.tolist()
    # ------------------------------------------------------------

    def affine_align_to_fixed_ends(
        self,
        q_eq: List[List[float]],
        q_start_req: np.ndarray,
        q_end_req: np.ndarray,
    ) -> List[List[float]]:
        """对弧长重采样后的轨迹做仿射变换（线性缩放+平移），使轨迹的首尾点精确对齐到指定的起点和终点。
        
        数学形式：
        q_aligned = q_start_req + (q_eq - q0) * scale
        其中 scale = (q_end_req - q_start_req) / (q1 - q0)
        
        Args:
            q_eq (List[List[float]]): 重采样后的轨迹。
            q_start_req (np.ndarray): 要求的起点。
            q_end_req (np.ndarray): 要求的终点。
        
        Returns:
            List[List[float]]: 对齐后的轨迹。
        """
        q_eq = np.asarray(q_eq, dtype=float)
        q_start_req = np.asarray(q_start_req, dtype=float)
        q_end_req = np.asarray(q_end_req, dtype=float)

        # 获取原轨迹的首尾点
        q0, q1 = q_eq[0], q_eq[-1]

        # 防止除0：若某个关节起终角度相同，则强制除数不为0
        denom = (q1 - q0)
        denom[np.abs(denom) < 1e-9] = 1e-9

        # 每个关节单独计算缩放比例
        scale = (q_end_req - q_start_req) / denom

        # 仿射变换（线性缩放 + 平移）
        q_aligned = q_start_req + (q_eq - q0) * scale

        # 确保边界精确匹配
        q_aligned[0] = q_start_req
        q_aligned[-1] = q_end_req

        return q_aligned.tolist()
    # ------------------------------------------------------------
