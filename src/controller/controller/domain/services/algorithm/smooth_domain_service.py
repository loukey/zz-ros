"""
轨迹平滑领域服务 - Domain/Algorithm层
负责所有轨迹的平滑处理算法
"""
import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
from scipy.interpolate import interp1d
from typing import List, Tuple
from math import pi
import toppra as ta
import toppra.constraint as ta_constraint
import toppra.algorithm as ta_algo


class SmoothDomainService:
    """
    轨迹平滑服务
    
    提供多种平滑算法：
    1. Cubic Spline + Savitzky-Golay 滤波（默认，旧版）
    2. TOPPRA 时间参数化平滑（可选，旧版测试方法）
    
    适用场景：
    - 示教轨迹播放
    - 多点轨迹规划
    - 任何需要轨迹平滑的场景
    """
    
    def __init__(self):
        pass
    
    def smooth_trajectory(
        self, 
        angles_list: List[List[float]], 
        method: str = "spline_savgol",
        **kwargs
    ) -> List[List[float]]:
        """
        平滑轨迹（统一接口）
        
        Args:
            angles_list: 原始轨迹 [[θ1,...,θ6], ...]
            method: 平滑方法 ("spline_savgol" 或 "toppra")
            **kwargs: 各算法的特定参数
        
        Returns:
            平滑后的轨迹点列表
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
        """
        Cubic Spline + Savitzky-Golay 滤波（参考旧版）
        
        流程：
        1. Savitzky-Golay 平滑原始数据
        2. Cubic Spline 上采样插值
        3. 重采样回原始点数
        4. 再次 Savitzky-Golay 平滑
        
        Args:
            angles_list: 原始轨迹 N×6
            upsample: 上采样倍数
            sg_window: Savgol 滤波窗口（必须是奇数）
            sg_poly: Savgol 多项式阶数
        
        Returns:
            平滑后的轨迹列表
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
    
    def toppra_smooth(
        self,
        angles_list: List[List[float]],
        v_max: List[float] = [pi/4] * 6,
        a_max: List[float] = [pi/8] * 6,
        dt: float = 0.01
    ) -> List[List[float]]:
        """
        TOPPRA 时间参数化平滑（参考旧版 RuckigSmooth）
        
        使用时间最优路径参数化算法
        
        Args:
            angles_list: 原始轨迹 N×6
            v_max: 最大速度限制（关节空间）
            a_max: 最大加速度限制（关节空间）
            dt: 采样时间间隔
        
        Returns:
            平滑后的轨迹列表
        """
        
        arr = np.asarray(angles_list, dtype=float)
        N = len(arr)
        
        if N < 2:
            return angles_list
        
        try:
            # 1. 构造路径（使用归一化参数 s）
            path = ta.SplineInterpolator(
                np.linspace(0, 1, N),  # 路径参数 s
                arr                    # 关节角度
            )
            
            # 2. 定义约束
            pc_vel = ta_constraint.JointVelocityConstraint(v_max)
            pc_acc = ta_constraint.JointAccelerationConstraint(a_max)
            
            # 3. 计算时间参数化
            instance = ta_algo.TOPPRA(
                [pc_vel, pc_acc],
                path,
                parametrizer="ParametrizeConstAccel"
            )
            
            jnt_traj = instance.compute_trajectory()
            
            if jnt_traj is None:
                return angles_list
            
            # 4. 采样轨迹
            duration = jnt_traj.duration
            ts_sample = np.arange(0, duration, dt)
            qs_sample = jnt_traj(ts_sample)
            
            return qs_sample.tolist()
        
        except Exception as e:
            return angles_list

    def remove_redundant_points(self, q_teach: List[List[float]], eps: float = 1e-4) -> List[List[float]]:
            """
            改进版去除重复点：
            保留“任意一个关节角变化超过阈值”的点。
            """
            q_teach = np.asarray(q_teach, dtype=float)
            dq = np.abs(np.diff(q_teach, axis=0))  # 相邻关节角绝对差
            moving_mask = np.any(dq > eps, axis=1)  # 只要任意一轴动了就认为在动
            mask = np.hstack([[True], moving_mask])  # 保留第一个点
            return q_teach[mask].tolist()
        # ------------------------------------------------------------
    def resample_equal_arclen(self, q_points: List[List[float]], step: float = 0.002) -> List[List[float]]:
        """
        按弧长重采样，使路径在关节空间中分布均匀。
        原理：
        - 计算相邻点的欧氏距离（弧长）
        - 将弧长积分得到累计s
        - 对s均匀取样后，用插值函数生成新点
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
        """
        对弧长重采样后的轨迹做仿射变换（线性缩放+平移），
        使轨迹的首尾点精确对齐到指定的起点和终点。
        数学形式：
        q_aligned = q_start_req + (q_eq - q0) * scale
        其中 scale = (q_end_req - q_start_req) / (q1 - q0)
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

    def teach_smooth(
        self,
        q_teach: List[List[float]],
        q_start_req: np.ndarray,
        q_end_req: np.ndarray,
        step: float = 0.002,
        eps: float = 1e-6,
    ) -> List[List[float]]:
        """
        整体处理流程：
        1. 去除停顿/重复点
        2. 按弧长重采样（空间均匀）
        3. 仿射缩放和平移对齐指定的起点/终点
        """
        # ① 清洗原始示教数据（去掉重复点）
        q_clean = self.remove_redundant_points(q_teach, eps=eps)

        # ② 弧长重采样（得到等弧长轨迹）
        q_eq = self.resample_equal_arclen(q_clean, step=step)

        # ③ 仿射线性变换（对齐新的起点和终点）
        q_aligned = self.affine_align_to_fixed_ends(q_eq, q_start_req, q_end_req)

        # 返回最终平滑、对齐后的关节轨迹
        return q_aligned
