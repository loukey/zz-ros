"""
轨迹平滑领域服务 - Domain/Algorithm层
负责所有轨迹的平滑处理算法
"""
import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
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
        v_max: List[float] = None,
        a_max: List[float] = None,
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
        if v_max is None:
            v_max = [pi/4] * 6
        if a_max is None:
            a_max = [pi/8] * 6
        
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

