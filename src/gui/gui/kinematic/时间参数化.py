# toppra_6dof_timeparam.py
# -*- coding: utf-8 -*-
"""
输入：
    waypoints : (N, 6) 的关节角度路标（单位：rad），N>=2
可调：
    v_max, a_max : 每关节速度/加速度上限（rad/s, rad/s^2），可标量或(6,)
    dt           : 采样周期（s）
    grid_n       : TOPPRA 在路径参数 s 上的离散点数
输出：
    t   : (M,)     采样时间戳（s）
    q   : (M, 6)   关节位置（rad）
    qd  : (M, 6)   关节速度（rad/s）
    qdd : (M, 6)   关节加速度（rad/s^2）
"""

from __future__ import annotations
import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo


def toppra_time_parameterize(
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

    v_bounds = np.vstack((-np.abs(v_max), np.abs(v_max)))  # (2,6) -> [v_min; v_max]
    a_bounds = np.vstack((-np.abs(a_max), np.abs(a_max)))  # (2,6) -> [a_min; a_max]

    pc_vel = constraint.JointVelocityConstraint(v_bounds)
    pc_acc = constraint.JointAccelerationConstraint(a_bounds)

    # 3) TOPPRA 主算法 + 常用参数化器（常用且满足边界/约束）
    gridpoints = np.linspace(0, path.duration, int(grid_n))
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")

    # 4) 求解时间参数化，得到可按时间采样的 jnt_traj
    jnt_traj = instance.compute_trajectory(gridpoints)
    if jnt_traj is None:
        raise RuntimeError("TOPPRA 求解失败：给定约束下不可行，或路径异常。")

    # 5) 按 dt 采样
    T = float(jnt_traj.duration)
    M = max(2, int(np.ceil(T / dt)) + 1)
    t = np.linspace(0.0, T, M)

    q   = jnt_traj.eval(t, 0)
    qd  = jnt_traj.eval(t, 1)
    qdd = jnt_traj.eval(t, 2)
    return t, q, qd, qdd


# ===== 示例（可删除） =====
if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)

    # 你的 (N,6) 路标：单位 rad，这里给个例子
    waypoints = np.array([
        [0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
        [0.3, -0.2,  0.5, -0.3,  0.1,  0.2],
        [0.8,  0.1,  1.0, -0.6, -0.2,  0.4],
        [0.2, -0.1,  0.2, -0.2,  0.0,  0.1],
    ], dtype=float)

    # 每关节上限（示例；请替换为你的机器人规格）
    v_max = np.deg2rad([120, 120, 120, 180, 180, 180])  # rad/s
    a_max = np.deg2rad([300, 300, 300, 500, 500, 500])  # rad/s^2

    t, q, qd, qdd = toppra_time_parameterize(waypoints, v_max, a_max, dt=0.01, grid_n=500)

    print(f"总时长: {t[-1]:.3f}s  采样点: {len(t)}")
    print("首末位置对比：")
    print("q[0]  =", q[0])
    print("q[-1] =", q[-1])

    # 保存为 CSV（供前端/控制器读取）
    # np.savetxt("traj_time_q.csv", np.column_stack([t, q]), delimiter=",",
    #            header="t,q1,q2,q3,q4,q5,q6", comments="")
