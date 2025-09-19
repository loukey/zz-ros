# -*- coding: utf-8 -*-
"""
将一串 IK 关节路标（N × dof）用 Ruckig 做“时间重定时”，
生成 C^2（含 jerk 限制）的轨迹，输出按 dt 采样的 位置 / 速度 / 加速度 / 时间序列。

适用：位置模式（Position Mode）
- 每个周期下发 q[k]（关节位置）；
- qd[k]、qdd[k] 可作为速度/加速度前馈或用于验证绘图。

说明：
- 若 Ruckig 版本支持 `intermediate_positions`，优先走“waypoints 连续通过”（中间点不停，终点停）。
- 若不支持，自动降级为“逐段到点停”（每个路标停一下），同样满足 v/a/jerk 约束但用时更长。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
import os
import matplotlib.pyplot as plt


# -----------------------------------------------------------------------------#
# 工具函数
# -----------------------------------------------------------------------------#

def wrap_to_pi(q: np.ndarray) -> np.ndarray:
    """将角度包裹到 (-pi, pi]，避免 2π 跳变影响计算。"""
    return (q + np.pi) % (2 * np.pi) - np.pi


def ensure_2d_array(arr: np.ndarray) -> np.ndarray:
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


# -----------------------------------------------------------------------------#
# 数据结构
# -----------------------------------------------------------------------------#

@dataclass
class JointLimits:
    """每个关节的速度 / 加速度 / 加加速度（jerk）上限。"""
    v_max: Sequence[float]
    a_max: Sequence[float]
    j_max: Sequence[float]

    def as_lists(self) -> Tuple[List[float], List[float], List[float]]:
        return list(map(float, self.v_max)), list(map(float, self.a_max)), list(map(float, self.j_max))


# -----------------------------------------------------------------------------#
# 核心：Ruckig 时间重定时（waypoints 连续通过）
# -----------------------------------------------------------------------------#

def retime_with_ruckig_waypoints(
    q_waypoints: np.ndarray,
    limits: JointLimits,
    dt: float = 0.002,
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[float]]:
    """
    使用 Ruckig 的“中间路标连续通过”能力进行时间重定时（中间点不停，终点停）。

    Parameters
    ----------
    q_waypoints : np.ndarray
        路标序列，形状 (N, dof)，单位：弧度。
    limits : JointLimits
        关节约束（v/a/jerk 上限）。
    dt : float
        采样周期（秒），例如 0.002 → 500 Hz。

    Returns
    -------
    q_list, qd_list, qdd_list, t_list
        依次为：位置、速度、加速度、时间戳（均为等间隔 dt）。
    """
    try:
        from ruckig import Ruckig, InputParameter, OutputParameter, Result  # pip install ruckig
    except Exception as exc:
        raise RuntimeError("需要安装 python-ruckig（pip install ruckig）。") from exc

    q_wp = wrap_to_pi(ensure_2d_array(q_waypoints))
    dof = int(q_wp.shape[1])
    num_wp = max(0, q_wp.shape[0] - 2)
    otg = Ruckig(dof, dt,num_wp)
    inp = InputParameter(dof)
    out = OutputParameter(dof)

    # 当前状态（通常从静止开始；若不是静止请填真实值）
    inp.current_position = q_wp[0].tolist()
    inp.current_velocity = [0.0] * dof
    inp.current_acceleration = [0.0] * dof

    # 中间路标（不停顿通过）+ 终点（停住）
    if q_wp.shape[0] > 2:
        # 低版本 Ruckig 可能没有该属性；外部会捕获 AttributeError 并降级。
        inp.intermediate_positions = [q.tolist() for q in q_wp[1:-1]]

    inp.target_position = q_wp[-1].tolist()
    inp.target_velocity = [0.0] * dof
    inp.target_acceleration = [0.0] * dof

    inp.max_velocity, inp.max_acceleration, inp.max_jerk = limits.as_lists()

    q_list: List[np.ndarray] = []
    qd_list: List[np.ndarray] = []
    qdd_list: List[np.ndarray] = []
    t_list: List[float] = []

    t = 0.0
    while True:
        res = otg.update(inp, out)

        q_list.append(np.array(out.new_position))
        qd_list.append(np.array(out.new_velocity))
        qdd_list.append(np.array(out.new_acceleration))
        t_list.append(t)
        t += dt

        if res == Result.Working:
            out.pass_to_input(inp)
        elif res == Result.Finished:
            break
        else:
            raise RuntimeError(f"Ruckig 失败：{res}")

    return q_list, qd_list, qdd_list, t_list





# -----------------------------------------------------------------------------#
# 顶层封装：自动选择最优/保底路径
# -----------------------------------------------------------------------------#

def ruckig_time_retiming(
    q_waypoints: np.ndarray,
    limits: JointLimits | Dict[str, Iterable[float]],
    dt: float = 0.002,
    prefer_pass_through: bool = False,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    仅使用 waypoints 连续通过方案；去掉逐段到点停与回退逻辑。
    """
    if isinstance(limits, dict):
        limits = JointLimits(limits["v_max"], limits["a_max"], limits["j_max"])

    q_wp = ensure_2d_array(q_waypoints)

    q, qd, qdd, t_list = retime_with_ruckig_waypoints(q_wp, limits, dt)

    return np.asarray(q), np.asarray(qd), np.asarray(qdd), np.asarray(t_list)


# ====== 追加到你的脚本底部（或单独保存为 plot_utils.py 引用）======



def save_csv(q: np.ndarray, qd: np.ndarray, qdd: np.ndarray, t: np.ndarray, outdir: str = "export"):
    """把 q/qd/qdd/t 导出为 CSV，便于做离线对比或导入上位机。"""
    os.makedirs(outdir, exist_ok=True)
    np.savetxt(os.path.join(outdir, "Q.csv"),   q,   delimiter=",")
    np.savetxt(os.path.join(outdir, "dQ.csv"),  qd,  delimiter=",")
    np.savetxt(os.path.join(outdir, "ddQ.csv"), qdd, delimiter=",")
    np.savetxt(os.path.join(outdir, "t.csv"),   t,   delimiter=",")
    print(f"[CSV] 已导出到 {os.path.abspath(outdir)}")


def plot_all_joints(
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    t: np.ndarray,
    outdir: str = "export",
    joint_names=None,
    show: bool = False,
):
    """
    为每个关节画 3 条曲线（位置/速度/加速度），分别保存为 PNG。
    同时额外输出“总览图”（所有关节叠在一起，便于快速巡检峰值与限幅）。
    """
    os.makedirs(outdir, exist_ok=True)

    q = np.asarray(q); qd = np.asarray(qd); qdd = np.asarray(qdd); t = np.asarray(t)
    assert q.shape == qd.shape == qdd.shape, "q/qd/qdd 形状不一致"
    assert q.shape[0] == t.shape[0], "时间长度与轨迹点数不一致"

    n, dof = q.shape
    if joint_names is None:
        joint_names = [f"J{i+1}" for i in range(dof)]

    # ---- 每个关节单独三联图 ----
    for j in range(dof):
        fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        axes[0].plot(t, q[:, j]);   axes[0].set_ylabel("pos (rad)"); axes[0].grid(True)
        axes[1].plot(t, qd[:, j]);  axes[1].set_ylabel("vel (rad/s)"); axes[1].grid(True)
        axes[2].plot(t, qdd[:, j]); axes[2].set_ylabel("acc (rad/s²)"); axes[2].set_xlabel("time (s)"); axes[2].grid(True)
        fig.suptitle(f"{joint_names[j]} trajectory")
        fig.tight_layout(rect=[0, 0.03, 1, 0.97])

        fpath = os.path.join(outdir, f"{joint_names[j]}_q_qd_qdd.png")
        fig.savefig(fpath, dpi=150)
        if show:
            plt.show()
        plt.close(fig)

    # ---- 总览图（各关节叠加）----
    def _overview(data, ylabel, fname):
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for j in range(dof):
            ax.plot(t, data[:, j], label=joint_names[j])
        ax.set_xlabel("time (s)"); ax.set_ylabel(ylabel); ax.grid(True); ax.legend(ncol=min(dof, 6), fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, fname), dpi=150)
        if show:
            plt.show()
        plt.close(fig)

    _overview(q,   "pos (rad)",     "overview_q.png")
    _overview(qd,  "vel (rad/s)",   "overview_qd.png")
    _overview(qdd, "acc (rad/s²)",  "overview_qdd.png")

    print(f"[PNG] 曲线图已保存至：{os.path.abspath(outdir)}")

# -----------------------------------------------------------------------------#
# 示例（可直接运行）
# -----------------------------------------------------------------------------#

if __name__ == "__main__":
    # 示例：6 轴 IK 路标（单位：度 → 弧度）
    q_waypoints_demo = np.deg2rad(np.array([
        [0,  -20,  40,   0,  45,  0],   # q0
        [10, -22,  42,   1,  45,  2],   # q1（轻微抖动）
        [20, -25,  45,   2,  45,  4],   # q2
        [35, -15,  30,  -5,  30,  5],   # q3
        [40, -10,  20, -10,  20,  8],   # q4（终点）
    ]), dtype=float)

    # 关节约束（按你的设备填写实际值；通常取额定 60~80% 更稳）
    limits_demo = JointLimits(
        v_max=[2.5, 2.5, 2.5, 3.0, 3.0, 3.5],          # rad/s
        a_max=[10.0, 10.0, 10.0, 15.0, 15.0, 20.0],    # rad/s^2
        j_max=[800.0, 800.0, 800.0, 1200.0, 1200.0, 1500.0],  # rad/s^3
    )

    # 采样周期
    dt_demo = 0.01  # 100 Hz


    q, qd, qdd, t = ruckig_time_retiming(
        q_waypoints_demo,
        limits_demo,
        dt=dt_demo,
        prefer_pass_through=False,   # 升级支持后改为 True
    )

    print(f"采样点数: {len(t)}, 总时长: {t[-1]:.3f}s, dof={q.shape[1]}")
    print("首帧位置(rad): ", np.round(q[0], 6))
    print("末帧位置(rad): ", np.round(q[-1], 6))
    print("首帧速度(rad/s):", np.round(qd[0], 6))
    print("末帧速度(rad/s):", np.round(qd[-1], 6))

    # 如需导出 CSV 便于上位机/可视化：
    np.savetxt("Q.csv",   q,   delimiter=",")
    np.savetxt("dQ.csv",  qd,  delimiter=",")
    np.savetxt("ddQ.csv", qdd, delimiter=",")
    np.savetxt("t.csv",   t,   delimiter=",")

    save_csv(q, qd, qdd, t, outdir="export")

    # 画图并保存 PNG
    plot_all_joints(q, qd, qdd, t, outdir="export", joint_names=None, show=False)