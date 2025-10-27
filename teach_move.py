import numpy as np
from typing import List, Tuple
from math import pi
from scipy.interpolate import interp1d
import toppra as ta
import toppra.constraint as ta_constraint
import toppra.algorithm as ta_algo
import json
import matplotlib.pyplot as plt

class TeachPathProcessor:
    """
    示例教路径处理器：
    - 去除停顿导致的重复点
    - 在关节空间上按弧长重采样（保证空间均匀）
    - 通过仿射线性变换（缩放+平移）匹配指定的起点和终点
    """

    # ------------------------------------------------------------
   
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

        return q_eq.tolist(), num_points
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
        q_eq, num_points = self.resample_equal_arclen(q_clean, step=step)

        # ③ 仿射线性变换（对齐新的起点和终点）
        q_aligned = self.affine_align_to_fixed_ends(q_eq, q_start_req, q_end_req)

        # 返回最终平滑、对齐后的关节轨迹
        return q_aligned, num_points
    
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
            # duration = jnt_traj.duration
            # ts_sample = np.arange(0, duration, dt)
            # qs_sample = jnt_traj(ts_sample)
            
            # return qs_sample.tolist()
            T = float(jnt_traj.duration)
            M = max(2, int(np.ceil(T / dt)) + 1)
            t = np.linspace(0.0, T, M)

            q = jnt_traj.eval(t)
            qd = jnt_traj.evald(t)
            qdd = jnt_traj.evaldd(t)
            return t.tolist(), q.tolist(), qd.tolist(), qdd.tolist()

        except Exception as e:
            return angles_list

if __name__ == "__main__":
    # 示例：原始示教关节角（弧度）
    
    with open("teach_record.json", "r", encoding="utf-8") as f:
         data = json.load(f) 
         q_teach = data["record2"]

    # 目标：关节空间相邻点的距离约 0.01 rad
    q_start_req = np.array([1.3918838331828283, -0.8470450163106025, -1.3399202339932021, -0.49325871287957135, -1.2632571472736362, 1.5706165634213163 ])
    q_end_req = np.array([1.5384868564500553, -1.5853212073801888, -1.7178187979338109, -0.1775343077479523, -1.4224436066914876, 2.9365785181338584 ])
    proc = TeachPathProcessor()  # ← 需要实例化
    q_eq, num_points = proc.teach_smooth(q_teach, q_start_req, q_end_req, step=0.2, eps=1e-4)
    print(num_points)
    t_list, positions, qd, qdd = proc.toppra_smooth(q_eq, [pi/4] * 6, [pi/8] * 6, 0.01)
    t = np.array(t_list)
    q = np.array(positions)
    qd = np.array(qd)
    qdd = np.array(qdd)

    # 画 q
    plt.figure(figsize=(10, 6))
    for i in range(q.shape[1]):
        plt.plot(t, q[:, i], label=f'Joint {i+1}')
    plt.title('Joint Position vs Time')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.legend()
    plt.grid(True)
    plt.show()

    # 画 qd
    plt.figure(figsize=(10, 6))
    for i in range(qd.shape[1]):
        plt.plot(t, qd[:, i], label=f'Joint {i+1}')
    plt.title('Joint Velocity vs Time')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [rad/s]')
    plt.legend()
    plt.grid(True)
    plt.show()

    # 画 qdd
    plt.figure(figsize=(10, 6))
    for i in range(qdd.shape[1]):
        plt.plot(t, qdd[:, i], label=f'Joint {i+1}')
    plt.title('Joint Acceleration vs Time')
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [rad/s²]')
    plt.legend()
    plt.grid(True)
    plt.show()