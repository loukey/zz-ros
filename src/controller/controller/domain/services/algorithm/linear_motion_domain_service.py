from math import pi
import numpy as np
from .kinematic_domain_service import KinematicDomainService
from ...utils import KinematicUtils


class LinearMotionDomainService:
    """直线运动规划服务。

    提供空间直线插值运动规划功能，支持指定起点终点或指定方向距离的规划。

    Attributes:
        v_max (np.ndarray): 各关节最大速度。
        a_max (np.ndarray): 各关节最大加速度。
        j_max (np.ndarray): 各关节最大加加速度。
        dt (float): 时间步长。
        kinematic_solver (KinematicDomainService): 运动学求解器。
    """

    def __init__(self, kinematic_solver: KinematicDomainService, v_max=None, a_max=None, j_max=None, dt=0.01):
        """初始化直线运动服务。

        Args:
            kinematic_solver (KinematicDomainService): 运动学求解器（通过 DI 注入）。
            v_max (list[float], optional): 最大速度列表. Defaults to [pi/4]*6.
            a_max (list[float], optional): 最大加速度列表. Defaults to [pi/8]*6.
            j_max (list[float], optional): 最大加加速度列表. Defaults to [pi/16]*6.
            dt (float, optional): 时间步长. Defaults to 0.01.
        """
        self.v_max = np.asarray(v_max if v_max is not None else [pi/4] * 6, dtype=float)
        self.a_max = np.asarray(a_max if a_max is not None else [pi/8] * 6, dtype=float)
        self.j_max = np.asarray(j_max if j_max is not None else [pi/16] * 6, dtype=float)
        self.dt = float(dt)
        self.kinematic_solver = kinematic_solver
        self.nearest_position = []
        self.initial_pos = []

    def linear_motion(self, start_position: list[float], end_position: list[float]) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
        """规划两点间的直线运动。

        Args:
            start_position (list[float]): 起点关节角度。
            end_position (list[float]): 终点关节角度。

        Returns:
            tuple: (t_list, positions, qd, qdd) 轨迹数据。
        """
        self.nearest_position = start_position
        self.initial_pos = self.nearest_position.copy()
        start_quat, start_pos = self.kinematic_solver.get_gripper2base(start_position)
        end_quat, end_pos = self.kinematic_solver.get_gripper2base(end_position)
        quat_list, pos_list, n_seg = self.sampling(start_quat, start_pos, end_quat, end_pos)
        t_list, positions, qd, qdd = self.smooth(quat_list, pos_list, n_seg)
        return t_list, positions, qd, qdd

    def linear_motion_z_axis(self, start_position: list[float], distance: float, direction: list[float], ds: float = 0.02, include_end: bool = True) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
        """沿指定方向矢量进行直线运动规划。

        注意：此模式下末端姿态保持不变，仅位置发生位移。

        Args:
            start_position (list[float]): 起点关节角度。
            distance (float): 移动距离 (m)。
            direction (list[float]): 方向矢量 [x, y, z]。
            ds (float, optional): 空间采样步长. Defaults to 0.002.
            include_end (bool, optional): 是否包含终点. Defaults to True.

        Returns:
            tuple: (t_list, positions, qd, qdd) 轨迹数据。

        Raises:
            ValueError: 如果方向向量模长过小。
        """
        self.nearest_position = start_position
        self.initial_pos = self.nearest_position.copy()
        start_quat, start_pos = self.kinematic_solver.get_gripper2base(start_position)
        p0 = np.asarray(start_pos, dtype=float).reshape(3)
        q0 = KinematicUtils.q_normalize(start_quat)

        direction = np.asarray(direction, dtype=float)
        norm = np.linalg.norm(direction)
        if norm < 1e-9:
            raise ValueError("Direction vector is zero or too small")
        direction = direction / norm

        # 需要的段数（保证每步不超过 ds）
        L = abs(float(distance))
        if L < 1e-12:
            t = np.array([0.0, 1.0]) if include_end else np.array([0.0])
        else:
            n_seg = max(1, int(np.ceil(L / ds)))
            t = np.linspace(0.0, 1.0, n_seg + 1 if include_end else n_seg)

        d = direction * float(distance)

        # 位置线性采样；姿态保持不变
        pos_list = p0[None, :] + t[:, None] * d[None, :]
        quat_list = np.repeat(q0[None, :], len(t), axis=0)

        t_list, positions, qd, qdd = self.smooth(quat_list, pos_list, n_seg)

        return t_list, positions, qd, qdd

    def sampling(self, start_quat: np.ndarray, start_pos: np.ndarray, end_quat: np.ndarray, end_pos: np.ndarray, sampling_dis: float = 0.02, include_end: bool = True) -> tuple[np.ndarray, np.ndarray, int]:
        """对起点和终点进行空间线性插值采样。

        Args:
            start_quat (np.ndarray): 起点四元数。
            start_pos (np.ndarray): 起点位置。
            end_quat (np.ndarray): 终点四元数。
            end_pos (np.ndarray): 终点位置。
            sampling_dis (float, optional): 采样间距. Defaults to 0.002.
            include_end (bool, optional): 是否包含终点. Defaults to True.

        Returns:
            tuple: (quat_list, pos_list, n_seg)
        """
        quat_list = []
        pos_list = []
        p0=start_pos
        p1=end_pos
        p0 = np.asarray(p0, dtype=float).reshape(3)
        p1 = np.asarray(p1, dtype=float).reshape(3)

        # 兼容两种四元数顺序：若最后一项绝对值最大，视为 [x,y,z,w]，转成 [w,x,y,z]
        q0=start_quat
        q1=end_quat
        q0 = np.asarray(q0, dtype=float).reshape(4)
        q1 = np.asarray(q1, dtype=float).reshape(4)

        # 计算需要的采样点数
        L = np.linalg.norm(p1 - p0)
        if L < 1e-9:
            t = np.array([0.0, 1.0]) if include_end else np.array([0.0])
        else:
            n_seg = max(1, int(np.ceil(L / float(sampling_dis))))
            t = np.linspace(0.0, 1.0, n_seg + 1 if include_end else n_seg)

        # 位置线性插值
        pos_list = (p0[None, :] + (p1 - p0)[None, :] * t[:, None])

        # 姿态 SLERP
        quat_list = np.vstack([KinematicUtils.q_slerp(q0, q1, ti) for ti in t])


        return quat_list, pos_list, n_seg

    def smooth(self, quat_list: np.ndarray, pos_list: np.ndarray, n_seg: int) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
        """对采样点序列进行逆运动学求解和时间参数化平滑。

        Args:
            quat_list (np.ndarray): 四元数序列。
            pos_list (np.ndarray): 位置序列。
            n_seg (int): 分段数。

        Returns:
            tuple: (t_list, positions, qd, qdd)
        """
        positions = []
        for quat, pos in zip(quat_list, pos_list):
            position = self.inverse_kinematic(quat, pos)
            if position is None:
                continue
            positions.append(position)
        positions = np.array(positions)
        q_wp = KinematicUtils.ensure_waypoints_2d(positions)
        grid_n = KinematicUtils.clamp(6*n_seg,300,3000)
        t_list, positions, qd, qdd = KinematicUtils.toppra_time_parameterize(q_wp, self.v_max, self.a_max, self.dt, grid_n)
        return t_list, positions, qd, qdd

    def inverse_kinematic(self, quat, pos):
        """单点逆运动学求解（利用上一位置作为初值）。"""
        rm = KinematicUtils.quat2rm(quat)
        try:
            inverse_position = self.kinematic_solver.inverse_kinematic(rm, pos, initial_theta=self.nearest_position)
        except ValueError:
            return None
        self.nearest_position = inverse_position
        return inverse_position
