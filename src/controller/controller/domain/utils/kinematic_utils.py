from math import cos, sin, pi
import numpy as np
from scipy.spatial.transform import Rotation as R
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

class KinematicUtils:
    """运动学工具类。
    
    提供基本的运动学变换和角度处理函数。
    """

    @staticmethod
    def dh2rm(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """根据 DH 参数计算变换矩阵。
        
        Args:
            a (float): 连杆长度。
            alpha (float): 连杆扭转角。
            d (float): 连杆偏移。
            theta (float): 关节角。
            
        Returns:
            np.ndarray: 4x4 齐次变换矩阵。
        """
        c_theta = cos(theta)
        s_theta = sin(theta)
        c_alpha = cos(alpha)
        s_alpha = sin(alpha)
    
        return np.array([
            [c_theta, -s_theta, 0, a],
            [s_theta * c_alpha, c_theta * c_alpha, -s_alpha, -s_alpha * d],
            [s_theta * s_alpha, c_theta * s_alpha, c_alpha, c_alpha * d],
            [0, 0, 0, 1]
        ], dtype=np.float64)

    @staticmethod
    def rm2quat(rm: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数。
        
        Args:
            rm (np.ndarray): 3x3 旋转矩阵或 4x4 变换矩阵。
            
        Returns:
            np.ndarray: 四元数 [x, y, z, w]。
        """
        # 如果是 4x4 矩阵，提取 3x3 部分
        if rm.shape == (4, 4):
            rm = rm[:3, :3]
        return R.from_matrix(rm).as_quat()

    @staticmethod
    def quat2rm(quat: np.ndarray) -> np.ndarray:
        """四元数转旋转矩阵。
        
        Args:
            quat (np.ndarray): 四元数 [x, y, z, w]。
            
        Returns:
            np.ndarray: 3x3 旋转矩阵。
        """
        return R.from_quat(quat).as_matrix()

    @staticmethod
    def quat2euler(quat: np.ndarray) -> np.ndarray:
        """四元数转欧拉角 (XYZ 顺序)。
        
        Args:
            quat (np.ndarray): 四元数 [x, y, z, w]。
            
        Returns:
            np.ndarray: 欧拉角 [roll, pitch, yaw] (弧度)。
        """
        return R.from_quat(quat).as_euler('xyz', degrees=False)

    @staticmethod
    def euler2quat(euler: np.ndarray) -> np.ndarray:
        """欧拉角转四元数 (XYZ 顺序)。
        
        Args:
            euler (np.ndarray): 欧拉角 [roll, pitch, yaw] (弧度)。
            
        Returns:
            np.ndarray: 四元数 [x, y, z, w]。
        """
        return R.from_euler('xyz', euler, degrees=False).as_quat()

    @staticmethod
    def euler2rm(euler: np.ndarray) -> np.ndarray:
        """欧拉角转旋转矩阵 (XYZ 顺序)。
        
        Args:
            euler (np.ndarray): 欧拉角 [roll, pitch, yaw] (弧度)。
            
        Returns:
            np.ndarray: 3x3 旋转矩阵。
        """
        return R.from_euler('xyz', euler, degrees=False).as_matrix()

    @staticmethod
    def rm2euler(rm: np.ndarray) -> np.ndarray:
        """旋转矩阵转欧拉角 (XYZ 顺序)。
        
        Args:
            rm (np.ndarray): 3x3 旋转矩阵。
            
        Returns:
            np.ndarray: 欧拉角 [roll, pitch, yaw] (弧度)。
        """
        # 如果是 4x4 矩阵，提取 3x3 部分
        if rm.shape == (4, 4):
            rm = rm[:3, :3]
        return R.from_matrix(rm).as_euler('xyz', degrees=False)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """将角度归一化到 [-pi, pi] 范围。

        Args:
            angle (float): 输入角度（弧度）。

        Returns:
            float: 归一化后的角度（弧度）。
        """
        return (angle + pi) % (2 * pi) - pi

    @staticmethod
    def q_normalize(q: np.ndarray) -> np.ndarray:
        """四元数归一化。"""
        q = np.asarray(q, dtype=float)
        n = np.linalg.norm(q)
        if n == 0:
            raise ValueError("zero quaternion")
        return q / n

    @staticmethod
    def q_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        """四元数最短弧 SLERP，t in [0,1]。"""
        q0 = KinematicUtils.q_normalize(q0)
        q1 = KinematicUtils.q_normalize(q1)
        dot = np.dot(q0, q1)

        if dot < 0.0:
            q1 = -q1
            dot = -dot

        if dot > 0.9995:
            q = q0 + t * (q1 - q0)
            return KinematicUtils.q_normalize(q)

        theta0 = np.arccos(np.clip(dot, -1.0, 1.0))
        sin_theta0 = np.sin(theta0)
        s0 = np.sin((1.0 - t) * theta0) / sin_theta0
        s1 = np.sin(t * theta0) / sin_theta0
        return s0 * q0 + s1 * q1

    @staticmethod
    def wrap_to_pi(q: np.ndarray) -> np.ndarray:
        """将角度包裹到 (-pi, pi]。"""
        return (q + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def ensure_waypoints_2d(arr: np.ndarray) -> np.ndarray:
        """将输入转换为二维数组 (N, dof) 并做基本校验。

        Raises:
            ValueError: 当路标数少于 2 时抛出。
        """
        q = np.asarray(arr, dtype=float)
        if q.ndim == 1:
            q = q[None, :]
        if not (q.ndim == 2 and q.shape[0] >= 2):
            raise ValueError("Q_waypoints 至少需要两个点（起点与终点）且维度为 (N, dof)。")
        return q

    @staticmethod
    def clamp(x, low, high):
        """限制数值范围。"""
        return max(low, min(x, high))

    @staticmethod
    def toppra_time_parameterize(
        waypoints: np.ndarray,
        v_max: 'np.ndarray | float',
        a_max: 'np.ndarray | float',
        dt: float = 0.01,
        grid_n: int = 800
    ) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
        """使用 TOPPRA 进行时间参数化。

        Args:
            waypoints (np.ndarray): 路径点 (N, dof)。
            v_max: 最大速度。
            a_max: 最大加速度。
            dt (float): 时间步长。
            grid_n (int): 网格点数。

        Returns:
            tuple: (t, q, qd, qdd).
        """
        waypoints = np.asarray(waypoints, dtype=float)
        if waypoints.ndim != 2 or waypoints.shape[1] != 6 or waypoints.shape[0] < 2:
            raise ValueError("waypoints 必须是 (N,6) 且 N>=2")

        dof = waypoints.shape[1]

        breaks = np.linspace(0.0, 1.0, waypoints.shape[0])
        path = ta.SplineInterpolator(breaks, waypoints)

        v_max = np.full(dof, float(v_max)) if np.isscalar(v_max) else np.asarray(v_max, dtype=float)
        a_max = np.full(dof, float(a_max)) if np.isscalar(a_max) else np.asarray(a_max, dtype=float)
        if v_max.shape != (dof,) or a_max.shape != (dof,):
            raise ValueError("v_max/a_max 需为标量或 shape=(6,)")

        v_bounds = np.column_stack((-np.abs(v_max), np.abs(v_max)))
        a_bounds = np.column_stack((-np.abs(a_max), np.abs(a_max)))

        pc_vel = constraint.JointVelocityConstraint(v_bounds)
        pc_acc = constraint.JointAccelerationConstraint(a_bounds)

        gridpoints = np.linspace(0, path.duration, int(grid_n))
        instance = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints, parametrizer="ParametrizeConstAccel")

        jnt_traj = instance.compute_trajectory(sd_start=0.0, sd_end=0.0)
        if jnt_traj is None:
            raise RuntimeError("TOPPRA 求解失败：给定约束下不可行，或路径异常。")

        T = float(jnt_traj.duration)
        M = max(2, int(np.ceil(T / dt)) + 1)
        t = np.linspace(0.0, T, M)

        q = jnt_traj.eval(t)
        qd = jnt_traj.evald(t)
        qdd = jnt_traj.evaldd(t)
        return t.tolist(), q.tolist(), qd.tolist(), qdd.tolist()
