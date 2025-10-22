from functools import reduce
from math import atan2, acos, sin, cos, pi, sqrt
import numpy as np
from scipy.spatial.transform import Rotation as R
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.interpolate import interp1d
class KinematicUtils:

    @staticmethod
    def dh2rm(a, alpha, d, theta):
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
    def rm2quat(rm):
        return R.from_matrix(rm).as_quat()

    @staticmethod
    def quat2rm(quat):
        return R.from_quat(quat).as_matrix()

    @staticmethod
    def quat2euler(quat):
        return R.from_quat(quat).as_euler('xyz', degrees=False)

    @staticmethod
    def euler2quat(euler):
        return R.from_euler('xyz', euler, degrees=False).as_quat()

    @staticmethod
    def euler2rm(euler):
        return R.from_euler('xyz', euler, degrees=False).as_matrix()

    @staticmethod
    def rm2euler(rm):
        return R.from_matrix(rm).as_euler('xyz', degrees=False)

    @staticmethod
    def normalize_angle(angle):
        return (angle + pi) % (2 * pi) - pi


class KinematicDomainService:
    def __init__(self):
        self.kinematic_dh = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, -pi / 2, 0.0, -pi / 2],
            [0.425, pi, 0.0, 0.0],
            [0.401, pi, 0.0856, pi / 2],
            [0.0, pi / 2, 0.086, 0.0],
            [0.0, -pi / 2, 0.231, 0.0]
        ], dtype=float)
        self.kinematic_utils = KinematicUtils()
        self.gripper2base = self.get_gripper2base(self.kinematic_dh[:, 3])

    def get_kinematic_dh(self):
        return self.kinematic_dh

    def get_gripper2base(self, theta_list=None):
        if theta_list is not None:
            self.kinematic_dh[:, 3] = theta_list
            rm_list = []
            for dh in self.kinematic_dh:
                rm_list.append(self.kinematic_utils.dh2rm(*dh))
            self.gripper2base = reduce(lambda x, y: x @ y, rm_list, np.eye(4))
        quat = R.from_matrix(self.gripper2base[:3, :3]).as_quat()
        return quat, self.gripper2base[:3, 3]

    def normalize_angle(self, angle):
        return (angle + pi) % (2 * pi) - pi

    def inverse_kinematic(self, rm, pos, initial_theta=None):
        if not initial_theta:
            initial_theta = self.kinematic_dh[:, 3]
        valid_solutions = []
        # rotation matrix
        nx, ny, nz = rm[:, 0]
        ox, oy, oz = rm[:, 1]
        ax, ay, az = rm[:, 2]
        px, py, pz = pos

        # DH参数
        a3 = 0.425
        a4 = 0.401
        d4 = 0.0856
        d5 = 0.086
        d6 = 0.231

        # 添加奇异性判断
        if abs(ax ** 2 + ay ** 2 - d4 ** 2) < 1e-6:
            return valid_solutions

        # theta1计算
        m = ay * d6 - py
        n = ax * d6 - px
        try:
            delta = m ** 2 + n ** 2 - d4 ** 2

            if delta < 0:
                return valid_solutions

            theta1_0 = atan2(m, n) - atan2(-d4, sqrt(m ** 2 + n ** 2 - d4 ** 2))
            theta1_1 = atan2(m, n) - atan2(-d4, -sqrt(m ** 2 + n ** 2 - d4 ** 2))
            theta1_candidate = [theta1_0, theta1_1]

        except Exception as e:
            return valid_solutions

        # theta5
        for i, theta1 in enumerate(theta1_candidate):
            s1 = sin(theta1)
            c1 = cos(theta1)
            # 修改theta5的计算方式
            cos_theta5 = -s1 * ax + c1 * ay
            if abs(cos_theta5) > 1:
                cos_theta5 = 1 if cos_theta5 > 0 else -1

            theta5_val = acos(cos_theta5)
            theta5_candidate = [theta5_val, -theta5_val]

            for j, theta5 in enumerate(theta5_candidate):
                s5 = sin(theta5)
                c5 = cos(theta5)

                # theta6
                if abs(s5) < 1e-6:
                    # 腕部奇异性
                    theta6 = initial_theta[5]
                else:
                    m1 = -s1 * nx + c1 * ny
                    n1 = -s1 * ox + c1 * oy
                    theta6 = atan2(-n1 / s5, m1 / s5)

                # theta3
                s6 = sin(theta6)
                c6 = cos(theta6)
                m2 = d5 * (s6 * (nx * c1 + ny * s1) + c6 * (ox * c1 + oy * s1)) - d6 * (
                            ax * c1 + ay * s1) + px * c1 + py * s1
                n2 = d5 * (oz * c6 + nz * s6) + pz - az * d6

                # 计算到关节3的距离
                r = sqrt(m2 ** 2 + n2 ** 2)

                if r > (a3 + a4) * 1.2 or r < abs(a3 - a4) * 0.8:  # 进一步放宽工作空间限制
                    continue
                temp = (m2 ** 2 + n2 ** 2 - a3 ** 2 - a4 ** 2) / (2 * a3 * a4)
                if abs(temp) > 1:
                    temp = 1 if temp > 0 else -1
                theta3_candidate = [acos(temp), -acos(temp)]

                for k, theta3 in enumerate(theta3_candidate):
                    # theta2
                    s3 = sin(theta3)
                    c3 = cos(theta3)
                    # 修改theta2的计算方式
                    k1 = a3 + a4 * c3
                    k2 = a4 * s3
                    s2 = (k1 * n2 - k2 * m2) / (k1 ** 2 + k2 ** 2)
                    c2 = (k1 * m2 + k2 * n2) / (k1 ** 2 + k2 ** 2)
                    theta2 = -atan2(s2, c2)

                    # theta4
                    if abs(s5) < 1e-6:
                        # 在腕部奇异性时，使用更稳定的计算方式
                        if initial_theta is not None:
                            theta4_candidate = [initial_theta[3]]
                        else:
                            theta4_candidate = [initial_theta[3]]
                    else:
                        T01 = self.kinematic_utils.dh2rm(self.kinematic_dh[0, 0], self.kinematic_dh[0, 1],
                                                         self.kinematic_dh[0, 2], theta1)
                        T12 = self.kinematic_utils.dh2rm(self.kinematic_dh[1, 0], self.kinematic_dh[1, 1],
                                                         self.kinematic_dh[1, 2], theta2)
                        T23 = self.kinematic_utils.dh2rm(self.kinematic_dh[2, 0], self.kinematic_dh[2, 1],
                                                         self.kinematic_dh[2, 2], theta3)
                        R03 = (T01 @ T12 @ T23)[:3, :3]
                        R36 = R03.T @ rm
                        theta4 = atan2(R36[1, 2], R36[0, 2])
                        theta4_candidate = [-theta4, pi - theta4]
                    for theta4 in theta4_candidate:
                        candidate = [theta1, theta2, theta3, theta4, theta5, theta6]
                        if self.verify_solution(candidate, (px, py, pz)):
                            valid_solutions.append(candidate)

        if not valid_solutions:
            raise ValueError("No valid solutions found")
        valid_solutions = [
            [self.kinematic_utils.normalize_angle(a) for a in sol] for sol in valid_solutions
        ]
        final_solution = min(valid_solutions, key=lambda sol: np.linalg.norm(np.array(sol) - np.array(initial_theta)))
        return final_solution

    def verify_solution(self, theta_list, target_pos):
        self.get_gripper2base(theta_list)
        current_pos = self.gripper2base[:3, 3]
        error = np.linalg.norm(current_pos - target_pos)

        return error < 1e-5


class CurveMotionDomainService:
    def __init__(self, v_max=[pi / 4] * 6, a_max=[pi / 8] * 6, j_max=[pi / 16] * 6, dt=0.01):
        self.v_max = np.asarray(v_max, dtype=float)
        self.a_max = np.asarray(a_max, dtype=float)
        self.j_max = np.asarray(j_max, dtype=float)
        self.dt = float(dt)
        self.kinematic_solver = KinematicDomainService()
        self.nearest_position = None

    def clamp(self, x, low, high):
        return max(low, min(x, high))

    def curve_motion(
            self,
            pos_fun,  # callable u -> (3,)
            u0: float,
            u1: float,
            start_position,  # 起点关节角，用于确定起始姿态/IK 近邻
            end_position,  # 可选：给终点姿态（用于 slerp）
            ds: float = 0.002,
            include_end: bool = True,
            orientation_mode: str = "slerp",  # 'fixed' | 'slerp' | 'tangent'
            tool_axis: str = "z",
            up_hint: np.ndarray = np.array([0, 0, 1.0])
    ):
        # 起点姿态来自起点关节角的正解
        start_quat, start_pos_fk = self.kinematic_solver.get_gripper2base(start_position)
        # 1) 等弧长采样
        pos_list, u_eq, n_seg = self.sample_parametric_equal_arclen(pos_fun, u0, u1, ds, include_end)
        
        # 2) 姿态生成
        if orientation_mode == "fixed":
            quat_list = np.repeat(self._q_normalize(start_quat)[None, :], len(pos_list), axis=0)

        elif orientation_mode == "slerp":
            if end_position is None:
                raise ValueError("slerp 模式需要提供 end_position 以确定终止姿态。")
            end_quat, _ = self.kinematic_solver.get_gripper2base(end_position)
            q0 = self._q_normalize(start_quat)
            q1 = self._q_normalize(end_quat)
            tau = np.linspace(0, 1, len(pos_list))
            quat_list = np.vstack([self._q_slerp(q0, q1, ti) for ti in tau])
            
        elif orientation_mode == "tangent":
            quat_list = self._quat_follow_tangent(pos_list, tool_axis=tool_axis, up_hint=up_hint)
        else:
            raise ValueError("orientation_mode 必须是 {'fixed','slerp','tangent'}")

        # 3) 原有平滑/IK/TOPPRA 管线复用
        t_list, positions, qd, qdd = self.smooth(quat_list, pos_list, n_seg)
        return t_list, positions, qd, qdd

    def make_pos_fun_spline(self, points: np.ndarray, bc_type="natural"):
        """
        points: (N,3) 或 (N,2) 控制点，按轨迹顺序排列
        返回:  pos_fun(u)  支持 u 标量或一维数组，定义域 [0,1]
        """
        P = np.asarray(points, dtype=float)
        if P.ndim != 2 or P.shape[0] < 2 or P.shape[1] not in (2, 3):
            raise ValueError("points 形状必须是 (N,3) 或 (N,2)，且 N>=2")
        if P.shape[1] == 2:
            P = np.hstack([P, np.zeros((P.shape[0], 1))])  # 2D 自动补 z=0

        # --- 弦长参数化，避免参数拥挤 ---
        d = np.linalg.norm(np.diff(P, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        if s[-1] == 0:
            raise ValueError("所有点重合，无法构造曲线")
        t = s / s[-1]  # 归一化到 [0,1]

        # 分别构造 x,y,z 的样条
        cs_x = CubicSpline(t, P[:, 0], bc_type=bc_type)
        cs_y = CubicSpline(t, P[:, 1], bc_type=bc_type)
        cs_z = CubicSpline(t, P[:, 2], bc_type=bc_type)

        def _eval(u):
            u = np.asarray(u, dtype=float)
            # 可选：夹紧到 [0,1]，避免数值越界
            u = np.clip(u, 0.0, 1.0)
            X = np.stack([cs_x(u), cs_y(u), cs_z(u)], axis=-1)  # (..., 3)
            if X.ndim == 1:  # 标量 u -> (3,)
                return X
            return X  # 数组 u -> (M,3)

        return _eval, s


    def _q_slerp(self, q0, q1, t):
        """四元数最短弧 SLERP，t in [0,1]"""
        q0 = self._q_normalize(q0)
        q1 = self._q_normalize(q1)
        dot = np.dot(q0, q1)

        # 如果点积为负，翻转 q1 以走最短路径
        if dot < 0.0:
            q1 = -q1
            dot = -dot

        # 非常接近时退化为线性插值再归一化，避免数值问题
        if dot > 0.9995:
            q = q0 + t * (q1 - q0)
            return self._q_normalize(q)

        theta0 = np.arccos(np.clip(dot, -1.0, 1.0))  # 初始夹角
        sin_theta0 = np.sin(theta0)
        s0 = np.sin((1.0 - t) * theta0) / sin_theta0
        s1 = np.sin(t * theta0) / sin_theta0
        return s0 * q0 + s1 * q1

    def sample_parametric_equal_arclen(
            self,
            pos_fun,  # callable: u -> (3,) numpy array
            u0: float,
            u1: float,
            ds: float = 0.002,
            include_end: bool = True,
            dense: int = 4000  # 先密采样用于构建弧长映射
    ):
        """
        对参数曲线 r(u) 做等弧长采样，返回 pos_list (M,3)、u_eq (M,) 和总弧长 L
        """
        # 1) 先在参数上做均匀密采样，估算弧长
        u_dense = np.linspace(u0, u1, max(1000, int(abs(u1 - u0) * dense)))
        XYZ = np.vstack([np.asarray(pos_fun(ui), dtype=float).reshape(3) for ui in u_dense])
        seg = np.linalg.norm(np.diff(XYZ, axis=0), axis=1)
        s_cum = np.concatenate([[0.0], np.cumsum(seg)])
        L = float(s_cum[-1])
        if L < 1e-12:
            # 退化：几乎零长
            pos_list = XYZ[[0, -1], :] if include_end else XYZ[[0], :]
            u_eq = np.array([u_dense[0], u_dense[-1]]) if include_end else np.array([u_dense[0]])
            return pos_list, u_eq, 0.0

        # 2) 弧长 -> u 的反查，用插值近似
        s2u = interp1d(s_cum, u_dense, kind="linear", bounds_error=False, fill_value=(u_dense[0], u_dense[-1]))

        # 3) 目标等弧长序列
        n_seg = max(1, int(np.ceil(L / ds)))
        s_target = np.linspace(0.0, L, n_seg + 1 if include_end else n_seg)
        u_eq = s2u(s_target)

        # 4) 得到等弧长位置
        pos_list = np.vstack([np.asarray(pos_fun(ui), dtype=float).reshape(3) for ui in u_eq])
        return pos_list, u_eq, n_seg

    def _quat_follow_tangent(self, pos_list, tool_axis="z", up_hint=np.array([0, 0, 1.0])):
        pos = np.asarray(pos_list, dtype=float)
        v = np.gradient(pos, axis=0)  # 中心差分
        v_norm = np.linalg.norm(v, axis=1, keepdims=True) + 1e-12
        t = v / v_norm  # 切向

        up = np.tile(up_hint / (np.linalg.norm(up_hint) + 1e-12), (len(pos), 1))
        # up 与切向几乎共线时换备用 up
        collinear = (np.abs(np.sum(up * t, axis=1)) > 0.98)
        if np.any(collinear):
            alt = np.array([1.0, 0, 0]) if abs(up_hint[0]) < 0.9 else np.array([0, 1.0, 0])
            up[collinear] = alt

        n = np.cross(up, t);
        n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
        b = np.cross(t, n);
        b /= (np.linalg.norm(b, axis=1, keepdims=True) + 1e-12)

        if tool_axis == "z":
            Rm = np.stack([n, b, t], axis=2)  # 列向量
        elif tool_axis == "x":
            Rm = np.stack([t, n, b], axis=2)
        elif tool_axis == "y":
            Rm = np.stack([n, t, b], axis=2)
        else:
            raise ValueError("tool_axis must be in {'x','y','z'}")

        quat_list = np.array([R.from_matrix(Rm[i]).as_quat() for i in range(Rm.shape[0])])
        return quat_list

    def smooth(self, quat_list, pos_list, n_seg):
        positions = []

        pos_list = np.array(pos_list)
        quat_list = np.array(quat_list)
        for quat, pos in zip(quat_list, pos_list):
            position = self.inverse_kinematic(quat, pos)
            print(position)
            positions.append(position)

        
        positions = np.array(positions)
        # todo: 基于这个positions列表，规划rucking smooth
        q_wp = self.ensure_2d_array(positions)
        grid_n = self.clamp(6 * n_seg, 300, 3000)
        t_list, positions, qd, qdd = self.toppra_time_parameterize(q_wp, self.v_max, self.a_max, self.dt, grid_n)
        return t_list, positions, qd, qdd

    def inverse_kinematic(self, quat, pos):
        rm = R.from_quat(quat).as_matrix()
        inverse_position = self.kinematic_solver.inverse_kinematic(rm, pos, initial_theta=self.nearest_position)
        self.nearest_position = inverse_position
        return inverse_position

    def _q_normalize(self, q):
        q = np.asarray(q, dtype=float)
        n = np.linalg.norm(q)
        if n == 0:
            raise ValueError("zero quaternion")
        return q / n

    def wrap_to_pi(self, q: np.ndarray) -> np.ndarray:
        """将角度包裹到 (-pi, pi]，避免 2π 跳变影响计算。"""
        return (q + np.pi) % (2 * np.pi) - np.pi

    def ensure_2d_array(self, arr: np.ndarray) -> np.ndarray:
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

    def toppra_time_parameterize(self,
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

        q = jnt_traj.eval(t)
        qd = jnt_traj.evald(t)
        qdd = jnt_traj.evaldd(t)
        return t.tolist(), q.tolist(), qd.tolist(), qdd.tolist()


start_position = [1.39,
                  -0.85,
                  -1.34,
                  -0.49,
                  -1.26,
                  1.57
                  ]

end_position = [-0.38,
                -1.34,
                -2.01,
                -0.7,
                -0.63,
                -1.53
                ]
start_pos = [-0.000093, 0.868412, 0.216573]
end_pos   = [0.607092, 0.050687, 0.254759]
mid_points = np.array([
    [0.1, 0.5, 0.23],
    [0.5,0.05,0.24]
])
way_pts = np.vstack([start_pos, mid_points, end_pos])
curve_motion_domain_service = CurveMotionDomainService()
pos_fun,s = curve_motion_domain_service.make_pos_fun_spline(way_pts, bc_type="natural")

u_vals = np.linspace(0, s[-1], 100)  # 100 个点
positions = np.array([pos_fun(u) for u in u_vals])

# 提取 x, y, z 坐标
x_vals = positions[:, 0]
y_vals = positions[:, 1]
z_vals = positions[:, 2]

# 绘制 3D 曲线图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_vals, y_vals, z_vals, label="Spline Path")
ax.scatter(way_pts[:, 0], way_pts[:, 1], way_pts[:, 2], color='red', label="Control Points")

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()

t_list, positions, qd, qdd = curve_motion_domain_service.curve_motion(
    pos_fun=pos_fun, 
    u0=0, 
    u1=1, 
    start_position=start_position, 
    end_position=end_position, 
    ds = 0.002, 
    include_end=True,
    orientation_mode = "slerp", 
    tool_axis = "z", 
    up_hint= np.array([0, 0, 1.0]))


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
