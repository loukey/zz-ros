from .kinematic_6dof import *
from config import GlobalVars


class HandEyeTransform:
    def __init__(self):
        self.hand_eye_transform_rm = np.array([
            [ 0.99927428,  0.03697782, -0.00914077, -0.00990620],
            [-0.03793413,  0.98783005, -0.15084031, -0.06102098],
            [ 0.00345179,  0.15107759,  0.98851588, -0.22443320],
            [ 0.        ,  0.        ,  0.        ,  1.        ]
        ])
        self.kinematic_solver = Kinematic6DOF()
        self.fx = 721.717082498008
        self.fy = 720.0607366770212
        self.cx = 641.0458509844366
        self.cy = 318.7078463463553

    def set_hand_eye_transform(self, hand_eye_transform_rm):
        self.hand_eye_transform_rm = hand_eye_transform_rm

    def get_Z_rotation_matrix(self, rotation_angle):
        return np.array([
            [cos(rotation_angle), -sin(rotation_angle), 0, 0],
            [sin(rotation_angle), cos(rotation_angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def get_theta_list(self, pos, rotation_angle):
        px, py, pz = pos
        px = (px - self.cx) / self.fx
        py = (py - self.cy) / self.fy
        pz *= 0.001
        rotation_angle *= pi / 180
        z_rotate_m = self.get_Z_rotation_matrix(rotation_angle)
        part_pos = np.zeros((4, 4))
        part_pos[:3, :3] = z_rotate_m[:3, :3]
        part_pos[:3, 3] = [px, py, pz]
        part_pos[3, 3] = 1
        end_pos = self.hand_eye_transform_rm @ part_pos
        theta_list_now = GlobalVars.get_current_joint_angles()
        print("获取当前关节角度", theta_list_now)
        self.kinematic_solver.update_dh(theta_list_now)
        forward_rm = self.kinematic_solver.get_forward_rm()
        forward_rm = forward_rm @ z_rotate_m
        base_pos = forward_rm @ end_pos
        print("计算目标位置", base_pos)
        theta_list = self.kinematic_solver.inverse_kinematic(base_pos[:3, :3], base_pos[:3, 3])
        origin_theta_list = [0, -pi/2, 0, pi/2, 0, 0]
        theta_list = (np.array(theta_list) - np.array(origin_theta_list)).tolist()
        print("计算目标关节角度", theta_list)

        return theta_list
