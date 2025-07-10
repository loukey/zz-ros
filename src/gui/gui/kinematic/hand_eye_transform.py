from .kinematic_6dof import *
from gui.config import GlobalVars


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
        c = cos(rotation_angle)
        s = sin(rotation_angle)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def get_Y_rotation_matrix(self, rotation_angle):
        c = cos(rotation_angle)
        s = sin(rotation_angle)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0,c, 0],
            [0, 0, 0, 1]
        ])

    def get_X_rotation_matrix(self, rotation_angle):
        c = cos(rotation_angle)
        s = sin(rotation_angle)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
    
    def get_XYZ_rotation_matrix(self, x_angle, y_angle, z_angle):
        c_x = cos(x_angle)
        s_x = sin(x_angle)
        c_y = cos(y_angle)
        s_y = sin(y_angle)
        c_z = cos(z_angle)
        s_z = sin(z_angle)
        return np.array([
            [c_y*c_z, -c_x*s_z + s_x*s_y*c_z, s_x*s_z + c_x*s_y*c_z, 0],
            [c_y*s_z, c_x*c_z + s_x*s_y*s_z, -s_x*c_z + c_x*s_y*s_z, 0],
            [-s_y, s_x*c_y, c_x*c_y, 0],
            [0, 0, 0, 1]
        ])

    def get_theta_list(self, pos, rotation_angle):
        px, py, pz = pos
        pz *= 0.001
        px = (px - self.cx) * pz / self.fx
        py = (py - self.cy) * pz / self.fy     
        v_cam = np.array([-sin(rotation_angle), cos(rotation_angle), 0])

        theta_list_now = GlobalVars.get_current_joint_angles()
        print("thetalist_now:",theta_list_now)
        forward_rm = self.kinematic_solver.update_dh(theta_list_now)
        T_cam2base = forward_rm @ self.hand_eye_transform_rm  
        print("T_cam2base:",T_cam2base)
        R_cam2base = T_cam2base[:3,:3]

        p_cam = np.array([px, py, pz, 1])
        p_base = T_cam2base @ p_cam
        print("p_base",p_base)
        v_base = R_cam2base @ v_cam
        
        theta = np.arctan2(v_base[1], v_base[0])
        print("theta:",theta)
        T_target2base = self.get_Z_rotation_matrix(theta)
        T_target2base[:, 3] = p_base
        print("T_target2base:",T_target2base)
        
        T_offset2target = np.eye(4)
        T_offset2target[:3,3] = np.array([0.01, 0, 0.05])
        T_target2base = T_target2base @ T_offset2target
        T_target2base = T_target2base @ self.get_Y_rotation_matrix(-135*pi/180) @ self.get_Z_rotation_matrix(pi/2)
        theta_list = self.kinematic_solver.inverse_kinematic(T_target2base[:3, :3], T_target2base[:3, 3])
        print(theta_list)

        return theta_list


