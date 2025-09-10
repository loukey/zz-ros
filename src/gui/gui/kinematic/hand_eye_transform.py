from .kinematic_6dof import *
from gui.config import GlobalVars


class HandEyeTransform:
    def __init__(self):
        self.hand_eye_transform_rm = np.array([
            [ 0.99994527,  0.01043147,  -0.00079424, -0.0069770],
            [-0.01040318,   0.99950205,  0.02978964, -0.0510317],
            [0.001104600,  -0.0297797,  0.99555875, -0.1496010],
            [ 0,          0,          0,          1]
        ])
        self.kinematic_solver = Kinematic6DOF()
        self.fx = 678.88344562
        self.fy = 681.85313997
        self.cx = 645.31442426
        self.cy = 380.55650809

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

    def get_theta_list(self, pos, real_pos, rotation_angle):
        px1, py1, pz1 = pos
        px2, py2, pz2 = real_pos
        pz1 *= 0.001
        pz2 *= 0.001
        px1 = (px1 - self.cx) * pz1 / self.fx
        py1 = (py1 - self.cy) * pz1 / self.fy
        px2 = (px2 - self.cx) * pz2 / self.fx  
        py2 = (py2 - self.cy) * pz2 / self.fy  
        p1_cam = np.array([px1, py1, pz1, 1])
        p2_cam = np.array([px2, py2, pz2, 1])


        theta_list_now = GlobalVars.get_current_joint_angles()
        print("thetalist_now:",theta_list_now)
        forward_rm = self.kinematic_solver.update_dh(theta_list_now)
        T_cam2base = forward_rm @ self.hand_eye_transform_rm  
        print("T_cam2base:",T_cam2base)
        R_cam2base = T_cam2base[:3,:3]

        p1_base = T_cam2base @ p1_cam
        print("p1_base",p1_base)
        p2_base = T_cam2base @ p2_cam
        print("p2_base",p2_base)
        v_base = p2_base - p1_base
        
        theta = np.arctan2(v_base[1], v_base[0])
        theta = theta 
        print("theta:",theta)
        T_target2base = self.get_Z_rotation_matrix(theta)
        T_target2base[:, 3] = p2_base 
        print("T_target2base:",T_target2base)
        
        T_offset2target = np.eye(4)
        T_offset2target[:3,3] = np.array([0.2, 0, 0.1])
        T_target2base = T_target2base @ T_offset2target
        T_target2base = T_target2base @ self.get_Y_rotation_matrix(-90*pi/180) @ self.get_X_rotation_matrix(-pi/2)
        print("T_target2base2:",T_target2base)
        theta_list = self.kinematic_solver.inverse_kinematic(T_target2base[:3, :3], T_target2base[:3, 3])
        print(theta_list)

        return theta_list


