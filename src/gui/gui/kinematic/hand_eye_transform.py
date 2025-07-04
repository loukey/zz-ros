from .kinematic_6dof import *


class HandEyeTransform:
    def __init__(self):
        self.hand_eye_transform_rm = np.array([
            [ 0.9996605 ,  0.02568488,  0.00437868, -0.01748838],
            [-0.02513582,  0.99491608,  0.09752014, -0.06800609],
            [-0.00686121,  0.09737697,  0.99522392, -0.03227995],
            [ 0.        ,  0.        ,  0.        ,  1.        ]
        ])
        # self.hand_eye_transform_rm = np.array([
        #     [ 1,  0,  0, 0],
        #     [ 0,  1,  0, -0.07],
        #     [ 0,  0,  1, 0.0085],
        #     [ 0,  0,  0, 1]
        # ])
        self.kinematic_solver = Kinematic6DOF()
        self.fx = 721.717082498008
        self.fy = 720.0607366770212
        self.cx = 641.0458509844366
        self.cy = 318.7078463463553

    def set_hand_eye_transform(self, hand_eye_transform_rm):
        self.hand_eye_transform_rm = hand_eye_transform_rm

    def get_theta_list(self, pos):
        px, py, pz = pos
        px = (px - self.cx) / self.fx
        py = (py - self.cy) / self.fy
        pz *= 0.001
        print('相机:', px, py, pz)
        end_pos = self.hand_eye_transform_rm @ np.array([px, py, pz, 1])
        print('夹爪:', end_pos)
        origin_theta_list = [0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]
        theta_list_now = [0.0590, -0.2400, 0.9288, -0.0674, 1.4647, 0.0276]
        theta_list_now = (np.array(theta_list_now) + np.array(origin_theta_list)).tolist()
        print(theta_list_now)
        self.kinematic_solver.update_dh(theta_list_now)
        forward_rm = self.kinematic_solver.get_forward_rm()
        print(forward_rm)
        base_pos = forward_rm @ end_pos
        print('基座', base_pos)
        theta_list = self.kinematic_solver.inverse_kinematic(forward_rm[:3, :3], base_pos[:3])

        return theta_list
