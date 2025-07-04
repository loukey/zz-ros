from .kinematic_6dof import *


class HandEyeTransform:
    def __init__(self):
        self.hand_eye_transform_rm = np.eye(4)
        self.kinematic_solver = Kinematic6DOF()

    def set_hand_eye_transform(self, hand_eye_transform_rm):
        self.hand_eye_transform_rm = hand_eye_transform_rm

    def get_theta_list(self, pos):
        px, py, pz = pos
        end_pos = self.hand_eye_transform_rm @ np.array([px, py, pz, 1])
        forward_rm = self.kinematic_solver.get_forward_rm()
        theta_list = self.kinematic_solver.inverse_kinematic(forward_rm[:3, :3], end_pos)

        return theta_list
