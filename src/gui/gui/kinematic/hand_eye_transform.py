from .kinematic_6dof import *


class HandEyeTransform:
    def __init__(self):
        hand_eye_transform_rm = np.eye(4)
        self.kinematic_solver = Kinematic6DOF()

    def set_hand_eye_transform(self, hand_eye_transform_rm):
        self.hand_eye_transform_rm = hand_eye_transform_rm

    def get_theta_list(self, pos):
        px, py, pz = pos
        end_pos = self.hand_eye_transform_rm @ np.array([px, py, pz, 1])

    def get_end_rm(self, rotation_angle):
        pass

