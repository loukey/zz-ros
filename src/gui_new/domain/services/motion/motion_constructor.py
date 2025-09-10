from .motion_runner import MotionRunner
from ..algorithm import SCurve


class MotionConstructor:
    def __init__(self, motion_runner: MotionRunner):
        self.motion_runner = motion_runner
        self.data_list = []
        self.s_curve = SCurve()

    def clear_data(self):
        self.data_list = []

    def add_motion_data(self, motion_type, data):
        self.data_list.append((motion_type, data))

    def construct_motion_data(self, start_position):
        self.motion_runner.clear_data()
        last_position = start_position
        for motion_type, data in self.data_list:
            if motion_type == "motion":
                _, _, _, positions = self.s_curve.planning(last_position, data)
                self.motion_runner.add_motion_data(positions)
                last_position = data
            elif motion_type == "gripper":
                self.motion_runner.add_gripper_data(**data)
    