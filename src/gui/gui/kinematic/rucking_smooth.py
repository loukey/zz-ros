from math import pi
from ruckig import Ruckig, InputParameter, OutputParameter, Result
import numpy as np
from .kinematic_6dof import *


class RuckigSmooth:
    def __init__(self, v_max=[pi/4] * 6, a_max=[pi/8] * 6, j_max=[pi/16] * 6, dt=0.01):
        self.v_max = v_max
        self.a_max = a_max
        self.j_max = j_max
        self.dt = dt
        self.kinematic_solver = Kinematic6DOF()

    def rucking_smooth(self, start_position, end_position):
        start_quat, start_pos = self.kinematic_solver.get_end_position(start_position)
        end_quat, end_pos = self.kinematic_solver.get_end_position(end_position)
        quat_list, pos_list = self.sampling(start_quat, start_pos, end_quat, end_pos)
        positions = self.smooth(quat_list, pos_list)
        return positions

    def rucking_smooth_z_axis(self, start_position, distance, axis=-1):
        start_quat, start_pos = self.kinematic_solver.get_end_position(start_position)
        # todo: 按照distance计算终点位置
        pass

    def sampling(self, start_quat, start_pos, end_quat, end_pos, sampling_dis=0.02):
        quat_list = []
        pos_list = []
        # todo: 按照起始四元数+位置和终点四元数+位置以及采样间隔0.02m，进行采样
        return quat_list, pos_list

    def smooth(self, quat_list, pos_list):
        positions = []
        for quat, pos in zip(quat_list, pos_list):
            position = self.inverse_kinematic(quat, pos)
            positions.append(position)
        positions = np.array(positions)
        # todo: 基于这个positions列表，规划rucking smooth
        return positions

    def inverse_kinematic(self, quat, pos):
        rm = R.from_quat(quat).as_matrix()
        return self.kinematic_solver.inverse_kinematic(rm, pos)
