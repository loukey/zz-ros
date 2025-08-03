from dataclasses import dataclass
import numpy as np
from math import pi

@dataclass
class DHParam:
    kinematic_dh: np.array
    dynamic_dh: np.array

    def __init__(self):
        self.kinematic_dh = np.array([
            [0.0, 0.0, 0.0,  0.0],
            [0.0, -pi/2, 0.0, -pi/2],
            [0.425, pi, 0.0, 0.0],
            [0.401, pi, 0.0856, pi/2],
            [0.0, pi/2, 0.086, 0.0],
            [0.0, -pi/2, 0.2785, 0.0]
        ], dtype=float)

        self.dynamic_dh = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, -pi/2, 0.0, -pi/2],
            [0.425, pi, 0.0, 0.0],
            [0.401, pi, 0.0856, pi/2],
            [0.0, pi/2, 0.086, 0.0],
            [0.0, -pi/2, 0.0725, 0.0]
        ])

    def get_kinematic_dh(self):
        return self.kinematic_dh

    def get_dynamic_dh(self):
        return self.dynamic_dh
