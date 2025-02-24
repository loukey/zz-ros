from math import pi
import numpy as np
from core.core.kinematic import *
import math


if __name__ == "__main__":
    k = Kinematic6DOF()
    # temp = k.inverse_kinematic(0,0,0,0.2,0.2,0.3)
    k.test_kinematic([0.47793504240980056, -2.025109349713061, 2.420328398253977, 1.1755772782539804, 1.5707963267948966, -2.048731369204697])
    # temp = [math.degrees(temp) for temp in temp]
    # print(temp)
    