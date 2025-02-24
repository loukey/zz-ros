from math import pi
import numpy as np
from core.core.kinematic import *
import math


if __name__ == "__main__":
    k = Kinematic6DOF()
    # [0.22087425776630992, -0.06578997882737658, 1.582684426523854, 0.053901853980430126, 1.5707963016769075, -1.7916705594432174])
    # temp = k.inverse_kinematic(2.94185109, 0.69444419, 1.01555572, 0.37576529, 0.29737335, -0.69803581)
    # print(temp)
    # for r in k.rm_list:
    #     print(r)
    rm = k.forward_kinematic()
    print(rm)   
    euler = XYZ_rotation_matrix_to_euler_angles(rm)
    print(euler)
    temp = k.inverse_kinematic(0,0,0,0,0.158,0.5)
    k.test_kinematic([0,0,0,1.57,0,0])
    temp = [math.degrees(temp) for temp in temp]
    
