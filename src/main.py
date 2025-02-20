from core.core.kinematic.kinematic_6dof import Kinematic6DOF
from math import pi

if __name__ == "__main__":
    k = Kinematic6DOF()
    k.test_kinematic([pi/9, pi/4, pi/5, pi/6, pi/7, pi/8])
    