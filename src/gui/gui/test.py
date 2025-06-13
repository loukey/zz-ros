from kinematic import *

kinematic = Kinematic6DOF()

test_theta = [pi, -pi/10, -pi/8, pi/2, pi/5, -pi/4]

kinematic.test_kinematic(test_theta)
