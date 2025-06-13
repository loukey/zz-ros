from kinematic import *
from utils import *

positions = [78555, 341827, 122306, 391368, 508057, 455126]
radians = position_to_radian(positions)
print(radians)

dynamic = Dynamic()
torques = dynamic.compute_gravity_compensation(radians)
print(torques)
