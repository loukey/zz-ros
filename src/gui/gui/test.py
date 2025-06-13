from utils import *
from kinematic import *

positions = [78555, 341827, 122306, 391368, 508057, 455126]
radians = position_to_radian(positions)

print(radians)
def radians_to_angles(radians):
    angles = []
    for radian in radians:
        angle = radian * 180 / pi
        angles.append(angle)
    return angles

angles = radians_to_angles(radians)
print(angles)

dynamic = Dynamic()
torques = dynamic.compute_gravity_compensation(radians)
print(torques)
