from models import *

s_curve = SCurve()

start_angles =  [0.4654, -0.0369, -0.0792, 2.2555, -1.1260, -0.0001]
target_angles = [0, 0, 0, 0, 0, 0]
times, accelerations, velocities, positions = s_curve.planning(start_angles, target_angles)

print(positions)
