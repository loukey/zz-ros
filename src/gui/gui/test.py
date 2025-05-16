# from models.trajectory_model import Linear

# linear = Linear()
# times, positions = linear.planning(start_angles=[0,0,0,0,0,0], 
#                                    target_angles=[0,0,0,0,0,0], 
#                                    dt=0.1)

# print(times)
# print(positions)

from models import *

s_curve = SCurve()
times, _, _, positions = s_curve.planning(start_angles=[0,0,0,0,0,0], 
                                    target_angles=[0,0,0,1.57,0,0], 
                                    dt=0.1)

print(times)
print(positions)
