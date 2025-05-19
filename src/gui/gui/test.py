# from models.trajectory_model import Linear

# linear = Linear()
# times, positions = linear.planning(start_angles=[0,0,0,0,0,0], 
#                                    target_angles=[0,0,0,0,0,0], 
#                                    dt=0.1)

# print(times)
# print(positions)

from utils import *
import math

print((2**19) / (2 * math.pi) * 3.1416)
print(position_to_radian(0))