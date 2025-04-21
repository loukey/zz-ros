from kinematic import *

start_angles = [0, 0, 0, 0, 0, 0]
target_angles = [pi/100, pi/38, 0, 0, 0, 0]

print(target_angles[0:2])
s_curve = SCurve()
times, accelerations, velocities, positions = s_curve.planning(start_angles=start_angles, 
                                                                                target_angles=target_angles,
                                                                                v_start=[0] * 6,)


print(positions)