from models.trajectory_model import SCurve

s_curve = SCurve()

times, accelerations, velocities, positions = s_curve.planning(start_angles=[0,0,0,0,0,0], 
                                                                target_angles=[-0.5,1.3,0,0,0,0], 
                                                                dt=0.1)

print(times)
print(positions)
