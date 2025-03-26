from kinematic.velocity_planning import trapezoidal_velocity_planning, s_curve_velocity_planning

start_angles = [0, 0, 0, 0, 0, 0]
target_angles = [1.57, 1, 1, 1, 1, 1]
trajectory = s_curve_velocity_planning(start_angles, target_angles, dt=1)
print(trajectory[3])
