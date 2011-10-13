trajectory_msgs/JointTrajectory traj

# interpolation method to use for the trajectory points, one of Interp_ consts.
# This parameter must be followed strictly if all collision obstacles are to be avoided.
uint8 interpolation

uint8 Interp_None = 0 # doesn't matter
uint8 Interp_Linear = 1 # linear interpolation using velocities
uint8 Interp_Quadratic = 2 # quadratic blends with max accelerations
uint8 Interp_Cubic = 3 # cubic spline interpolation
uint8 Interp_Quadric = 4 # quadric interpolation
uint8 Interp_Quintic = 4 # quintic min-jerk interpolation
