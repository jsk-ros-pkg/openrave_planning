# information specific to a manipulator for a robot

# zero-based index of base link that manipulator is attached to
int8 baselink

# zero-based index of link defining the end-effector
int8 eelink

# 3x4 matrix of the grasp frame relative to the end-effector link
AffineTransformMatrix tgrasp

# zero-based hand joint indices
uint8[] handjoints

# zero-based arm joints indices (from base to end-effector)
uint8[] armjoints

# name of ik solver using
string iksolvername

# manipulator name
string name
