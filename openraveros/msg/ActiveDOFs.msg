# specifies the active degrees of freedom of the robot

# mask of active base degrees of freedom
uint32 affine

# active joints
int32[] indices

float32[3] rotationaxis

# mask for affine dofs
uint8 DOF_X = 1
uint8 DOF_Y = 2
uint8 DOF_Z = 4
uint8 DOF_RotationAxis = 8
uint8 DOF_Rotation3D = 16
uint8 DOF_RotationQuat = 32
