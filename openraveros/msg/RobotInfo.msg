# robot information

BodyInfo bodyinfo
Manipulator[] manips
AttachedSensor[] sensors

# total active dof
uint8 activedof

# current active manipulator
uint8 activemanip

# information about the active dofs
ActiveDOFs active

# joint limits
float32[] activelowerlimit
float32[] activeupperlimit

# upper 8 bits
uint16 Req_Manipulators=256
uint16 Req_Sensors=512
uint16 Req_ActiveDOFs=1024
uint16 Req_ActiveLimits=2048
