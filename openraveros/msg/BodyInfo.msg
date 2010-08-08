# information about a body
int32 bodyid

# current transformation
AffineTransformMatrix transform

# degrees of freedom (number of joints)
uint8 dof

# enabled status
uint8 enabled


# filename used to create body geometry
string filename

# name of the body
string name

# type of body
string type

float32[] jointvalues
AffineTransformMatrix[] links
string[] linknames
string[] jointnames

# joint limits
float32[] lowerlimit
float32[] upperlimit

# request information bitmasks, also holds robot specific request information (lower 8 bits)
uint16 Req_JointValues=1
uint16 Req_Links=2
uint16 Req_LinkNames=4 # fills the linknames
uint16 Req_JointLimits=8
uint16 Req_Names=16 # if set, fills filename, name, and type
uint16 Req_JointNames=32 # fills the jointnames