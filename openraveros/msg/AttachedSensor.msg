# information specific to an attached sensor for a robot

# name of sensor
string name

# zero-based index of link sensor is attached to
int8 attachedlink

# 3x4 matrix of the relative transform of the sensor frame with respect to the link frame
AffineTransformMatrix trelative

# 3x4 matrix of the global transform of the sensor (ie, with link transform applied)
AffineTransformMatrix tglobal

# type of sensor
string type
