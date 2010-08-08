% sensors = orRobotGetAttachedSensors(robotid)
%
% sensors is a cell array describing the attached sensors of the robot
% Each cell is a struct with fields:
%   name - name of the attached sensor
%   link - one-based index of link sensor is attached to
%   Trelative - 3x4 matrix of the relative transform of the camera with respect to the robot
%   Tglobal - 3x4 matrix of the global transform of the sensor of the current robot
%             Tglobal = Tlink * Trelative
%   type - the xml id of the sensor that is attached
function sensors = orRobotGetAttachedSensors(robotid)
robot = orEnvGetRobots(robotid);
if( isempty(robot) )
    sensors = [];
else
    sensors = robot.sensors;
end
