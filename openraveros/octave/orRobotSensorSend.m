% out = orRobotSensorSend(robotid, sensorindex, cmd,args)
%
% sends a command to a sensor attached to the robot 
% OpenRAVE sends directly to SensorBase::SendCmd,
% SensorBase::SupportsCmd is used to check for command support.
%
% robotid - unique id of the robot
% sensorindex - zero-based index of sensor into robot's attached sensor array
% out - the output of the command
function sout = orRobotSensorSend(robotid, sensorindex, cmd, args)
session = openraveros_getglobalsession();
req = openraveros_robot_sensorsend();
req.bodyid = robotid;
req.sensorindex = sensorindex;
req.cmd = cmd;
if( exist('args','var') )
    req.args = args;
end

res = rosoct_session_call(session.id,'robot_sensorsend',req);
if( ~isempty(res) )
    sout = res.out;
else
    sout = [];
end
