% [success,output] = orRobotControllerSend(robotid, cmd)
%
% sends a command to the current controller the robot is connected to.
% OpenRAVE sends directly to ControllerBase::SendCmd,
% ControllerBase::SupportsCmd is also used to check for support.
%
% success - 1 if command was accepted, 0 if not

function [success,output] = orRobotControllerSend(robotid, cmd)
session = openraveros_getglobalsession();
req = openraveros_robot_controllersend();
req.bodyid = robotid;
req.cmd = cmd;
res = rosoct_session_call(session.id,'robot_controllersend',req);
success = ~isempty(res);
output = [];
if( success )
    output = res.output;
end
