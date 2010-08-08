% success = orRobotControllerSet(robotid, controllername, controllerargs)
%
% Sets a new robot controller and destroys the old.
% controllername - name used to query a controller
% controllerargs [optional] - the arguments to ControllerBase::Init

function success = orRobotControllerSet(robotid, controllername, controllerargs)
session = openraveros_getglobalsession();
req = openraveros_robot_controllerset();
req.bodyid = robotid;
req.controllername = controllername;
if( exist('controllerargs','var') )
    req.controllerargs = controllerargs;
end
res = rosoct_session_call(session.id,'robot_controllerset',req);
success = ~isempty(res);
