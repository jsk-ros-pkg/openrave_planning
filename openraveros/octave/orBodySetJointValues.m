% success=orBodySetJointValues(bodyid, values, indices)
%
% Set the raw joint values of a body. If bodyid is a robot, sets the robot's
% joints ignoring its current active degrees of freedom. If a controller on
% the robot is running, this function might not have any effect. Instead
% use orRobotSetDOFValues
% indices [optional] - array specifying the indices to control

function success = orBodySetJointValues(bodyid, values, indices)
session = openraveros_getglobalsession();
req = openraveros_body_setjointvalues();
req.bodyid = bodyid;
req.jointvalues = values;
if( exist('indices','var') )
    req.indices = indices(:);
end
res = rosoct_session_call(session.id,'body_setjointvalues',req);
success = ~isempty(res);
