% values = orRobotGetDOFValues(robotid, indices)
%
% Gets the robot's dof values in a Nx1 vector where N is the DOF
% robotid - unique id of the robot
% indices [optional]- The indices of the joints whose values should be returned.
%                     If not specified, the active degreees of freedeom set by
%                     orRobotSetActiveDOFs will be used.

function values = orRobotGetDOFValues(robotid, indices)

session = openraveros_getglobalsession();
req = openraveros_robot_getactivevalues();
req.bodyid = robotid;
if( exist('indices','var') )
    req.indices = indices(:);
end
res = rosoct_session_call(session.id,'robot_getactivevalues',req);

if(~isempty(res))
    values = res.values;
else
    values = [];
end
