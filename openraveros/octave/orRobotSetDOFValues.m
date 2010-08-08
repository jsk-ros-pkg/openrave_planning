% orRobotSetDOFValues(robotid, values, indices)
%
% Sets the DOF values of the robot
% robotid - unique id of the robot
% values - the joint values of the robot
% indices [optional] - the indices of the dofs to set of the robot. 
%                      If indices is not specified the active degrees of freedom
%                      set by previous calls to orRobotSetActiveDOFs will be used.
%                      Note that specifying indices will not change the active dofs
%                      of the robot.

function success = orRobotSetDOFValues(robotid, values, indices)
session = openraveros_getglobalsession();
req = openraveros_robot_setactivevalues();
req.bodyid = robotid;
req.values = values(:);
if( exist('indices','var') )
    req.indices = indices(:);
end
res = rosoct_session_call(session.id,'robot_setactivevalues',req);
success = ~isempty(res);
