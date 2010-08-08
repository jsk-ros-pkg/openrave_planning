% values = orBodyGetJointValues(bodyid, indices)
%
% Gets the body's dof values in a Nx1 vector where N is the DOF
% bodyid - unique id of the robot
% indices [optional]- The indices of the joints whose values should be returned.
%                     If not specified, all joints are returned
function values = orBodyGetJointValues(bodyid, indices)
session = openraveros_getglobalsession();
req = openraveros_body_getjointvalues();
req.bodyid = bodyid;
if( exist('indices','var') )
    req.indices = indices(:);
end
res = rosoct_session_call(session.id,'body_getjointvalues',req);

if(~isempty(res))
    values = res.values;
else
    values = [];
end
