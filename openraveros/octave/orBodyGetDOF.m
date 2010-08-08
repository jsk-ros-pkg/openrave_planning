% dof = orBodyGetDOF(robotid)
%
% returns the number of active joints of the body

function dof = orBodyGetDOF(bodyid)
session = openraveros_getglobalsession();
req = openraveros_body_getdof();
req.bodyid = bodyid;
res = rosoct_session_call(session.id,'body_getdof',req);

if(~isempty(res))
    dof = res.dof;
else
    dof = [];
end

