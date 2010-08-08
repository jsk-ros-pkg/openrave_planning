% success=orBodyDestroy(bodyid)
%
% Destroys a body of id bodyid. bodyid can also be a robot.

function success = orBodyDestroy(bodyid)
session = openraveros_getglobalsession();
req = openraveros_body_destroy();
req.bodyid = bodyid;
res = rosoct_session_call(session.id,'body_destroy',req);
success = ~isempty(res);
