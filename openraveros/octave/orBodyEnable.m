% success = orBodyEnable(bodyid, enable)
%
% Enables or disables the body. If a body is disabled, collision detection
% and physics will will be turned off for it.

function success = orBodyEnable(bodyid,enable)
session = openraveros_getglobalsession();
req = openraveros_body_enable();
req.bodyid = bodyid;
req.enable = enable;
res = rosoct_session_call(session.id,'body_enable',req);
success = ~isempty(res);
