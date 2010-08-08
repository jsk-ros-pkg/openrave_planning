% id = orEnvGetBody(bodyname)
%
% returns the id of the body that corresponds to bodyname

function id = orEnvGetBody(bodyname)
session = openraveros_getglobalsession();
req = openraveros_env_getbody();
req.name = bodyname;
res = rosoct_session_call(session.id,'env_getbody',req);

if(~isempty(res))
    id = res.bodyid;
else
    id = 0;
end
