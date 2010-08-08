% robotid = orEnvCreateRobot(robotname, xmlfile, type)
%
% Creates a robot of the given type. If type is not specified, creates a generic robot

function bodyid = orEnvCreateRobot(robotname, xmlfile, type)

session = openraveros_getglobalsession();
req = openraveros_env_createrobot();
req.name = robotname;
req.file = xmlfile;
if( exist('type','var') )
    req.type = type;
else
    req.type = 'GenericRobot';
end
res = rosoct_session_call(session.id,'env_createrobot',req);

if(~isempty(res))
    bodyid = res.bodyid;
else
    bodyid = [];
end
