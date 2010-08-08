% bodyid = orEnvCreateKinBody(name, xmlfile)

function bodyid = orEnvCreateKinBody(name, xmlfile)
session = openraveros_getglobalsession();
req = openraveros_env_createbody();
req.name = name;
req.file = xmlfile;
res = rosoct_session_call(session.id,'env_createbody',req);

if(~isempty(res))
    bodyid = res.bodyid;
else
    bodyid = [];
end

