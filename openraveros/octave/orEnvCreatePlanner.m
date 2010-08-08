% plannerid = orEnvCreatePlanner(plannertype)

function plannerid = orEnvCreatePlanner(plannertype)
session = openraveros_getglobalsession();
req = openraveros_env_createplanner();
req.plannertype = plannertype;
res = rosoct_session_call(session.id,'env_createplanner',req);

if(~isempty(res))
    plannerid = res.plannerid;
else
    plannerid = [];
end
