% success = orPlannerInit(planner, robot, parameters)
%
% Initialize a planner to plan for a robot and give some parameters

function success = orPlannerInit(plannerid, robotid, parameters)
session = openraveros_getglobalsession();
req = openraveros_planner_init();
req.plannerid = plannerid;
req.robotid = robotid;
req.params = parameters;
res = rosoct_session_call(session.id,'planner_init',req);
success = ~isempty(res);
