% trajectory = orPlannerPlan(planner)
%
% Start planning. The planner returns a trajectory when successful (otherwise returns an empty matrix)
% The trajectory is an (DOF+1)xN matrix where N is the number of points in the trajectory. The first row
% are the time values of each trajectory point.


function trajectory = orPlannerPlan(plannerid)
session = openraveros_getglobalsession();
req = openraveros_planner_plan();
req.plannerid = plannerid;
res = rosoct_session_call(session.id,'planner_plan',req);

if(~isempty(res))
    trajectory = res.trajectory;
else
    trajectory = [];
end
