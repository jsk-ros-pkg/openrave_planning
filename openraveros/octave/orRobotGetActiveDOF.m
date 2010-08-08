% dof = orRobotGetActiveDOF(robotid)
%
% returns the robot's active degrees of freedom used for planning (not necessary corresponding to joints)

function dof = orRobotGetActiveDOF(robotid)
%% get some robot info
session = openraveros_getglobalsession();
req = openraveros_env_getrobots();
req.bodyid = robotid;
res = rosoct_session_call(session.id,'env_getrobots',req);
if( ~isempty(res) )
    if( res.robots{1}.bodyinfo.bodyid ~= robotid )
        error('wrong robot id');
    end
    dof = res.robots{1}.activedof;
else
    dof = [];
end
