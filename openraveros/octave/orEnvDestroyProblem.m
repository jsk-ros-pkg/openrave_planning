% success=orEnvDestroyProblem(problemid)
%
% Destroys problem instance whose id is problemid.

function success = orEnvDestroyProblem(problemid)
session = openraveros_getglobalsession();
req = openraveros_env_destroyproblem();
req.problemid = problemid;
res = rosoct_session_call(session.id,'env_destroyproblem',req);
success = ~isempty(res);
