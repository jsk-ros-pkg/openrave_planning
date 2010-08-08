% problemid = orEnvCreateProblem(problemtype, args, destroyduplicates)
% 
% Creates an instance of a problem and returns its id for future communicate with it
% problemtype - the problem type
% args - a string of arguments to send to the problem's main function
% destroyduplicates [optional] - if 1, will destroy any previous problems with the same problem name.
%                                If 0, will not destroy anything.
%                                The default value is 1. 
function problemid = orEnvCreateProblem(problemtype, args, destroyduplicates)

session = openraveros_getglobalsession();
req = openraveros_env_createproblem();
req.problemtype = problemtype;
if( exist('args','var') )
    req.args = args;
end
if( exist('destroyduplicates', 'var') )
    req.destroyduplicates = destroyduplicates;
else
    req.destroyduplicates = 1;
end
res = rosoct_session_call(session.id,'env_createproblem',req);

if(~isempty(res))
    problemid = res.problemid;
else
    problemid = [];
end
