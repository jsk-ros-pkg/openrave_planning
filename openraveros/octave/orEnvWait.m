% isdone = orEnvWait(robotid, robot_timeout)
%
% wait until all previously sent commands to matlab are finished.
% Since problems are meant to last for a long time orEnvWait waits
% until the problem's main function finishes.
%
% robotid - optional argument. If a robot id is specified, will wait until
% the robot finishes with its trajectory.
%
% robot_timeout (s) - function will return with success set to 0 if robot
% did not finish its commands by robot_timeout ms. If not specified, orEnvWait
% will not return until robot completes.

function isdone = orEnvWait(robotid, robot_timeout)

session = openraveros_getglobalsession();
req = openraveros_env_wait();

if( exist('robotid','var') )
    req.bodyid = robotid;
end
if( exist('robot_timeout','var') )
    req.timeout = robot_timeout;
end

res = rosoct_session_call(session.id,'env_wait',req);

if(~isempty(res))
    isdone = res.isdone;
else
    isdone = [];
end
