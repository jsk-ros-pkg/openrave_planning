% success=orEnvClose(figureids)
%
% closes the figures and plots
% figureids - array of ids returned from orEnvPlot or other plotting functions
function success=orEnvClose(figureids)
session = openraveros_getglobalsession();
req = openraveros_env_closefigures();
if( exist('figureids','var') && ~isempty(figureids) )
    req.figureids = figureids(:);
end

res = rosoct_session_call(session.id,'env_closefigures',req);
success = ~isempty(res);
