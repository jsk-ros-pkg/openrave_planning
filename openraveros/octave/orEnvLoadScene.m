% success = orEnvLoadScene(filename, [ClearScene])
%
% Loads a new environment.
% filename - The filename of the scene to load. If a relative file
%            is specified, note that it is relative to the current direction
%            of the OpenRAVE executable.
% ClearScene - If 1, then clears the scene before loading. Else leaves the
%              scene alone and loads in addition to it.

function success = orEnvLoadScene(filename, ClearScene)
session = openraveros_getglobalsession();
req = openraveros_env_loadscene();
req.filename = filename;
if( exist('ClearScene','var') )
    req.resetscene = ClearScene;
end
res = rosoct_session_call(session.id,'env_loadscene',req);
success = ~isempty(res);
