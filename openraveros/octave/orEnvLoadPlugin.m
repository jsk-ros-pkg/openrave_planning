% success = orEnvLoadPlugin(filename)
%
% Loads a plugin.
% filename - the relative path of the plugin to load. (*.so for linux, *.dll for windows)

function success = orEnvLoadPlugin(filename)
session = openraveros_getglobalsession();
req = openraveros_env_loadplugin();
req.filename = filename;
res = rosoct_session_call(session.id,'env_loadplugin',req);
success = ~isempty(res);
