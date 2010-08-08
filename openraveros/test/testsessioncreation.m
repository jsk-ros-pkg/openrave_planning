%% tests session creation and destruction, simple loading, and attaching viewers
addpath(fullfile(pwd,'..','octave')); % in case launched from test folder

openraveros_restart();
orEnvSetOptions('debug verbose');

if( isempty(openraveros_getglobalsession()) )
    error('empty session');
end
sessionname = openraveros_getglobalsession().server;
openraveros_destroysession();
if( ~isempty(openraveros_getglobalsession()) )
    error('valid session');
end

req = openraveros_openrave_session();

openraveros_setglobalsession(openraveros_createsession());
if( isempty(openraveros_getglobalsession()) )
    error('empty session');
end

display('creating empty sessions');
sessions = {};
for i = 1:10
    session = openraveros_createsession('openrave_session');
    if( isempty(session) )
        error('failed');
    end
    sessions{end+1} = session;
end

%% make clones
display('creating empty clone sessions');
for i = 1:10
    session = openraveros_createsession('openrave_session',sessions{i}.uuid,req.CloneBodies());
    if( isempty(session) )
        error('clone failed');
    end
    sessions{end+1} = session;
    openraveros_setglobalsession(session);
end

display('destroying empty sessions');
for i = 1:20
    openraveros_destroysession(sessions{i});
end
sessions = {};

openraveros_restart();
% for i = 1:4
%     orEnvSetOptions('viewer qtcoin');
%     orEnvLoadScene('data/lab1.env.xml');
% end

orEnvLoadScene('data/lab1.env.xml');
orEnvSetOptions('viewer qtcoin');
orEnvLoadScene('robots/barrettwam.robot.xml',1);
