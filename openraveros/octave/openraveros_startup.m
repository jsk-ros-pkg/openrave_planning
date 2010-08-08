%% openraveros_startup(sessionserver, createsession, viewer, destroyall)
%%
%% adds all the necessary paths for the openraveros octave client
%% if destroyall is 1 and openraveros cannot find a currently existing session,
%%   destroys all other sessions before creating a new one

%% Software License Agreement (BSD License)
%% Copyright (c) 2008, Willow Garage, Inc.
%% Redistribution and use in source and binary forms, with or without
%% modification, are permitted provided that the following conditions are met:
%%   * Redistributions of source code must retain the above copyright notice,
%%     this list of conditions and the following disclaimer.
%%   * Redistributions in binary form must reproduce the above copyright
%%     notice, this list of conditions and the following disclaimer in the
%%     documentation and/or other materials provided with the distribution.
%%   * The name of the author may not be used to endorse or promote products
%%     derived from this software without specific prior written permission.
%%
%% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%% POSSIBILITY OF SUCH DAMAGE.
%%
%% author: Rosen Diankov
function openraveros_startup(sessionserver,createsession, viewer, destroyall)
global openraveros_globalsession
persistent openraveros_initialized

if( ~exist('createsession','var') )
    createsession = 1;
end
if( ~exist('destroyall','var') )
    destroyall = 0;
end

if( isempty(openraveros_initialized))
    [status,rosoctpath] = system('rospack find rosoct');
    rosoctpath = strtrim(rosoctpath);
    addpath(fullfile(rosoctpath,'octave'));

    rosoct_add_msgs('openraveros');
    rosoct_add_srvs('openraveros');

    addpath(fullfile(rosoct_findpackage('sensor_msgs'),'octave'));

    rosoct('shutdown'); % restart the client
    openraveros_initialized = 1;
end

if( createsession && isempty(openraveros_globalsession) )
    if( ~exist('sessionserver','var') )
        sessionserver = 'openrave_session';
    end
    
    if( ~exist('viewer','var') )
        viewer = 'qtcoin';
    end

    
    if( destroyall )
        reqdestroy = openraveros_openrave_session();
        reqdestroy.sessionid = -1;
        resdestroy = rosoct_service_call(sessionserver,reqdestroy);
        if( isempty(resdestroy) )
            warning('failed to destroy sessions');
        end
    end
    req = openraveros_openrave_session();
    req.viewer = viewer; % default viewer
    while(1)
        [localid,res] = rosoct_create_session(sessionserver,req);
        
        if( ~isempty(localid) && ~isempty(res) )
            if( res.sessionid==0 )
                error('bad session id');
            end
            openraveros_globalsession.id = localid;
            openraveros_globalsession.server = sessionserver;
            openraveros_globalsession.uuid = res.sessionid;
            %%display(sprintf('created openraveros session uuid %d',res.sessionid));
            return;
        end

        sleep(0.2); % give some time
    end
end
