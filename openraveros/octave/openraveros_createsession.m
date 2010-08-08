%% session = openraveros_createsession(sessionserver, cloneuuid, cloneoptions)
%
%% creates a session and returns its id.
%% clonesession (optional) - if passed in, will clone the new session with this session
%% the environment id that uniquely determines a session is passed in the uuid
%% Output:
%% session.id - local identifier used to make session calls or session terminations
%% session.uuid - unique identifier for the session server across the entire ROS network
%% session.server - name of the session server

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
function session = openraveros_createsession(sessionserver, cloneuuid, cloneoptions)

if( ~exist('sessionserver','var') )
    sessionserver = 'openrave_session';
end
openraveros_startup(sessionserver,0);

req = openraveros_openrave_session();
if( exist('cloneuuid','var') )
    req.clone_sessionid = cloneuuid;
    if( exist('cloneoptions','var') )
        req.clone_options = cloneoptions;
    end
end

session = [];
while(1)
    [localid,res] = rosoct_create_session(sessionserver,req);
    
    if( ~isempty(localid) && ~isempty(res) )
        if( res.sessionid==0 )
            error('bad session id');
        end
        session.id = localid;
        session.server = sessionserver;
        session.uuid = res.sessionid;
        return;
    end

    sleep(0.2); % give some time
end
