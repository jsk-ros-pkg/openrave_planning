% robotinfo = orEnvGetRobots(robotid,options)
%
%% Input:
%% robotid - uid of robot, if 0 gets all robots
%% options - set of options that controls what is sent back (BodyInfo.Req_X and RobotInfo.Req_X).
%%           If none specified gets everything.
%% Output:
%% if robotid is 0 or wasn't specified, robotinfo is a cell array of robots.
%% Otherwise robotinfo holds the info of just one robot
%% Every entry contains a struct with the following parameters
%% id - robotid
%% filename - filename used to initialize the body with
%% name - human robot name
%% type - type of robot

%% Software License Agreement (BSD License)
%% Copyright (c) 2006-2009, Rosen Diankov
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
function robotinfo = orEnvGetRobots(robotid, options)
session = openraveros_getglobalsession();
req = openraveros_env_getrobots();
if( exist('robotid','var') && ~isempty(robotid) )
    req.bodyid = robotid;
end
if( exist('options','var') )
    req.options = options;
else
    req.options = 65535; % get everything
end
res = rosoct_session_call(session.id,'env_getrobots',req);

if(~isempty(res) && ~isempty(res.robots) )
    if( exist('robotid','var') && robotid ~= 0 )
        robotinfo = openraveros_getrobotinfo(res.robots{1});
    else
        robotinfo = cell(length(res.robots),1);
        for i = 1:length(res.robots)
            robotinfo{i} = openraveros_getrobotinfo(res.robots{i});
        end
    end
else
    robotinfo = [];
end
