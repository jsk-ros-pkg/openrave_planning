% bodyinfo = orEnvGetBodies(bodyid,options)
%
% Input:
% bodyid (optional) - specific body to get info for, if not specified returns all bodies
% options - mask of BodyInfo.Req_X options (if not specified gets all information)
% Output:
%% if bodyid is 0 or wasn't specified, bodyinfo is a cell array of robots.
%% Otherwise bodyinfo holds the info of just one body
%% every entry contains a struct with the following parameters
% id - bodyid
% filename - filename used to initialize the body with
% name - human robot name
% type - xml type of body

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
function bodyinfo = orEnvGetBodies(bodyid,options)
session = openraveros_getglobalsession();
req = openraveros_env_getbodies();
if( exist('bodyid','var') )
    req.bodyid = bodyid;
end
if( exist('options','var') )
    req.options = options;
else
    req.options = 65535; % get everything
end
res = rosoct_session_call(session.id,'env_getbodies',req);

if(~isempty(res) && ~isempty(res.bodies) )
    if( exist('bodyid','var') && bodyid ~= 0 )
        bodyinfo = openraveros_getbodyinfo(res.bodies{1});
    else
        bodyinfo = cell(length(res.bodies),1);
        for i = 1:length(res.bodies)
            bodyinfo{i} = openraveros_getbodyinfo(res.bodies{i});
        end
    end
else
    bodyinfo = [];
end
