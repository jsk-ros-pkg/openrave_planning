%% body = openraveros_getbodyinfo(bodyinfo)
%%
%% parses the BodyInfo.msg into a simpler form for octave consumption

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
function body = openraveros_getbodyinfo(bodyinfo)

body.id = bodyinfo.bodyid;
body.name = bodyinfo.name;
body.type = bodyinfo.type;
body.filename = bodyinfo.filename;

%% extra
body.enabled = bodyinfo.enabled;
body.dof = bodyinfo.dof;
body.T = reshape(bodyinfo.transform.m,[3 4]);

body.jointvalues = bodyinfo.jointvalues;
body.links = zeros(12,length(bodyinfo.links));
for i = 1:length(bodyinfo.links)
    body.links(:,i) = bodyinfo.links{i}.m;
end

body.linknames = bodyinfo.linknames;
body.jointnames = bodyinfo.jointnames;
body.lowerlimit = bodyinfo.lowerlimit;
body.upperlimit = bodyinfo.upperlimit;
