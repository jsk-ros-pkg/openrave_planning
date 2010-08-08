%% robot = openraveros_getrobotinfo(robotinfo)
%%
%% parses the RobotInfo.msg into a simpler form for octave consumption

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
function robot = openraveros_getrobotinfo(robotinfo)

robot = openraveros_getbodyinfo(robotinfo.bodyinfo);

robot.manips = cell(length(robotinfo.manips),1);
for i = 1:length(robot.manips)
    mi = robotinfo.manips{i};
    robot.manips{i} = struct('baselink',mi.baselink+1,...
                             'eelink',mi.eelink+1,...
                             'Tgrasp',reshape(mi.tgrasp.m,[3 4]),...
                             'handjoints',mi.handjoints,...
                             'armjoints',mi.armjoints,...
                             'iksolvername',mi.iksolvername);
end

robot.sensors = cell(length(robotinfo.sensors),1);
for i = 1:length(robot.sensors)
    si = robotinfo.sensors{i};
    robot.sensors{i} = struct('name',si.name,...
                              'attachedlink',si.attachedlink+1,...
                              'Trelative',reshape(si.trelative.m,[3 4]),...
                              'Tglobal',reshape(si.tglobal.m,[3 4]),...
                              'type',si.type);
end

robot.activemanip = robotinfo.activemanip+1;
robot.activedof = robotinfo.activedof;
robot.affinedof = robotinfo.active.affine;
robot.activejoints = robotinfo.active.joints;
robot.rotationaxis = robotinfo.active.rotationaxis;

robot.activelowerlimit = robotinfo.activelowerlimit;
robot.activeupperlimit = robotinfo.activeupperlimit;
