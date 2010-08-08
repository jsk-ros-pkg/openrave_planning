% success = orEnvSetOptions(options)
%
% A string of various environment options. Example usage:
% orEnvSetOptions('debug 4');
%
% Current options:
% - simulation [start/stop] [time_step] - toggles the internal simulation loop, ie all the calls to SimulationStep. If time_step is specified, will set the simulation time step for all objects. Note that this is not tied to real time at all, how fast the simulation goes in reality depends on complexity of the scene and the physics engine being used.
% - physics engine_name - switches the physics engine to another one with id 'engine_name'
% - collision checker_name - switches the collision checker to a new one with id 'checker_name'
% - viewer viewer_name - attaches a new viewer to the session
% - wdims [width height] - resets the size of the current viewer
% - gravity [x y z] - changes to gravity vector
% - publishanytime [1/0] - switch between publishing the body transformations
%           to the GUI anytime or only between stepsimulation and server  messsages.
%           When publishing anytime, the GUI will reflect the body movements after every
%           move. This is useful when visualizing internal C++ states. When off, the GUI
%           will only reflect the state of robots after all calls to stepsimulation and
%           server send messages have been done. The default is off.
% - debug [debuglevel] - toggles debugging messages by RAVELOG_X. Can be one of
%                        fatal, error, warn, info, debug, verbose
% - quit - closes the openrave instance

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
function success = orEnvSetOptions(options)

session = openraveros_getglobalsession();
req = openraveros_env_set();

[cmd, rem] = strtok(options, ' ');
cmd = strtrim(cmd);
switch(cmd)
    case 'wdims'
        req.setmask = req.Set_ViewerDims();
        dims = sscanf(rem,'%d',2);
        req.viewerwidth = dims(1);
        req.viewerheight = dims(2);
    case 'simulation'
        req.setmask = req.Set_Simulation();
        [cmd,rem] = strtok(rem);
        cmd = strtrim(cmd);
        
        if( strcmp(cmd,'start') )
            req.sim_action = req.SimAction_Start();
        elseif( strcmp(cmd,'stop') )
            req.sim_action = req.SimAction_Stop();
        elseif( strcmp(cmd,'timestep') )
            req.sim_action = req.SimAction_Timestep();
        end

        n = str2num(rem);
        if( ~isempty(n) )
            req.sim_timestep = n;
        end
        
    case 'physics'
        req.setmask = req.Set_PhysicsEngine();
        req.physicsengine = strtrim(rem);
    case 'collision'
        req.setmask = req.Set_CollisionChecker();
        req.collisionchecker = strtrim(rem);
    case 'viewer'
        req.setmask = req.Set_Viewer();
        req.viewer = strtrim(rem);
    case 'gravity'
        req.setmask = req.Set_Gravity();
        req.gravity(1:3) = str2num(rem);
    case 'debug'
        req.setmask = req.Set_DebugLevel();
        req.debuglevel = strtrim(rem);
    otherwise
        display('unknown command');
end

res = rosoct_session_call(session.id,'env_set',req);
success = ~isempty(res);
