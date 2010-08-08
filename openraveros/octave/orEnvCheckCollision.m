% [collision, colbodyid,contacts,hitbodies,mindist] = orEnvCheckCollision(bodyid,excludeid,...)
%
% Check collision of the robot with the environment. collision is 1 if the robot
% is colliding, colbodyid is the id of the object that body collided with
%% bodyid - the uid of the body, if size > 1, bodyidD(2) narrows collision down to specific body link (one-indexed)
%% extra options to specify at end
%%   'distance' - get back distance queries
%%   'tolerance' - set tolerance for collision error
%%   'contacts' - get back the contact points
%%   'selfcollision' - adds a self collision check
function [collision, colbodyid, contacts, hitbodies, mindist] = orEnvCheckCollision(varargin)

session = openraveros_getglobalsession();
req = openraveros_env_checkcollision();

bodyid = varargin{1};

req.bodyid = bodyid(1);
if( length(bodyid)>1) 
    req.linkid = bodyid(2);
else
    req.linkid = -1; % all links of the body
end

if( nargin > 1 )
    req.excludeids = varargin{2};
end

req.options = 0;
index = 3;
while(index <= nargin)
    switch(varargin{index})
        case 'distance'
            req.options = req.options + req.CO_Distance();
        case 'tolerance'
            req.options = req.options + req.CO_UseTolerance();
            req.tolerance = varargin{i+1};
            index = index+1;
        case 'contacts'
            req.options = req.options + req.CO_Contacts();
        case 'selfcollision'
            req.checkselfcollision = 1;
    end
    index = index+1;
end

if( exist('req_contacts','var') && req_contacts )
    req.options = req.options + req.CO_Contacts();
end
if( exist('req_distance','var') && req_distance )
    req.options = req.options + req.CO_Distance();
end

res = rosoct_session_call(session.id,'env_checkcollision',req);

if(~isempty(res))
    collision = res.collision;
    colbodyid = res.collidingbodyid;
    
    if( ~isempty(res.contacts) )
        contacts = zeros(6,length(res.contacts));
        for i = 1:length(res.contacts)
            contacts(:,i) = [res.contacts{i}.position;res.contacts{i}.normal];
        end
    else
        contacts = [];
    end

    hitbodies = [];%res.hitbodies;
    mindist = res.mindist;
else
    collision = [];
    colbodyid = [];
    contacts = [];
    hitbodies = [];
    mindist = [];
end
