% [collision, colinfo, hitbodies] = orEnvRayCollision(rays,[bodyid],...)
%
% performs ray collision checks and returns the position and normals
% where all the rays collide
%% Input:
%% bodyid (optional) - if non zero, will only collide with that ray
%% rays - a 6xN matrix where the first 3
%%        rows are the ray position and last 3 are the ray direction
%% options - 
%%     'nocontacts' - don't send contacts (by default rays send)
%%     'bodies' - get back the bodies every ray hit
%% Output:
%% collision - N dim vector that is 1 for colliding rays and 0
%%      for non-colliding rays colinfo is a 6xN vector that describes 
%%      where the ray hit and the normal to the surface of the hit point
%%      where the first 3 columns are position and last 3 are normals
%%      if bodyid is specified, only checks collisions with that body

function [collision, colinfo, hitbodies] = orEnvRayCollision(varargin)

session = openraveros_getglobalsession();
req = openraveros_env_raycollision();

rays = varargin{1};
numrays = size(rays,2);
req.rays = cell(numrays,1);
for i = 1:numrays
    req.rays{i} = openraveros_Ray();
    req.rays{i}.position(1:3) = rays(1:3,i);
    req.rays{i}.direction(1:3) = rays(4:6,i);
end

if( nargin > 1 )
    req.bodyid = varargin{2};
end

req.request_contacts = 1;
index = 3;
while(index <= nargin)
    switch(varargin{index})
        case 'nocontacts'
            req.request_contacts = 0;
        case 'bodies'
            req.request_bodies = 1;
    end
    index = index+1;
end

res = rosoct_session_call(session.id,'env_raycollision',req);

if(~isempty(res))
    collision = res.collision;

    if( req.request_contacts )
        if( length(res.contacts) ~= numrays )
            error('not equal');
        end
        colinfo = zeros(6,numrays);
        for i = 1:numrays
            colinfo(:,i) = [res.contacts{i}.position;res.contacts{i}.normal];
        end
    else
        colinfo = [];
    end

    hitbodies = res.hitbodies;
else
    collision = [];
    colinfo = [];
    hitbodies = [];
end
