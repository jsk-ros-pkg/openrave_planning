% aabbs = orBodyGetAABBs(bodyid)
%
% returns the axis-aligned boudning boxes of all the links of the body in world coordinates
% aabbs is a 6xn vector where each column describes the box for all n links.
% The first 3 values in each column describe the position of the aabb, and the next
% 3 values describe the extents (half width/length/height) on each of the axes.

function aabbs = orBodyGetAABBs(bodyid)
session = openraveros_getglobalsession();
req = openraveros_body_getaabbs();
req.bodyid = bodyid;
res = rosoct_session_call(session.id,'body_getaabbs',req);

if(~isempty(res))
    aabbs = zeros(6,length(res.boxes));
    for i = 1:length(res.boxes)
        aabbs(:,i) = [res.boxes{i}.center; res.boxes{i}.extents];
    end
else
    aabbs = [];
end
