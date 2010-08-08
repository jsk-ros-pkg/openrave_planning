% aabb = orBodyGetAABB(bodyid)
%
% returns an axis-aligned boudning box of the body in world coordinates
% aabb is a 3x2 vector where the first column is the position of the box
% and the second is the extents

function aabb = orBodyGetAABB(bodyid)
session = openraveros_getglobalsession();
req = openraveros_body_getaabb();
req.bodyid = bodyid;
res = rosoct_session_call(session.id,'body_getaabb',req);

if(~isempty(res))
    aabb = [res.box.center res.box.extents];
else
    aabb = [];
end

