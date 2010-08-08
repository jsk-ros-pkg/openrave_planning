% orRobotSetActiveDOfs(robotid, indices, affinedofs, rotationaxis)
%
% robotid - unique id of the robot
% indices - zero based indices of the robot joints to activate
% affinedofs [optional] - is a mask of affine transformations to enable
% 1 - X, adds 1 DOF
% 2 - Y, adds 1 DOF
% 4 - Z, adds 1 DOF
% 8 - RotationAxis, adds 1 DOF, rotationaxis has to be valid
% 16 - full 3D rotation, adds 3 DOF. Because any orientation is a rotation around some axis,
%                        the format of 3D rotation is axis*angle which rotates angle radians around axis
% 32 - quaternion rotation, adds 4 DOF. The quaternion [cos(angle/2) sin(angle/2)*axis]
% rotationaxis [optional] - the 3D rotation axis (if the RotationAxis bit is set in affinedofs)

function success = orRobotSetActiveDOFs(robotid, indices, affinedofs, rotationaxis)

session = openraveros_getglobalsession();
req = openraveros_robot_setactivedofs();
req.bodyid = robotid;
if( exist('affinedofs','var') )
    req.active.affine = affinedofs;
end
if( exist('indices','var') )
    req.active.joints = indices(:);
end
if( exist('rotationaxis','var') )
    req.active.rotationaxis(1:3) = rotationaxis(:);
end

res = rosoct_session_call(session.id,'robot_setactivedofs',req);
success = ~isempty(res);
