% manipulators = orRobotGetManipulators(robotid)
%
% manipulators is a cell array describes the manipulators of the robot
% Each cell is a struct with fields
%   baselink - zero-based index of base link manipulator is attached to
%   eelink - zero-based index of link defining the end-effector
%   Tgrasp - 3x4 matrix of the grasp frame relative to the end effector link,
%            Tglobalgrasp = Tendeffector*Tgrasp
%   joints - 1xK zero-based joint indices of the hand attached to the end effector
%   armjoints - 1xN zero-based manipulator joint indices that have an
%               effect on the end effector
%   iksolvername - name of ik solver to use
function manipulators = orRobotGetManipulators(robotid)
robot = orEnvGetRobots(robotid);
if( isempty(robot) )
    manipulators = [];
else
    manipulators = robot.manips;
end
