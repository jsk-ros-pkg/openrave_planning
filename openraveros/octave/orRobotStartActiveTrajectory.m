% orRobotStartActiveTrajectory(robotid, jointvalues, timestamps, transformations,torques)
%
% Starts/Queues a robot trajectory of the robot where the size of
% each trajectory point is the current active degrees of freedom
% of the robot (others are held constants)
% D is the number of active degrees of freedom.
% N is the number of points of the trajectory
% robotid - unique id of the robot
% jointvalues - DxN matrix of the joint values of each point in the trajrectory.
% timestamps [optional] - the time stamps in seconds of each trajectory point.
% transformations [optional] - 12xN or 7xN matrix. The base link transformations of
%                              each trajectory point.
%                              If the column size is 12, then it is a 3x4 matrix
%                              in column first order
%                              If the column size is 7, then it is a quaterion and a translation.
%                              If active degrees of freedom contains a affine transformation component
%                              it is overwritten with the transformations matrices
% torques - optional feedforward torques for each point
function success = orRobotStartActiveTrajectory(robotid, jointvalues, timestamps, transformations,torques)

session = openraveros_getglobalsession();

%% get some robot info
reqinfo = openraveros_env_getrobots();
reqinfo.bodyid = robotid;
resinfo = rosoct_session_call(session.id,'env_getrobots',reqinfo);
if( isempty(resinfo) )
    success = 0;
    return;
end

req = openraveros_robot_starttrajectory();
req.bodyid = robotid;
req.trajectory.active = resinfo.robots{1}.active;

arr = ones(size(jointvalues,1),1);
numpts = size(jointvalues,2);
req.trajectory.points = cell(numpts,1);
for i = 1:numpts
    pt = openraveros_TrajectoryPoint();
    pt.position = jointvalues(:,i);
    req.trajectory.points{i} = pt;
end

if( exist('timestamps','var') && ~isempty(timestamps) )
    req.options = req.options + req.Traj_UseTimestamps();
    for i = 1:numpts
        req.trajectory.points{i}.timestamp = timestamps(i);
    end
end

if( exist('torques','var') && ~isempty(torques) )
    req.options = req.options + req.Traj_UseTorques();
    for i = 1:numpts
        req.trajectory.points{i}.torque = torques(:,i);
    end
end

if( exist('transformations','var') && ~isempty(transformations) )
    req.options = req.options + req.Traj_UseTransforms();

    if( size(transformations,1) == 7 )
        %% convert from quaternions
        for i = 1:numpts
            R = openraveros_rotfromquat(transformations(1:4,i));
            req.trajectory.points{i}.transform.m(1:12) = [R(:);transformations(5:7,i)];
        end
    elseif( size(transformations,1) == 12 )
        for i = 1:numpts
            req.trajectory.points{i}.transform.m(1:12) = transformations(:,i);
        end
    else
        error('transformations wrong size');
    end
end

res = rosoct_session_call(session.id,'robot_starttrajectory',req);
success = ~isempty(res);
