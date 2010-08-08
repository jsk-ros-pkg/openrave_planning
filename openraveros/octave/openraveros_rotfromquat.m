%% R = openraveros_rotfromquat(quat)
%%
%% quat - the format is [cos(angle/2) axis*sin(angle/2)]
function R = openraveros_rotfromquat(quat)

R = zeros(3,3);
qq1 = 2*quat(2)*quat(2);
qq2 = 2*quat(3)*quat(3);
qq3 = 2*quat(4)*quat(4);
R(1,1) = 1 - qq2 - qq3;
R(1,2) = 2*(quat(2)*quat(3) - quat(1)*quat(4));
R(1,3) = 2*(quat(2)*quat(4) + quat(1)*quat(3));
R(2,1) = 2*(quat(2)*quat(3) + quat(1)*quat(4));
R(2,2) = 1 - qq1 - qq3;
R(2,3) = 2*(quat(3)*quat(4) - quat(1)*quat(2));
R(3,1) = 2*(quat(2)*quat(4) - quat(1)*quat(3));
R(3,2) = 2*(quat(3)*quat(4) + quat(1)*quat(2));
R(3,3) = 1 - qq1 - qq2;
