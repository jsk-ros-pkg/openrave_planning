function R = openraveros_rotfromaxisangle(axis)
len = norm(axis);
if( len < 1e-10 )
    R = eye(3);
else
    R = openraveros_rotfromquat([cos(len/2) sin(len/2)*reshape(axis/len,[1 3])]);
end
