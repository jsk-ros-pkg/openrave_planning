% data = orRobotSensorGetData(robotid, sensorindex)
%
% Gets the sensor data. The format returned is dependent on the type
% of sensor. Look at the different data SensorData implementations in rave.h.
% Although the data returned is not necessarily one of them.

% data.type - contains the id of the data type (see SensorBase::SensorType)
% For laser data
%  data.laserrange - 3xN array where each column is the direction * distance
%  data.laserpos - 3xN array where each column is the corresponding origin of each range measurement
%  data.laserint - 1xN optional laser intensity array
% For image data
%  data.KK - 3x3 intrinsic matrix
%  data.T - 3x4 camera matrix (to project a point multiply by KK*inv(T))
%  data.P - 3x4 camera projection matrix
%  data.I - the rgb image size(I) = [height width 3]
function data = orRobotSensorGetData(robotid, sensorindex)

session = openraveros_getglobalsession();
req = openraveros_robot_sensorgetdata();
req.bodyid = robotid;
req.sensorindex = sensorindex;
res = rosoct_session_call(session.id,'robot_sensorgetdata',req);
if( isempty(res) )
    data = [];
    return;
end

data.type = res.type;
switch(data.type)
    case 'laser'
        numrange = length(res.laserrange);
        data.laserrange = reshape(res.laserrange,[3 numrange/3]);

        numpos = length(res.laserpos);
        data.laserpos = reshape(res.laserpos,[3 numpos/3]);

        numint = length(res.laserint);
        data.laserint = reshape(res.laserint,[3 numint/3]);
    case 'camera'
        data.KK = reshape(res.caminfo.K,[3 3]);
        data.P = reshape(res.caminfo.P,[3 4]);
        data.T = inv([inv(data.KK)*data.P; 0 0 0 1]);
        data.T = data.T(1:3,:);
        data.I = sensor_msgs_processImage(res.camimage);
    otherwise
        error('unknown type')
end
