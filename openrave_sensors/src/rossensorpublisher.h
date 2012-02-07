// Copyright (c) 2010 Rosen Diankov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// \author Rosen Diankov
#ifndef OPENRAVE_SENSORPUBLISHER_H
#define OPENRAVE_SENSORPUBLISHER_H

#include "plugindefs.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

class ROSSensorPublisher : public ProblemInstance
{
public:
    ROSSensorPublisher(EnvironmentBasePtr penv) : ProblemInstance(penv), _bDestroyThread(true) {
        __description = ":Interface Author: Rosen Diankov\n\nPublishes openrave robot sensor messages to the ROS network";
        RegisterCommand("RegisterRobot",boost::bind(&ROSSensorPublisher::RegisterRobot,this,_1,_2),
                        "Registers a robot to publish its sensor data.");
        RegisterCommand("RegisterSensor",boost::bind(&ROSSensorPublisher::RegisterSensor,this,_1,_2),
                        "Registers a sensor directly added to the environment for ROS publishing.");
        RegisterCommand("UnregisterRobot",boost::bind(&ROSSensorPublisher::UnregisterRobot,this,_1,_2),
                        "Unregisters a robot.");
        RegisterCommand("UnregisterSensor",boost::bind(&ROSSensorPublisher::UnregisterSensor,this,_1,_2),
                        "Unregisters a sensor added with RegisterSensor.");
    }
    virtual ~ROSSensorPublisher() {
        Destroy();
    }

    virtual void Destroy()
    {
        _bDestroyThread = true;
        _ros.reset();
        _threadros.join();
        {
            boost::mutex::scoped_lock lock(_mutex);
        }
        ProblemInstance::Destroy();
    }

    virtual void Reset()
    {
        _robots.clear();
        _sensors.clear();
        {
            boost::mutex::scoped_lock lock(_mutex);
        }
        ProblemInstance::Reset();
    }

    virtual int main(const std::string& args)
    {
        stringstream ss(args);
        _bDestroyThread = false;
        int argc=0;
        ros::init(argc,NULL,"openrave", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);

        if( !ros::master::check() ) {
            RAVELOG_WARN("failed to create ros\n");
            return -1;
        }
        _ros.reset(new ros::NodeHandle());
        _threadros = boost::thread(boost::bind(&ROSSensorPublisher::_threadrosfn, this));
        return 0;
    }

    virtual bool SimulationStep(dReal fElapsedTime)
    {
        return false;
    }

protected:
    inline boost::shared_ptr<ROSSensorPublisher> shared_problem() {
        return boost::static_pointer_cast<ROSSensorPublisher>(shared_from_this());
    }
    inline boost::shared_ptr<ROSSensorPublisher const> shared_problem_const() const {
        return boost::static_pointer_cast<ROSSensorPublisher const>(shared_from_this());
    }

    virtual void _threadrosfn()
    {
        while(!_bDestroyThread && ros::ok()) {
            ros::spinOnce();
            usleep(1000); // query every 1ms?
        }
    }

    static geometry_msgs::Pose rosPose(const Transform& t)
    {
        geometry_msgs::Pose p;
        p.position.x = t.trans.x;
        p.position.y = t.trans.y;
        p.position.z = t.trans.z;
        p.orientation.x = t.rot.y;
        p.orientation.y = t.rot.z;
        p.orientation.z = t.rot.w;
        p.orientation.w = t.rot.x;
        return p;
    }

    static geometry_msgs::Vector3 rosVector3(const Vector& v)
    {
        geometry_msgs::Vector3 r;
        r.x = v.x;
        r.y = v.y;
        r.z = v.z;
        return r;
    }

    class SensorPublisher
    {
public:
        SensorPublisher(SensorBasePtr sensor, const string& frame_id, boost::shared_ptr<ros::NodeHandle> node) : _sensor(sensor), _frame_id(frame_id), _node(node), _failcount(0) {
        }

        class DataSensorPublisher
        {
public:
            DataSensorPublisher(boost::shared_ptr<ros::NodeHandle> node, SensorBasePtr sensor, const string& frame_id) : _node(node), _sensor(sensor), _frame_id(frame_id) {
            }
            virtual ~DataSensorPublisher() {
            }
            virtual bool Publish() = 0;

protected:
            boost::shared_ptr<ros::NodeHandle> _node;
            SensorBasePtr _sensor;
            string _frame_id;
        };

        class CameraSensorPublisher : public DataSensorPublisher
        {
public:
            CameraSensorPublisher(boost::shared_ptr<ros::NodeHandle> node, SensorBasePtr sensor, const string& frame_id) : DataSensorPublisher(node, sensor, frame_id) {
                _camerainfopub = _node->advertise<sensor_msgs::CameraInfo>("camera_info",5);
                _imagepub = _node->advertise<sensor_msgs::Image>("image",5);
                _data = boost::dynamic_pointer_cast<SensorBase::CameraSensorData>(_sensor->CreateSensorData(SensorBase::ST_Camera));
                BOOST_ASSERT(!!_data);
            }

            bool Publish()
            {
                if( !_sensor->GetSensorData(_data) ) {
                    return false;
                }
                std_msgs::Header header;
                header.stamp = ros::Time(_data->__stamp/1000000,(uint32_t)(1000*(_data->__stamp%1000000)));
                header.frame_id = _frame_id;
                if( _imagemsg.header.stamp == header.stamp || _camerainfomsg.header.stamp == header.stamp ) {
                    return true;
                }
                boost::shared_ptr<SensorBase::CameraGeomData> pcamerageom = boost::static_pointer_cast<SensorBase::CameraGeomData>(_sensor->GetSensorGeometry());
                if( _camerainfopub.getNumSubscribers() > 0 ) {
                    _camerainfomsg.header = header;
                    _camerainfomsg.width = pcamerageom->width;
                    _camerainfomsg.height = pcamerageom->height;
#if ROS_VERSION >= ROS_VERSION_COMBINED(1,4,0)
                    _camerainfomsg.distortion_model = pcamerageom->KK.distortion_model;
#endif
                    _camerainfomsg.D.resize(pcamerageom->KK.distortion_coeffs.size());
                    std::copy(pcamerageom->KK.distortion_coeffs.begin(),pcamerageom->KK.distortion_coeffs.end(),_camerainfomsg.D.begin());
                    _camerainfomsg.K[0] = pcamerageom->KK.fx; _camerainfomsg.K[1] = 0; _camerainfomsg.K[2] = pcamerageom->KK.cx;
                    _camerainfomsg.K[3] = 0; _camerainfomsg.K[4] = pcamerageom->KK.fy; _camerainfomsg.K[5] = pcamerageom->KK.cy;
                    _camerainfomsg.K[6] = 0; _camerainfomsg.K[7] = 0; _camerainfomsg.K[8] = 1;
                    _camerainfomsg.R[0] = 1; _camerainfomsg.R[1] = 0; _camerainfomsg.R[2] = 0;
                    _camerainfomsg.R[3] = 0; _camerainfomsg.R[4] = 1; _camerainfomsg.R[5] = 0;
                    _camerainfomsg.R[6] = 0; _camerainfomsg.R[7] = 0; _camerainfomsg.R[8] = 1;
                    _camerainfomsg.P[0] = pcamerageom->KK.fx; _camerainfomsg.P[1] = 0; _camerainfomsg.P[2] = pcamerageom->KK.cx; _camerainfomsg.P[3] = 0;
                    _camerainfomsg.P[4] = 0; _camerainfomsg.P[5] = pcamerageom->KK.fy; _camerainfomsg.P[6] = pcamerageom->KK.cy; _camerainfomsg.P[7] = 0;
                    _camerainfomsg.P[8] = 0; _camerainfomsg.P[9] = 0; _camerainfomsg.P[10] = 1; _camerainfomsg.P[11] = 0;
                    _camerainfopub.publish(_camerainfomsg);
                }
                if( _imagepub.getNumSubscribers() > 0 ) {
                    _imagemsg.header = header;
                    _imagemsg.width = pcamerageom->width;
                    _imagemsg.height = pcamerageom->height;
                    _imagemsg.encoding = "rgb8";
                    _imagemsg.step = pcamerageom->width*3;
                    _imagemsg.data = _data->vimagedata;
                    _imagepub.publish(_imagemsg);
                }
                return true;
            }

protected:
            sensor_msgs::Image _imagemsg;
            sensor_msgs::CameraInfo _camerainfomsg;
            ros::Publisher _imagepub, _camerainfopub;
            boost::shared_ptr<SensorBase::CameraSensorData> _data;
        };

        class LaserSensorPublisher : public DataSensorPublisher
        {
public:
            LaserSensorPublisher(boost::shared_ptr<ros::NodeHandle> node, SensorBasePtr sensor, const string& frame_id) : DataSensorPublisher(node, sensor, frame_id) {
                _laserscanpub = _node->advertise<sensor_msgs::Image>("scan",5);
                _pointcloudpub = _node->advertise<sensor_msgs::PointCloud>("pointcloud_old",5);
                _pointcloud2pub = _node->advertise<sensor_msgs::PointCloud2>("pointcloud",5);
                _data = boost::dynamic_pointer_cast<SensorBase::LaserSensorData>(_sensor->CreateSensorData(SensorBase::ST_Laser));
                BOOST_ASSERT(!!_data);
            }

            bool Publish()
            {
                if( !_sensor->GetSensorData(_data) ) {
                    return false;
                }
                std_msgs::Header header;
                header.stamp = ros::Time(_data->__stamp/1000000,(uint32_t)(1000*(_data->__stamp%1000000)));
                header.frame_id = _frame_id;
                if( _laserscanmsg.header.stamp == header.stamp || _pointcloudmsg.header.stamp == header.stamp || _pointcloud2msg.header.stamp == header.stamp ) {
                    return true;
                }
                boost::shared_ptr<SensorBase::LaserGeomData> plasergeom = boost::static_pointer_cast<SensorBase::LaserGeomData>(_sensor->GetSensorGeometry());
                Transform tinv = _data->__trans.inverse();

                if( _laserscanpub.getNumSubscribers() > 0 ) {
                    _laserscanmsg.header = header;
                    _laserscanmsg.angle_min = plasergeom->min_angle[0];
                    _laserscanmsg.angle_max = plasergeom->max_angle[0];
                    _laserscanmsg.angle_increment = plasergeom->resolution[0];
                    _laserscanmsg.time_increment = plasergeom->time_increment;
                    _laserscanmsg.scan_time = plasergeom->time_scan;
                    _laserscanmsg.range_min = plasergeom->min_range;
                    _laserscanmsg.range_max = plasergeom->max_range;
                    _laserscanmsg.ranges.resize(_data->ranges.size());
                    for(size_t i = 0; i < _data->ranges.size(); ++i) {
                        _laserscanmsg.ranges[i] = RaveSqrt(_data->ranges[i].lengthsqr3());
                    }
                    _laserscanmsg.intensities.resize(_data->intensity.size());
                    std::copy(_data->intensity.begin(),_data->intensity.end(),_laserscanmsg.intensities.begin());
                    _laserscanpub.publish(_laserscanmsg);
                }

                if( _pointcloud2pub.getNumSubscribers() > 0 ) {
                    _pointcloud2msg.header = header;
                    _pointcloud2msg.height = 1;
                    _pointcloud2msg.width = _data->ranges.size();
                    uint8_t drealtype = 0;
                    switch(sizeof(dReal)) {
                    case 4: drealtype = sensor_msgs::PointField::FLOAT32; break;
                    case 8: drealtype = sensor_msgs::PointField::FLOAT64; break;
                    default:
                        RAVELOG_WARN("bad type\n");
                        return false;
                    }
                    drealtype = sensor_msgs::PointField::FLOAT32;
                    _pointcloud2msg.point_step = 0;
                    _pointcloud2msg.fields.resize(3 + (_data->intensity.size()==_data->ranges.size()));
                    _pointcloud2msg.fields[0].name = "x";
                    _pointcloud2msg.fields[0].offset = _pointcloud2msg.point_step;
                    _pointcloud2msg.fields[0].datatype = drealtype;
                    _pointcloud2msg.fields[0].count = 1;
                    _pointcloud2msg.point_step += sizeof(float);
                    _pointcloud2msg.fields[1].name = "y";
                    _pointcloud2msg.fields[1].offset = _pointcloud2msg.point_step;
                    _pointcloud2msg.fields[1].datatype = drealtype;
                    _pointcloud2msg.fields[1].count = 1;
                    _pointcloud2msg.point_step += sizeof(float);
                    _pointcloud2msg.fields[2].name = "z";
                    _pointcloud2msg.fields[2].offset = _pointcloud2msg.point_step;
                    _pointcloud2msg.fields[2].datatype = drealtype;
                    _pointcloud2msg.fields[2].count = 1;
                    _pointcloud2msg.point_step += sizeof(float);
                    if( _data->intensity.size()==_data->ranges.size() ) {
                        _pointcloud2msg.fields[3].name = "intensity";
                        _pointcloud2msg.fields[3].offset = _pointcloud2msg.point_step;
                        _pointcloud2msg.fields[3].datatype = drealtype;
                        _pointcloud2msg.fields[3].count = 1;
                        _pointcloud2msg.point_step += sizeof(float);
                    }
                    _pointcloud2msg.is_bigendian = false;
                    _pointcloud2msg.row_step = _data->ranges.size()*_pointcloud2msg.point_step;
                    _pointcloud2msg.data.resize(_pointcloud2msg.row_step);
                    for(size_t i = 0; i < _data->ranges.size(); ++i) {
                        float* p = (float*)(&_pointcloud2msg.data.at(i*_pointcloud2msg.point_step));
                        if( i < _data->positions.size() ) {
                            Vector v = tinv*(_data->ranges[i] + _data->positions[i]);
                            p[0] = (float)v.x;
                            p[1] = (float)v.y;
                            p[2] = (float)v.z;
                        }
                        else if( _data->positions.size() > 0 ) {
                            Vector v = tinv*(_data->ranges[i] + _data->positions[0]);
                            p[0] = (float)v.x;
                            p[1] = (float)v.y;
                            p[2] = (float)v.z;
                        }
                        else {
                            Vector v = tinv*_data->ranges[i];
                            p[0] = (float)v.x;
                            p[1] = (float)v.y;
                            p[2] = (float)v.z;
                        }
                        if( _data->intensity.size()==_data->ranges.size() ) {
                            p[3] = _data->intensity[i];
                        }
                    }
                    _pointcloud2msg.is_dense = true;
                    _pointcloud2pub.publish(_pointcloud2msg);
                }

                if( _pointcloudpub.getNumSubscribers() > 0 ) {
                    _pointcloudmsg.header = header;
                    if( _data->intensity.size()==_data->ranges.size() ) {
                        _pointcloudmsg.channels.resize(1);
                        _pointcloudmsg.channels[0].name = "intensity";
                        _pointcloudmsg.channels[0].values.resize(_data->intensity.size());
                        for(size_t i = 0; i < _data->intensity.size(); ++i) {
                            _pointcloudmsg.channels[0].values[i] = _data->intensity[i];
                        }
                    }
                    _pointcloudmsg.points.resize(_data->ranges.size());
                    for(size_t i = 0; i < _data->ranges.size(); ++i) {
                        if( i < _data->positions.size() ) {
                            Vector v = tinv*(_data->ranges[i] + _data->positions[i]);
                            _pointcloudmsg.points[i].x = v.x;
                            _pointcloudmsg.points[i].y = v.y;
                            _pointcloudmsg.points[i].z = v.z;
                        }
                        else if( _data->positions.size() > 0 ) {
                            Vector v = tinv*(_data->ranges[i] + _data->positions[0]);
                            _pointcloudmsg.points[i].x = v.x;
                            _pointcloudmsg.points[i].y = v.y;
                            _pointcloudmsg.points[i].z = v.z;
                        }
                        else {
                            Vector v = tinv*(_data->ranges[i]);
                            _pointcloudmsg.points[i].x = v.x;
                            _pointcloudmsg.points[i].y = v.y;
                            _pointcloudmsg.points[i].z = v.z;
                        }
                    }
                    _pointcloudpub.publish(_pointcloudmsg);
                }
                return true;
            }

protected:
            sensor_msgs::LaserScan _laserscanmsg;
            sensor_msgs::PointCloud _pointcloudmsg;
            sensor_msgs::PointCloud2 _pointcloud2msg;
            ros::Publisher _laserscanpub, _pointcloudpub, _pointcloud2pub;
            boost::shared_ptr<SensorBase::LaserSensorData> _data;
        };

        class Force6DSensorPublisher : public DataSensorPublisher
        {
public:
            Force6DSensorPublisher(boost::shared_ptr<ros::NodeHandle> node, SensorBasePtr sensor, const string& frame_id) : DataSensorPublisher(node, sensor, frame_id) {
                _wrenchpub = _node->advertise<geometry_msgs::WrenchStamped>("force",5);
                _data = boost::dynamic_pointer_cast<SensorBase::Force6DSensorData>(_sensor->CreateSensorData(SensorBase::ST_Force6D));
                BOOST_ASSERT(!!_data);
            }

            bool Publish()
            {
                if( !_sensor->GetSensorData(_data) ) {
                    return false;
                }
                std_msgs::Header header;
                header.stamp = ros::Time(_data->__stamp/1000000,(uint32_t)(1000*(_data->__stamp%1000000)));
                header.frame_id = _frame_id;
                if( _wrenchmsg.header.stamp == header.stamp ) {
                    return true;
                }
                boost::shared_ptr<SensorBase::Force6DGeomData> pforce6dgeom = boost::static_pointer_cast<SensorBase::Force6DGeomData>(_sensor->GetSensorGeometry());
                if( _wrenchpub.getNumSubscribers() > 0 ) {
                    _wrenchmsg.header = header;
                    _wrenchmsg.wrench.force = rosVector3(_data->force);
                    _wrenchmsg.wrench.torque = rosVector3(_data->torque);
                    _wrenchpub.publish(_wrenchmsg);
                }
                return true;
            }
protected:
            geometry_msgs::WrenchStamped _wrenchmsg;
            ros::Publisher _wrenchpub;
            boost::shared_ptr<SensorBase::Force6DSensorData> _data;
        };

        class IMUSensorPublisher : public DataSensorPublisher
        {
public:
            IMUSensorPublisher(boost::shared_ptr<ros::NodeHandle> node, SensorBasePtr sensor, const string& frame_id) : DataSensorPublisher(node, sensor, frame_id) {
                _imupub = _node->advertise<sensor_msgs::Imu>("imu",5);
                _data = boost::dynamic_pointer_cast<SensorBase::IMUSensorData>(_sensor->CreateSensorData(SensorBase::ST_IMU));
                BOOST_ASSERT(!!_data);
            }

            bool Publish()
            {
                if( !_sensor->GetSensorData(_data) ) {
                    return false;
                }
                std_msgs::Header header;
                header.stamp = ros::Time(_data->__stamp/1000000,(uint32_t)(1000*(_data->__stamp%1000000)));
                header.frame_id = _frame_id;
                if( _imumsg.header.stamp == header.stamp ) {
                    return true;
                }
                boost::shared_ptr<SensorBase::IMUGeomData> pimugeom = boost::static_pointer_cast<SensorBase::IMUGeomData>(_sensor->GetSensorGeometry());
                if( _imupub.getNumSubscribers() > 0 ) {
                    _imumsg.header = header;
                    _imumsg.orientation.x = _data->rotation.y;
                    _imumsg.orientation.y = _data->rotation.z;
                    _imumsg.orientation.z = _data->rotation.w;
                    _imumsg.orientation.w = _data->rotation.x;
                    std::copy(_data->rotation_covariance.begin(),_data->rotation_covariance.end(),_imumsg.orientation_covariance.begin());
                    _imumsg.angular_velocity.x = _data->angular_velocity.x;
                    _imumsg.angular_velocity.y = _data->angular_velocity.y;
                    _imumsg.angular_velocity.z = _data->angular_velocity.z;
                    std::copy(_data->angular_velocity_covariance.begin(), _data->angular_velocity_covariance.end(), _imumsg.angular_velocity_covariance.begin());
                    _imumsg.linear_acceleration.x = _data->linear_acceleration.x;
                    _imumsg.linear_acceleration.y = _data->linear_acceleration.y;
                    _imumsg.linear_acceleration.z = _data->linear_acceleration.z;
                    std::copy(_data->linear_acceleration_covariance.begin(), _data->linear_acceleration_covariance.end(), _imumsg.linear_acceleration_covariance.begin());
                    _imupub.publish(_imumsg);
                }
                return true;
            }

protected:
            sensor_msgs::Imu _imumsg;
            ros::Publisher _imupub;
            boost::shared_ptr<SensorBase::IMUSensorData> _data;
        };

        class OdometrySensorPublisher : public DataSensorPublisher
        {
public:
            OdometrySensorPublisher(boost::shared_ptr<ros::NodeHandle> node, SensorBasePtr sensor, const string& frame_id) : DataSensorPublisher(node, sensor, frame_id) {
                _odometrypub = _node->advertise<nav_msgs::Odometry>("odometry",5);
                _data = boost::dynamic_pointer_cast<SensorBase::OdometrySensorData>(_sensor->CreateSensorData(SensorBase::ST_Odometry));
                BOOST_ASSERT(!!_data);
            }

            bool Publish()
            {
                if( !_sensor->GetSensorData(_data) ) {
                    return false;
                }
                std_msgs::Header header;
                header.stamp = ros::Time(_data->__stamp/1000000,(uint32_t)(1000*(_data->__stamp%1000000)));
                header.frame_id = _frame_id;
                if( _odometrymsg.header.stamp == header.stamp ) {
                    return true;
                }
                boost::shared_ptr<SensorBase::OdometryGeomData> podometrygeom = boost::static_pointer_cast<SensorBase::OdometryGeomData>(_sensor->GetSensorGeometry());
                if( _odometrypub.getNumSubscribers() > 0 ) {
                    _odometrymsg.header = header;
                    _odometrymsg.child_frame_id = podometrygeom->targetid;
                    _odometrymsg.pose.pose = rosPose(_data->pose);
                    std::copy(_data->pose_covariance.begin(),_data->pose_covariance.end(),_odometrymsg.pose.covariance.begin());
                    _odometrymsg.twist.twist.linear = rosVector3(_data->linear_velocity);
                    _odometrymsg.twist.twist.angular = rosVector3(_data->angular_velocity);
                    std::copy(_data->velocity_covariance.begin(),_data->velocity_covariance.end(),_odometrymsg.twist.covariance.begin());
                    _odometrypub.publish(_odometrymsg);
                }
                return true;
            }
protected:
            nav_msgs::Odometry _odometrymsg;
            ros::Publisher _odometrypub;
            boost::shared_ptr<SensorBase::OdometrySensorData> _data;
        };

        void Publish()
        {
            if( !_sensor->Configure(SensorBase::CC_PowerCheck) ) {
                return;
            }

            bool bCheckStamp=true;
            if( _sensordata.size() == 0 ) {
                // a sensor can support multiple types, so iterative through all of them
                for(int itype = 0; itype <= SensorBase::ST_NumberofSensorTypes; ++itype) {
                    SensorBase::SensorType type = (SensorBase::SensorType)itype;
                    if( _sensor->Supports(type) ) {
                        switch(type) {
                        case SensorBase::ST_Camera: _sensordata[type].reset(new CameraSensorPublisher(_node,_sensor, _frame_id)); break;
                        case SensorBase::ST_Laser: _sensordata[type].reset(new LaserSensorPublisher(_node,_sensor, _frame_id)); break;
                        case SensorBase::ST_Force6D: _sensordata[type].reset(new Force6DSensorPublisher(_node,_sensor, _frame_id)); break;
                        case SensorBase::ST_IMU: _sensordata[type].reset(new IMUSensorPublisher(_node,_sensor, _frame_id)); break;
                        case SensorBase::ST_Odometry: _sensordata[type].reset(new OdometrySensorPublisher(_node,_sensor, _frame_id)); break;
                        default:
                            RAVELOG_WARN(str(boost::format("do not support publishing sensor type %d\n")%type));
                        }
                    }
                }
                bCheckStamp = false;
            }

            FOREACH(itdata, _sensordata) {
                if( !itdata->second->Publish() ) {
                    RAVELOG_DEBUG(str(boost::format("failed to publish sensor type %d\n")%itdata->first));
                }
            }
        }
private:
        SensorBasePtr _sensor;
        string _frame_id;
        boost::shared_ptr<ros::NodeHandle> _node;
        std::map<SensorBase::SensorType, boost::shared_ptr<DataSensorPublisher> > _sensordata;
        int _failcount;
    };

    class RobotPublisher
    {
public:
        RobotPublisher(RobotBasePtr robot, const string& rosnamespace, dReal polltime, boost::shared_ptr<ros::NodeHandle> node) : _robot(robot), _rosnamespace(rosnamespace), _node(node) {
            if( _rosnamespace.size() > 0 && _rosnamespace[_rosnamespace.size()-1] != '/' ) {
                _rosnamespace.push_back('/');
            }
            _pubjointstate = _node->advertise<sensor_msgs::JointState>(_rosnamespace+"joint_state",10);
            _timer = ros::WallTimer(_node->createWallTimer(ros::WallDuration(polltime), boost::bind(&RobotPublisher::Publish,this,_1)));
        }
        virtual ~RobotPublisher() {
        }

        void Publish(const ros::WallTimerEvent& event)
        {
            EnvironmentMutex::scoped_lock lock(_robot->GetEnv()->GetMutex());

            {
                // publish the joint state
                vector<dReal> vjointvalues;
                _robot->GetDOFValues(vjointvalues);
                _jointstate.name.resize(_robot->GetDOF());
                _jointstate.position.resize(_robot->GetDOF());
                for(int i = 0; i < _robot->GetDOF(); ++i) {
                    KinBody::JointPtr pjoint = _robot->GetJointFromDOFIndex(i);
                    if( pjoint->GetDOF() > 1 ) {
                        _jointstate.name[i] = str(boost::format("%s%d")%pjoint->GetName()%(i-pjoint->GetDOFIndex()));
                    }
                    else {
                        _jointstate.name[i] = pjoint->GetName();
                    }
                    _jointstate.position[i] = vjointvalues.at(i);
                }

                _robot->GetDOFVelocities(vjointvalues);
                _jointstate.velocity.insert(_jointstate.velocity.end(),vjointvalues.begin(),vjointvalues.end());
                _jointstate.effort.resize(0);
                if( !!_robot->GetController() ) {
                    try {
                        _robot->GetController()->GetTorque(vjointvalues);
                        _jointstate.effort.insert(_jointstate.effort.end(),vjointvalues.begin(),vjointvalues.end());
                    }
                    catch(...) { //const openrave_exception& ex) {
                    }
                }
                _pubjointstate.publish(_jointstate);
            }

            // publish each sensor
            FOREACH(itsensor,_robot->GetAttachedSensors()) {
                if( !(*itsensor)->GetSensor() ) {
                    continue;
                }
                if( _publishers.find(*itsensor) == _publishers.end() ) {
                    _publishers[*itsensor].reset(new SensorPublisher((*itsensor)->GetSensor(),(*itsensor)->GetName(),boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(*_node, _rosnamespace+(*itsensor)->GetName()))));
                }
                _publishers[*itsensor]->Publish();
            }
        }

        RobotBasePtr _robot;
        string _rosnamespace;
        boost::shared_ptr<ros::NodeHandle> _node;
        map<RobotBase::AttachedSensorPtr, boost::shared_ptr<SensorPublisher> > _publishers;
        sensor_msgs::JointState _jointstate;
        ros::Publisher _pubjointstate;
        ros::WallTimer _timer;
    };

    bool RegisterRobot(ostream& sout, istream& sinput)
    {
        string cmd;
        dReal polltime=0.001f;
        RobotBasePtr robot;
        string rosnamespace;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "robot" ) {
                string robotname;
                sinput >> robotname;
                robot = GetEnv()->GetRobot(robotname);
            }
            else if( cmd == "polltime" ) {
                sinput >> polltime;
            }
            else if( cmd == "namespace" ) {
                sinput >> rosnamespace;
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !robot ) {
            return false;
        }
        _robots[robot] = boost::shared_ptr<RobotPublisher>(new RobotPublisher(robot,rosnamespace,polltime,_ros));
        return true;
    }

    bool UnregisterRobot(ostream& sout, istream& sinput)
    {
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "robot" ) {
                string robotname;
                sinput >> robotname;
                RobotBasePtr probot = GetEnv()->GetRobot(robotname);
                if( !!probot )
                    _robots.erase(probot);
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }
        return true;
    }

    class EnvironmentSensorPublisher
    {
public:
        EnvironmentSensorPublisher(SensorBasePtr sensor, const string& rosnamespace, dReal polltime, boost::shared_ptr<ros::NodeHandle> node) : _sensor(sensor), _rosnamespace(rosnamespace), _node(node) {
            if( _rosnamespace.size() > 0 && _rosnamespace[_rosnamespace.size()-1] != '/' ) {
                _rosnamespace.push_back('/');
            }
            _timer = ros::WallTimer(_node->createWallTimer(ros::WallDuration(polltime), boost::bind(&EnvironmentSensorPublisher::Publish,this,_1)));
        }
        virtual ~EnvironmentSensorPublisher() {
        }

        void Publish(const ros::WallTimerEvent& event)
        {
            EnvironmentMutex::scoped_lock lock(_sensor->GetEnv()->GetMutex());
            if( !_publisher ) {
                _publisher.reset(new SensorPublisher(_sensor,_sensor->GetName(),boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(*_node, _rosnamespace+_sensor->GetName()))));
            }
            _publisher->Publish();
        }

        SensorBasePtr _sensor;
        string _rosnamespace;
        boost::shared_ptr<ros::NodeHandle> _node;
        boost::shared_ptr<SensorPublisher> _publisher;
        ros::WallTimer _timer;
    };

    bool RegisterSensor(ostream& sout, istream& sinput)
    {
        string cmd;
        dReal polltime=0.001f;
        SensorBasePtr sensor;
        string rosnamespace;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "sensor" ) {
                string name;
                sinput >> name;
                sensor = GetEnv()->GetSensor(name);
            }
            else if( cmd == "polltime" ) {
                sinput >> polltime;
            }
            else if( cmd == "namespace" ) {
                sinput >> rosnamespace;
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !sensor ) {
            return false;
        }
        _sensors[sensor] = boost::shared_ptr<EnvironmentSensorPublisher>(new EnvironmentSensorPublisher(sensor,rosnamespace,polltime,_ros));
        return true;
    }

    bool UnregisterSensor(ostream& sout, istream& sinput)
    {
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "sensor" ) {
                string name;
                sinput >> name;
                SensorBasePtr psensor = GetEnv()->GetSensor(name);
                if( !!psensor ) {
                    _sensors.erase(psensor);
                }
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }
            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }
        return true;
    }

    boost::shared_ptr<ros::NodeHandle> _ros;
    map<RobotBasePtr,boost::shared_ptr<RobotPublisher> > _robots;
    map<SensorBasePtr,boost::shared_ptr<EnvironmentSensorPublisher> > _sensors;
    boost::thread _threadros;
    boost::mutex _mutex;
    bool _bDestroyThread;
};

#endif
