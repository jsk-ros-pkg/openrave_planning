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
    virtual ~ROSSensorPublisher() { Destroy(); }

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
        vector<string> vargs = vector<string>((istream_iterator<string>(ss)), istream_iterator<string>());
        vector<char*> pargs(vargs.size());
        for(size_t i = 0; i < vargs.size(); ++i)
            pargs[i] = &vargs[i][0];
        int argc=vargs.size();
        ros::init(argc,pargs.size() > 0 ? &pargs[0] : NULL,"openrave", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);

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
    inline boost::shared_ptr<ROSSensorPublisher> shared_problem() { return boost::static_pointer_cast<ROSSensorPublisher>(shared_from_this()); }
    inline boost::shared_ptr<ROSSensorPublisher const> shared_problem_const() const { return boost::static_pointer_cast<ROSSensorPublisher const>(shared_from_this()); }

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

        void Publish()
        {
            bool bCheckStamp=true;
            if( _sensordata.size() == 0 ) {
                // a sensor can support multiple types, so iterative through all of them
                for(int itype = 0; itype <= SensorBase::ST_NumberofSensorTypes; ++itype) {
                    SensorBase::SensorType type = (SensorBase::SensorType)itype;
                    if( _sensor->Supports(type) ) {
                        SensorBase::SensorDataPtr pdata = _sensor->CreateSensorData(type);
                        if( !!pdata ) {
                            _sensordata[type] = pdata;
                        }
                    }
                }
            }
            if( _msgs.size() == 0 ) {
                FOREACH(itdata, _sensordata) {
                    switch(itdata->first) {
                    case SensorBase::ST_Camera:
                        _msgs.push_back(make_pair(ros::MessagePtr(new sensor_msgs::Image()), _node->advertise<sensor_msgs::CameraInfo>("camera_info",5)));
                        _msgs.push_back(make_pair(new sensor_msgs::CameraInfo(), _node->advertise<sensor_msgs::Image>("image",5)));
                        break;
                    case SensorBase::ST_Laser:
                        _msgs.push_back(make_pair(ros::MessagePtr(new sensor_msgs::LaserScan()), _node->advertise<sensor_msgs::LaserScan>("scan",5)));
                        _msgs.push_back(make_pair(ros::MessagePtr(new sensor_msgs::PointCloud2()), _node->advertise<sensor_msgs::PointCloud2>("pointcloud",5)));
                        _msgs.push_back(make_pair(ros::MessagePtr(new sensor_msgs::PointCloud()), _node->advertise<sensor_msgs::PointCloud>("pointcloud_old",5)));
                        break;
                    case SensorBase::ST_IMU:
                        _msgs.push_back(make_pair(ros::MessagePtr(new sensor_msgs::Imu()), _node->advertise<sensor_msgs::Imu>("imu",5)));
                        break;
                    case SensorBase::ST_Force6D:
                        _msgs.push_back(make_pair(ros::MessagePtr(new geometry_msgs::WrenchStamped()), _node->advertise<geometry_msgs::WrenchStamped>("force",5)));
                        break;
                    case SensorBase::ST_Odometry:
                        _msgs.push_back(make_pair(ros::MessagePtr(new nav_msgs::Odometry()), _node->advertise<nav_msgs::Odometry>("odometry",5)));
                        break;
                    default:
                        RAVELOG_WARN("unsupported sensor type\n");
                        break;
                    }
                }
                bCheckStamp = false;
            }

            int imsgindex = 0;
            FOREACH(itdata, _sensordata) {
                if( !_sensor->GetSensorData(itdata->second) ) {
                    continue;
                }
                std_msgs::Header header;
                header.stamp = ros::Time(itdata->second->__stamp/1000000,(uint32_t)(1000*(itdata->second->__stamp%1000000)));
                header.frame_id = _frame_id;
                switch(itdata->first) {
                case SensorBase::ST_Camera: {
                    sensor_msgs::ImagePtr imagemsg = boost::dynamic_pointer_cast<sensor_msgs::Image>(_msgs.at(imsgindex).first);
                    if( bCheckStamp && imagemsg->header.stamp == header.stamp ) {
                        imsgindex += 2;
                        break;
                    }
                    boost::shared_ptr<SensorBase::CameraGeomData> pcamerageom = boost::static_pointer_cast<SensorBase::CameraGeomData>(_sensor->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::CameraSensorData> pcameradata = boost::static_pointer_cast<SensorBase::CameraSensorData>(itdata->second);

                    if( _msgs.at(imsgindex+0).second.getNumSubscribers() > 0 ) {
                        sensor_msgs::CameraInfoPtr infomsg = boost::dynamic_pointer_cast<sensor_msgs::CameraInfo>(_msgs.at(imsgindex+1).first);
                        infomsg->header = header;
                        infomsg->width = pcamerageom->width;
                        infomsg->height = pcamerageom->height;
                        infomsg->distortion_model = pcamerageom->KK.distortion_model;
                        infomsg->D.resize(pcamerageom->KK.distortion_coeffs.size());
                        std::copy(pcamerageom->KK.distortion_coeffs.begin(),pcamerageom->KK.distortion_coeffs.end(),infomsg->D.begin());
                        infomsg->K[0] = pcamerageom->KK.fx; infomsg->K[1] = 0; infomsg->K[2] = pcamerageom->KK.cx;
                        infomsg->K[3] = 0; infomsg->K[4] = pcamerageom->KK.fy; infomsg->K[5] = pcamerageom->KK.cy;
                        infomsg->K[6] = 0; infomsg->K[7] = 0; infomsg->K[8] = 1;
                        infomsg->R[0] = 1; infomsg->R[1] = 0; infomsg->R[2] = 0;
                        infomsg->R[3] = 0; infomsg->R[4] = 1; infomsg->R[5] = 0;
                        infomsg->R[6] = 0; infomsg->R[7] = 0; infomsg->R[8] = 1;
                        infomsg->P[0] = pcamerageom->KK.fx; infomsg->P[1] = 0; infomsg->P[2] = pcamerageom->KK.cx; infomsg->P[3] = 0;
                        infomsg->P[4] = 0; infomsg->P[5] = pcamerageom->KK.fy; infomsg->P[6] = pcamerageom->KK.cy; infomsg->P[7] = 0;
                        infomsg->P[8] = 0; infomsg->P[9] = 0; infomsg->P[10] = 1; infomsg->P[11] = 0;
                        _msgs.at(imsgindex).second.publish(*infomsg);
                    }
                    if( _msgs.at(imsgindex+1).second.getNumSubscribers() > 0 ) {
                        imagemsg->header = header;
                        imagemsg->width = pcamerageom->width;
                        imagemsg->height = pcamerageom->height;
                        imagemsg->encoding = "rgb8";
                        imagemsg->step = pcamerageom->width*3;
                        imagemsg->data = pcameradata->vimagedata;
                        _msgs.at(imsgindex+1).second.publish(*imagemsg);
                    }
                    imsgindex += 2;
                    break;
                }
                case SensorBase::ST_Laser: {
                    sensor_msgs::LaserScanPtr laserscanmsg = boost::dynamic_pointer_cast<sensor_msgs::LaserScan>(_msgs.at(imsgindex).first);
                    if( bCheckStamp && laserscanmsg->header.stamp == header.stamp ) {
                        imsgindex += 2;
                        break;
                    }
                    boost::shared_ptr<SensorBase::LaserGeomData> plasergeom = boost::static_pointer_cast<SensorBase::LaserGeomData>(_sensor->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::LaserSensorData> plaserdata = boost::static_pointer_cast<SensorBase::LaserSensorData>(itdata->second);

                    if( _msgs.at(imsgindex+0).second.getNumSubscribers() > 0 ) {
                        laserscanmsg->header = header;
                        laserscanmsg->angle_min = plasergeom->min_angle[0];
                        laserscanmsg->angle_max = plasergeom->max_angle[0];
                        laserscanmsg->angle_increment = plasergeom->resolution[0];
                        laserscanmsg->time_increment = plasergeom->time_increment;
                        laserscanmsg->scan_time = plasergeom->time_scan;
                        laserscanmsg->range_min = plasergeom->min_range;
                        laserscanmsg->range_max = plasergeom->max_range;
                        laserscanmsg->ranges.resize(plaserdata->ranges.size());
                        for(size_t i = 0; i < plaserdata->ranges.size(); ++i) {
                            laserscanmsg->ranges[i] = RaveSqrt(plaserdata->ranges[i].lengthsqr3());
                        }
                        laserscanmsg->intensities.resize(plaserdata->intensity.size());
                        std::copy(plaserdata->intensity.begin(),plaserdata->intensity.end(),laserscanmsg->intensities.begin());
                        _msgs.at(imsgindex).second.publish(laserscanmsg);
                    }

                    if( _msgs.at(imsgindex+1).second.getNumSubscribers() > 0 ) {
                        sensor_msgs::PointCloud2Ptr pointcloud2 = boost::dynamic_pointer_cast<sensor_msgs::PointCloud2>(_msgs.at(imsgindex+1).first);
                        pointcloud2->header = header;
                        pointcloud2->height = 1;
                        pointcloud2->width = plaserdata->ranges.size();
                        uint8_t drealtype = 0;
                        switch(sizeof(dReal)) {
                        case 4: drealtype = sensor_msgs::PointField::FLOAT32; break;
                        case 8: drealtype = sensor_msgs::PointField::FLOAT64; break;
                        default:
                            RAVELOG_WARN("bad type\n");
                            return;
                        }
                        drealtype = sensor_msgs::PointField::FLOAT32;
                        pointcloud2->point_step = 0;
                        pointcloud2->fields.resize(3 + (plaserdata->intensity.size()==plaserdata->ranges.size()));
                        pointcloud2->fields[0].name = "x";
                        pointcloud2->fields[0].offset = pointcloud2->point_step;
                        pointcloud2->fields[0].datatype = drealtype;
                        pointcloud2->fields[0].count = 1;
                        pointcloud2->point_step += sizeof(float);
                        pointcloud2->fields[1].name = "y";
                        pointcloud2->fields[1].offset = pointcloud2->point_step;
                        pointcloud2->fields[1].datatype = drealtype;
                        pointcloud2->fields[1].count = 1;
                        pointcloud2->point_step += sizeof(float);
                        pointcloud2->fields[2].name = "z";
                        pointcloud2->fields[2].offset = pointcloud2->point_step;
                        pointcloud2->fields[2].datatype = drealtype;
                        pointcloud2->fields[2].count = 1;
                        pointcloud2->point_step += sizeof(float);
                        if( plaserdata->intensity.size()==plaserdata->ranges.size() ) {
                            pointcloud2->fields[3].name = "intensity";
                            pointcloud2->fields[3].offset = pointcloud2->point_step;
                            pointcloud2->fields[3].datatype = drealtype;
                            pointcloud2->fields[3].count = 1;
                            pointcloud2->point_step += sizeof(float);
                        }
                        pointcloud2->is_bigendian = false;
                        pointcloud2->row_step = plaserdata->ranges.size()*pointcloud2->point_step;
                        pointcloud2->data.resize(pointcloud2->row_step);
                        for(size_t i = 0; i < plaserdata->ranges.size(); ++i) {
                            float* p = (float*)(&pointcloud2->data.at(i*pointcloud2->point_step));
                            if( i < plaserdata->positions.size() ) {
                                p[0] = plaserdata->ranges[i].x + plaserdata->positions[i].x;
                                p[1] = plaserdata->ranges[i].y + plaserdata->positions[i].y;
                                p[2] = plaserdata->ranges[i].z + plaserdata->positions[i].z;
                            }
                            else if( plaserdata->positions.size() > 0 ) {
                                p[0] = plaserdata->ranges[i].x + plaserdata->positions[0].x;
                                p[1] = plaserdata->ranges[i].y + plaserdata->positions[0].y;
                                p[2] = plaserdata->ranges[i].z + plaserdata->positions[0].z;
                            }
                            else {
                                p[0] = plaserdata->ranges[i].x;
                                p[1] = plaserdata->ranges[i].y;
                                p[2] = plaserdata->ranges[i].z;
                            }
                            if( plaserdata->intensity.size()==plaserdata->ranges.size() ) {
                                p[3] = plaserdata->intensity[i];
                            }
                        }
                        pointcloud2->is_dense = true;
                        _msgs.at(imsgindex+1).second.publish(*pointcloud2);
                    }

                    if( _msgs.at(imsgindex+2).second.getNumSubscribers() > 0 ) {
                        sensor_msgs::PointCloudPtr pointcloud = boost::dynamic_pointer_cast<sensor_msgs::PointCloud>(_msgs.at(imsgindex+2).first);
                        pointcloud->header = header;
                        if( plaserdata->intensity.size()==plaserdata->ranges.size() ) {
                            pointcloud->channels.resize(1);
                            pointcloud->channels[0].name = "intensity";
                            pointcloud->channels[0].values.resize(plaserdata->intensity.size());
                            for(size_t i = 0; i < plaserdata->intensity.size(); ++i) {
                                pointcloud->channels[0].values[i] = plaserdata->intensity[i];
                            }
                        }
                        pointcloud->points.resize(plaserdata->ranges.size());
                        for(size_t i = 0; i < plaserdata->ranges.size(); ++i) {
                            if( i < plaserdata->positions.size() ) {
                                pointcloud->points[i].x = plaserdata->ranges[i].x + plaserdata->positions[i].x;
                                pointcloud->points[i].y = plaserdata->ranges[i].y + plaserdata->positions[i].y;
                                pointcloud->points[i].z = plaserdata->ranges[i].z + plaserdata->positions[i].z;
                            }
                            else if( plaserdata->positions.size() > 0 ) {
                                pointcloud->points[i].x = plaserdata->ranges[i].x + plaserdata->positions[0].x;
                                pointcloud->points[i].y = plaserdata->ranges[i].y + plaserdata->positions[0].y;
                                pointcloud->points[i].z = plaserdata->ranges[i].z + plaserdata->positions[0].z;
                            }
                            else {
                                pointcloud->points[i].x = plaserdata->ranges[i].x;
                                pointcloud->points[i].y = plaserdata->ranges[i].y;
                                pointcloud->points[i].z = plaserdata->ranges[i].z;
                            }
                        }
                        _msgs.at(imsgindex+2).second.publish(*pointcloud);
                    }

                    imsgindex += 3;
                    break;
                }
                case SensorBase::ST_Force6D: {
                    geometry_msgs::WrenchStampedPtr wrenchmsg = boost::dynamic_pointer_cast<geometry_msgs::WrenchStamped>(_msgs.at(imsgindex).first);
                    if( bCheckStamp && wrenchmsg->header.stamp == header.stamp ) {
                        imsgindex += 1;
                        break;
                    }
                    boost::shared_ptr<SensorBase::Force6DGeomData> pforcegeom = boost::static_pointer_cast<SensorBase::Force6DGeomData>(_sensor->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::Force6DSensorData> pforcedata = boost::static_pointer_cast<SensorBase::Force6DSensorData>(itdata->second);
                    wrenchmsg->header = header;
                    wrenchmsg->wrench.force = rosVector3(pforcedata->force);
                    wrenchmsg->wrench.torque = rosVector3(pforcedata->torque);
                    _msgs.at(imsgindex).second.publish(*wrenchmsg);
                    imsgindex += 1;
                    break;
                }
                case SensorBase::ST_IMU: {
                    sensor_msgs::ImuPtr imumsg = boost::dynamic_pointer_cast<sensor_msgs::Imu>(_msgs.at(imsgindex).first);
                    if( bCheckStamp && imumsg->header.stamp == header.stamp ) {
                        imsgindex += 1;
                        break;
                    }
                    boost::shared_ptr<SensorBase::IMUGeomData> pimugeom = boost::static_pointer_cast<SensorBase::IMUGeomData>(_sensor->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::IMUSensorData> pimudata = boost::static_pointer_cast<SensorBase::IMUSensorData>(itdata->second);
                    imumsg->header = header;
                    imumsg->orientation.x = pimudata->rotation.y;
                    imumsg->orientation.y = pimudata->rotation.z;
                    imumsg->orientation.z = pimudata->rotation.w;
                    imumsg->orientation.w = pimudata->rotation.x;
                    std::copy(pimudata->rotation_covariance.begin(),pimudata->rotation_covariance.end(),imumsg->orientation_covariance.begin());
                    imumsg->angular_velocity.x = pimudata->angular_velocity.x;
                    imumsg->angular_velocity.y = pimudata->angular_velocity.y;
                    imumsg->angular_velocity.z = pimudata->angular_velocity.z;
                    std::copy(pimudata->angular_velocity_covariance.begin(), pimudata->angular_velocity_covariance.end(), imumsg->angular_velocity_covariance.begin());
                    imumsg->linear_acceleration.x = pimudata->linear_acceleration.x;
                    imumsg->linear_acceleration.y = pimudata->linear_acceleration.y;
                    imumsg->linear_acceleration.z = pimudata->linear_acceleration.z;
                    std::copy(pimudata->linear_acceleration_covariance.begin(), pimudata->linear_acceleration_covariance.end(), imumsg->linear_acceleration_covariance.begin());
                    _msgs.at(imsgindex).second.publish(*imumsg);
                    imsgindex += 1;
                    break;
                }
                case SensorBase::ST_Odometry: {
                    nav_msgs::OdometryPtr odometrymsg = boost::dynamic_pointer_cast<nav_msgs::Odometry>(_msgs.at(imsgindex).first);
                    if( bCheckStamp && odometrymsg->header.stamp == header.stamp ) {
                        imsgindex += 1;
                        break;
                    }
                    boost::shared_ptr<SensorBase::OdometryGeomData> podometrygeom = boost::static_pointer_cast<SensorBase::OdometryGeomData>(_sensor->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::OdometrySensorData> podometrydata = boost::static_pointer_cast<SensorBase::OdometrySensorData>(itdata->second);
                    odometrymsg->header = header;
                    odometrymsg->child_frame_id = podometrygeom->targetid;
                    odometrymsg->pose.pose = rosPose(podometrydata->pose);
                    std::copy(podometrydata->pose_covariance.begin(),podometrydata->pose_covariance.end(),odometrymsg->pose.covariance.begin());
                    odometrymsg->twist.twist.linear = rosVector3(podometrydata->linear_velocity);
                    odometrymsg->twist.twist.angular = rosVector3(podometrydata->angular_velocity);
                    std::copy(podometrydata->velocity_covariance.begin(),podometrydata->velocity_covariance.end(),odometrymsg->twist.covariance.begin());
                    _msgs.at(imsgindex).second.publish(*odometrymsg);
                    imsgindex += 1;
                    break;
                }
                default:
                    if( _failcount == 0 ) {
                        RAVELOG_WARN(str(boost::format("failed to publish sensor type %d\n")%itdata->first));
                    }
                    _failcount++;
                    break;
                }
            }
        }
    private:
        SensorBasePtr _sensor;
        string _frame_id;
        boost::shared_ptr<ros::NodeHandle> _node;
        std::vector< pair<ros::MessagePtr, ros::Publisher> > _msgs;
        std::map<SensorBase::SensorType, SensorBase::SensorDataPtr > _sensordata;
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
        virtual ~RobotPublisher() {}
    
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
                    catch(...) {//const openrave_exception& ex) {
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
        virtual ~EnvironmentSensorPublisher() {}
    
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
