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
        __description = ":Interface Author: Rosen Diankov\nPublishes openrave robot sensor messages to the ROS network";
        RegisterCommand("RegisterRobot",boost::bind(&ROSSensorPublisher::RegisterRobot,this,_1,_2),
                        "Registers a robot to publish its sensor data.");
        RegisterCommand("UnregisterRobot",boost::bind(&ROSSensorPublisher::UnregisterRobot,this,_1,_2),
                        "Unregisters a robot.");
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
    SensorPublisher(RobotBasePtr robot, const string& rosnamespace, dReal polltime, boost::shared_ptr<ros::NodeHandle> node) : _robot(robot), _rosnamespace(rosnamespace), _node(node) {
            if( _rosnamespace.size() > 0 && _rosnamespace[_rosnamespace.size()-1] != '/' ) {
                _rosnamespace.push_back('/');
            }
            _pubjointstate = _node->advertise<sensor_msgs::JointState>(_rosnamespace+"joint_state",10);
            _timer = ros::WallTimer(_node->createWallTimer(ros::WallDuration(polltime), boost::bind(&SensorPublisher::PublishRobotSensors,this,_1)));
        }
        virtual ~SensorPublisher() {}
    
        void PublishRobotSensors(const ros::WallTimerEvent& event)
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
                _jointstate.velocity.resize(0);
                _jointstate.velocity.insert(_jointstate.velocity.end(),vjointvalues.begin(),vjointvalues.end());
                _jointstate.effort.resize(0);
                if( !!_robot->GetController() ) {
                    vjointvalues.resize(0);
                    _robot->GetController()->GetTorque(vjointvalues);
                    _jointstate.effort.insert(_jointstate.effort.end(),vjointvalues.begin(),vjointvalues.end());
                }
                _pubjointstate.publish(_jointstate);
            }

            // publish each sensor
            FOREACH(itsensor,_robot->GetAttachedSensors()) {
                if( !(*itsensor)->GetSensor() ) {
                    continue;
                }
                SensorBase::SensorDataPtr pdata = (*itsensor)->GetData();
                if( !pdata ) {
                    continue;
                }
                ros::Time stamp(pdata->__stamp/1000000,(uint32_t)(1000*(pdata->__stamp%1000000)));
                vector< pair<ros::MessagePtr, ros::Publisher> > msg;
                bool bCheckStamp=true;
                if( _publishers.find(*itsensor) == _publishers.end() ) {
                    switch(pdata->GetType()) {
                    case SensorBase::ST_Camera:
                        msg.push_back(make_pair(ros::MessagePtr(new sensor_msgs::Image()), _node->advertise<sensor_msgs::CameraInfo>(_rosnamespace+(*itsensor)->GetName()+"/camera_info",5)));
                        msg.push_back(make_pair(new sensor_msgs::CameraInfo(), _node->advertise<sensor_msgs::Image>(_rosnamespace+(*itsensor)->GetName()+"/image",5)));
                        break;
                    case SensorBase::ST_Laser:
                        msg.push_back(make_pair(ros::MessagePtr(new sensor_msgs::LaserScan()), _node->advertise<sensor_msgs::LaserScan>(_rosnamespace+(*itsensor)->GetName()+"/scan",5)));
                        msg.push_back(make_pair(ros::MessagePtr(new sensor_msgs::PointCloud2()), _node->advertise<sensor_msgs::PointCloud2>(_rosnamespace+(*itsensor)->GetName()+"/pointcloud",5)));
                        break;
                    case SensorBase::ST_IMU:
                        msg.push_back(make_pair(ros::MessagePtr(new sensor_msgs::Imu()), _node->advertise<sensor_msgs::Imu>(_rosnamespace+(*itsensor)->GetName()+"/imu",5)));
                        break;
                    case SensorBase::ST_Force6D:
                        msg.push_back(make_pair(ros::MessagePtr(new geometry_msgs::WrenchStamped()), _node->advertise<geometry_msgs::WrenchStamped>(_rosnamespace+(*itsensor)->GetName()+"/force",5)));
                        break;
                    case SensorBase::ST_Odometry:
                        msg.push_back(make_pair(ros::MessagePtr(new nav_msgs::Odometry()), _node->advertise<nav_msgs::Odometry>(_rosnamespace+(*itsensor)->GetName()+"/odometry",5)));
                        break;
                    default:
                        RAVELOG_WARN("unsupported sensor type\n");
                        break;
                    }
                    _publishers[*itsensor] = msg;
                    bCheckStamp = false;
                }
                else {
                    msg = _publishers[*itsensor];
                }

                roslib::Header header;
                header.stamp = stamp;
                header.frame_id = (*itsensor)->GetName();

                switch(pdata->GetType()) {
                case SensorBase::ST_Camera: {
                    sensor_msgs::ImagePtr imagemsg = boost::dynamic_pointer_cast<sensor_msgs::Image>(msg.at(0).first);
                    if( bCheckStamp && imagemsg->header.stamp == stamp ) {
                        break;
                    }
                    boost::shared_ptr<SensorBase::CameraGeomData> pcamerageom = boost::static_pointer_cast<SensorBase::CameraGeomData>((*itsensor)->GetSensor()->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::CameraSensorData> pcameradata = boost::static_pointer_cast<SensorBase::CameraSensorData>(pdata);
                    sensor_msgs::CameraInfoPtr infomsg = boost::dynamic_pointer_cast<sensor_msgs::CameraInfo>(msg.at(1).first);
                    infomsg->header = header;
                    infomsg->width = pcamerageom->width;
                    infomsg->height = pcamerageom->height;
                    infomsg->K[0] = pcamerageom->KK.fx; infomsg->K[1] = 0; infomsg->K[2] = pcamerageom->KK.cx;
                    infomsg->K[3] = 0; infomsg->K[4] = pcamerageom->KK.fy; infomsg->K[5] = pcamerageom->KK.cy;
                    infomsg->K[6] = 0; infomsg->K[7] = 0; infomsg->K[8] = 1;
                    infomsg->R[0] = 1; infomsg->R[1] = 0; infomsg->R[2] = 0;
                    infomsg->R[3] = 0; infomsg->R[4] = 1; infomsg->R[5] = 0;
                    infomsg->R[6] = 0; infomsg->R[7] = 0; infomsg->R[8] = 1;
                    infomsg->P[0] = pcamerageom->KK.fx; infomsg->P[1] = 0; infomsg->P[2] = pcamerageom->KK.cx; infomsg->P[3] = 0;
                    infomsg->P[4] = 0; infomsg->P[5] = pcamerageom->KK.fy; infomsg->P[6] = pcamerageom->KK.cy; infomsg->P[7] = 0;
                    infomsg->P[8] = 0; infomsg->P[9] = 0; infomsg->P[10] = 1; infomsg->P[11] = 0;
                    msg.at(0).second.publish(*infomsg);
                    imagemsg->header = header;
                    imagemsg->width = pcamerageom->width;
                    imagemsg->height = pcamerageom->height;
                    imagemsg->encoding = "rgb8";
                    imagemsg->step = pcamerageom->width*3;
                    imagemsg->data = pcameradata->vimagedata;
                    msg.at(1).second.publish(*imagemsg);
                    break;
                }
                case SensorBase::ST_Laser: {
                    sensor_msgs::LaserScanPtr laserscanmsg = boost::dynamic_pointer_cast<sensor_msgs::LaserScan>(msg.at(0).first);
                    if( bCheckStamp && laserscanmsg->header.stamp == stamp ) {
                        break;
                    }
                    boost::shared_ptr<SensorBase::LaserGeomData> plasergeom = boost::static_pointer_cast<SensorBase::LaserGeomData>((*itsensor)->GetSensor()->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::LaserSensorData> plaserdata = boost::static_pointer_cast<SensorBase::LaserSensorData>(pdata);
                    laserscanmsg->header = header;
                    laserscanmsg->angle_min = plasergeom->min_angle[0];
                    laserscanmsg->angle_max = plasergeom->max_angle[0];
                    laserscanmsg->angle_increment = plasergeom->resolution[0];
                    laserscanmsg->time_increment = plasergeom->time_increment;
                    laserscanmsg->scan_time = plasergeom->time_scan;
                    laserscanmsg->range_min = plasergeom->min_range;
                    laserscanmsg->range_max = plasergeom->max_range;
                    // data
                    laserscanmsg->ranges.resize(plaserdata->ranges.size());
                    for(size_t i = 0; i < plaserdata->ranges.size(); ++i) {
                        laserscanmsg->ranges[i] = RaveSqrt(plaserdata->ranges[i].lengthsqr3());
                    }
                    laserscanmsg->intensities.resize(plaserdata->intensity.size());
                    std::copy(plaserdata->intensity.begin(),plaserdata->intensity.end(),laserscanmsg->intensities.begin());
                    msg.at(0).second.publish(laserscanmsg);

                    sensor_msgs::PointCloud2Ptr pointcloud2 = boost::dynamic_pointer_cast<sensor_msgs::PointCloud2>(msg.at(1).first);
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

                    pointcloud2->point_step = 0;
                    pointcloud2->fields.resize(1 + (plaserdata->intensity.size()==plaserdata->ranges.size()));
                    pointcloud2->fields[0].name = "position";
                    pointcloud2->fields[0].offset = pointcloud2->point_step;
                    pointcloud2->fields[0].datatype = drealtype;
                    pointcloud2->fields[0].count = 3;
                    pointcloud2->point_step += 3*sizeof(dReal);
                    if( plaserdata->intensity.size()==plaserdata->ranges.size() ) {
                        pointcloud2->fields[1].name = "intensity";
                        pointcloud2->fields[1].offset = pointcloud2->point_step;
                        pointcloud2->fields[1].datatype = drealtype;
                        pointcloud2->fields[1].count = 1;
                        pointcloud2->point_step += sizeof(dReal);
                    }
                    pointcloud2->is_bigendian = false;
                    pointcloud2->row_step = plaserdata->ranges.size()*pointcloud2->point_step;
                    pointcloud2->data.resize(pointcloud2->row_step);
                    for(size_t i = 0; i < plaserdata->ranges.size(); ++i) {
                        dReal* p = (dReal*)(&pointcloud2->data.at(i*pointcloud2->point_step));
                        p[0] = plaserdata->ranges[i].x;
                        p[1] = plaserdata->ranges[i].y;
                        p[2] = plaserdata->ranges[i].z;
                        p[3] = plaserdata->intensity[i];
                    }
                    pointcloud2->is_dense = true;
                    msg.at(1).second.publish(*pointcloud2);
                    break;
                }
                case SensorBase::ST_Force6D: {
                    geometry_msgs::WrenchStampedPtr wrenchmsg = boost::dynamic_pointer_cast<geometry_msgs::WrenchStamped>(msg.at(0).first);
                    if( bCheckStamp && wrenchmsg->header.stamp == stamp ) {
                        break;
                    }
                    boost::shared_ptr<SensorBase::Force6DGeomData> pforcegeom = boost::static_pointer_cast<SensorBase::Force6DGeomData>((*itsensor)->GetSensor()->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::Force6DSensorData> pforcedata = boost::static_pointer_cast<SensorBase::Force6DSensorData>(pdata);
                    wrenchmsg->header = header;
                    wrenchmsg->wrench.force = rosVector3(pforcedata->force);
                    wrenchmsg->wrench.torque = rosVector3(pforcedata->torque);
                    msg.at(0).second.publish(*wrenchmsg);
                    break;
                }
                case SensorBase::ST_IMU: {
                    sensor_msgs::ImuPtr imumsg = boost::dynamic_pointer_cast<sensor_msgs::Imu>(msg.at(0).first);
                    if( bCheckStamp && imumsg->header.stamp == stamp ) {
                        break;
                    }
                    boost::shared_ptr<SensorBase::IMUGeomData> pimugeom = boost::static_pointer_cast<SensorBase::IMUGeomData>((*itsensor)->GetSensor()->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::IMUSensorData> pimudata = boost::static_pointer_cast<SensorBase::IMUSensorData>(pdata);
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
                    msg.at(0).second.publish(*imumsg);
                    break;
                }
                case SensorBase::ST_Odometry: {
                    nav_msgs::OdometryPtr odometrymsg = boost::dynamic_pointer_cast<nav_msgs::Odometry>(msg.at(0).first);
                    if( bCheckStamp && odometrymsg->header.stamp == stamp ) {
                        break;
                    }
                    boost::shared_ptr<SensorBase::OdometryGeomData> podometrygeom = boost::static_pointer_cast<SensorBase::OdometryGeomData>((*itsensor)->GetSensor()->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::OdometrySensorData> podometrydata = boost::static_pointer_cast<SensorBase::OdometrySensorData>(pdata);
                    odometrymsg->header = header;
                    odometrymsg->child_frame_id = pgeom->targetid;
                    odometrymsg->pose.pose = rosPose(podometrydata->pose);
                    std::copy(podometrydata->pose_covariance.begin(),podometrydata->pose_covariance.end(),odometrymsg->pose.covariance.begin());
                    odometrymsg->twist.twist.linear = rosVector3(podometrydata->linear_velocity);
                    odometrymsg->twist.twist.angular = rosVector3(podometrydata->angular_velocity);
                    std::copy(podometrydata->velocity_covariance.begin(),podometrydata->velocity_covariance.end(),odometrymsg->twist.covariance.begin());
                    msg.at(0).second.publish(*odometrymsg);
                    break;
                }
                default:
                    if( _setUnpublishedSensors.find((*itsensor)->GetName()) == _setUnpublishedSensors.end() ) {
                        RAVELOG_WARN(str(boost::format("failed to public sensor type %d\n")%pdata->GetType()));
                        _setUnpublishedSensors.insert((*itsensor)->GetName());
                        break;
                    }
                }
            }
        }
    
        RobotBasePtr _robot;
        string _rosnamespace;
        boost::shared_ptr<ros::NodeHandle> _node;
        map<RobotBase::AttachedSensorPtr, vector< pair<ros::MessagePtr, ros::Publisher> > > _publishers;
        std::set<std::string> _setUnpublishedSensors;
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
        
        if( !robot )
            return false;
        
        _robots[robot] = boost::shared_ptr<SensorPublisher>(new SensorPublisher(robot,rosnamespace,polltime,_ros));
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

    
    boost::shared_ptr<ros::NodeHandle> _ros;
    map<RobotBasePtr,boost::shared_ptr<SensorPublisher> > _robots;
    boost::thread _threadros;
    boost::mutex _mutex;
    bool _bDestroyThread;
};

#endif
