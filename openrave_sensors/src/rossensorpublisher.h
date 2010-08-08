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
#include <sensor_msgs/JointState.h>

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
                pair<ros::MessagePtr, vector<ros::Publisher> > msg;
                bool bCheckStamp=true;
                if( _publishers.find(*itsensor) == _publishers.end() ) {
                    switch(pdata->GetType()) {
                    case SensorBase::ST_Camera:
                        msg.first.reset(new sensor_msgs::Image());
                        msg.second.push_back(_node->advertise<sensor_msgs::CameraInfo>(_rosnamespace+(*itsensor)->GetName()+"/camera_info",5));
                        msg.second.push_back(_node->advertise<sensor_msgs::Image>(_rosnamespace+(*itsensor)->GetName()+"/image",5));
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

                switch(pdata->GetType()) {
                case SensorBase::ST_Camera: {
                    sensor_msgs::ImagePtr imagemsg = boost::dynamic_pointer_cast<sensor_msgs::Image>(msg.first);
                    if( bCheckStamp && imagemsg->header.stamp == stamp ) {
                        break;
                    }

                    boost::shared_ptr<SensorBase::CameraGeomData> pcamerageom = boost::static_pointer_cast<SensorBase::CameraGeomData>((*itsensor)->GetSensor()->GetSensorGeometry());
                    boost::shared_ptr<SensorBase::CameraSensorData> pcameradata = boost::static_pointer_cast<SensorBase::CameraSensorData>(pdata);

                    sensor_msgs::CameraInfo cinfo;
                    cinfo.header.stamp = stamp;
                    cinfo.header.frame_id = (*itsensor)->GetName();
                    cinfo.width = pcamerageom->width;
                    cinfo.height = pcamerageom->height;
                    cinfo.K[0] = pcamerageom->KK.fx; cinfo.K[1] = 0; cinfo.K[2] = pcamerageom->KK.cx;
                    cinfo.K[3] = 0; cinfo.K[4] = pcamerageom->KK.fy; cinfo.K[5] = pcamerageom->KK.cy;
                    cinfo.K[6] = 0; cinfo.K[7] = 0; cinfo.K[8] = 1;
                    cinfo.R[0] = 1; cinfo.R[1] = 0; cinfo.R[2] = 0;
                    cinfo.R[3] = 0; cinfo.R[4] = 1; cinfo.R[5] = 0;
                    cinfo.R[6] = 0; cinfo.R[7] = 0; cinfo.R[8] = 1;
                    cinfo.P[0] = pcamerageom->KK.fx; cinfo.P[1] = 0; cinfo.P[2] = pcamerageom->KK.cx; cinfo.P[3] = 0;
                    cinfo.P[4] = 0; cinfo.P[5] = pcamerageom->KK.fy; cinfo.P[6] = pcamerageom->KK.cy; cinfo.P[7] = 0;
                    cinfo.P[8] = 0; cinfo.P[9] = 0; cinfo.P[10] = 1; cinfo.P[11] = 0;
                    msg.second.at(0).publish(cinfo);
                    imagemsg->header = cinfo.header;
                    imagemsg->width = pcamerageom->width;
                    imagemsg->height = pcamerageom->height;
                    imagemsg->encoding = "rgb8";
                    imagemsg->step = pcamerageom->width*3;
                    imagemsg->data = pcameradata->vimagedata;
                    msg.second.at(1).publish(*imagemsg);
                    break;
                }
                default:
                    RAVELOG_WARN(str(boost::format("failed to output sensor type %d\n")%pdata->GetType()));
                    break;
                }
            }
        }
    
        RobotBasePtr _robot;
        string _rosnamespace;
        boost::shared_ptr<ros::NodeHandle> _node;
        map<RobotBase::AttachedSensorPtr, pair<ros::MessagePtr, vector<ros::Publisher> > > _publishers;
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
