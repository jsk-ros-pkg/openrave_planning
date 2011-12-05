// Software License Agreement (BSD License)
// Copyright (c) 2010, Rosen Diankov
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// \author Rosen Diankov
#ifndef ROS_BINDINGS_PROBLEM
#define ROS_BINDINGS_PROBLEM

#include "plugindefs.h"
#include <tf/transform_listener.h>
#include <posedetection_msgs/ObjectDetection.h>
#include <ros/package.h>
#include <boost/filesystem/operations.hpp>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ROSBindings : public ProblemInstance
{
public:
    ROSBindings(EnvironmentBasePtr penv) : ProblemInstance(penv), _bDestroyThread(true) {
        __description = ":Interface Author: Rosen Diankov\n\nBindings Simplifying integration with ROS";
        RegisterCommand("SetLocalizationFromTF",boost::bind(&ROSBindings::SetLocalizationFromTF,this,_1,_2),
                        "Set a robot's navigation from a published tf frame");
        RegisterCommand("SetLocalizationFromDetection",boost::bind(&ROSBindings::SetLocalizationFromDetection,this,_1,_2),
                        "Set a robot's navigation from a published object");
        RegisterCommand("MoveBase",boost::bind(&ROSBindings::MoveBase,this,_1,_2),
                        "Uses ROS's navigation stack to move the base");
//        RegisterCommand("MoveBase",boost::bind(&ROSBindings::ShowPointCloud,this,_1,_2),
//                        "Uses ROS's navigation stack to move the base");
    }
    virtual ~ROSBindings() {
        Destroy();
    }

    virtual void Destroy()
    {
        _bDestroyThread = true;
        _localizingrobots.clear();
        _tflistener.reset();
        _ros.reset();
        _threadros.join();
        {
            boost::mutex::scoped_lock lock(_mutexrobots);
            _listUpdatedRobots.clear();
        }
        ProblemInstance::Destroy();
    }

    virtual void Reset()
    {
        _localizingrobots.clear();
        {
            boost::mutex::scoped_lock lock(_mutexrobots);
            _listUpdatedRobots.clear();
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
        _threadros = boost::thread(boost::bind(&ROSBindings::_threadrosfn, this));
        _tflistener.reset(new tf::TransformListener(*_ros));
        return 0;
    }

    virtual bool SimulationStep(dReal fElapsedTime)
    {
        boost::mutex::scoped_lock lock(_mutexrobots);
        FOREACH(it,_listUpdatedRobots)
        it->first->SetTransform(it->second);
        _listUpdatedRobots.clear();
        return false;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return ProblemInstance::SendCommand(sout,sinput);
    }
protected:
    inline boost::shared_ptr<ROSBindings> shared_problem() {
        return boost::static_pointer_cast<ROSBindings>(shared_from_this());
    }
    inline boost::shared_ptr<ROSBindings const> shared_problem_const() const {
        return boost::static_pointer_cast<ROSBindings const>(shared_from_this());
    }

    virtual void _threadrosfn()
    {
        while(!_bDestroyThread && ros::ok()) {
            ros::spinOnce();
            usleep(1000); // query every 1ms?
        }
    }

    void UpdateRobotFromTF(const ros::WallTimerEvent& event, RobotBasePtr probot, string mapframe, geometry_msgs::PoseStamped posestamped)
    {
        //RAVELOG_INFO(str(boost::format("updating robot %s\n")%probot->GetName()));
        geometry_msgs::PoseStamped poseout;
        try {
            //posestamped.header.stamp = ros::Time();
            _tflistener->transformPose(mapframe, posestamped, poseout);
            boost::mutex::scoped_lock lock(_mutexrobots);
            _listUpdatedRobots.push_back(make_pair(probot,GetTransform(poseout.pose)));
        }
        catch(std::runtime_error& ex) {
            RAVELOG_DEBUG(str(boost::format("failed to get tf frames %s robot %s: %s\n")%posestamped.header.frame_id%probot->GetName()%ex.what()));
        }
    }

    bool SetLocalizationFromTF(ostream& sout, istream& sinput)
    {
        string cmd, mapframe="/map";
        RobotBasePtr probot;
        geometry_msgs::PoseStamped posestamped; posestamped.pose = GetPose(Transform());
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "robot" ) {
                string robotname;
                sinput >> robotname;
                probot = GetEnv()->GetRobot(robotname);
            }
            else if( cmd == "map" )
                sinput >> mapframe;
            else if( cmd == "offset" ) {
                Transform toffset;
                sinput >> toffset;
                posestamped.pose = GetPose(toffset);
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

        BOOST_ASSERT(!!probot);
        posestamped.header.frame_id = probot->GetLinks().at(0)->GetName();
        _localizingrobots[probot] = boost::shared_ptr<ros::WallTimer>(new ros::WallTimer(_ros->createWallTimer(ros::WallDuration(0.01), boost::bind(&ROSBindings::UpdateRobotFromTF,this,_1,probot,mapframe,posestamped))));
        return true;
    }

    virtual void UpdateRobotFromDetection(const boost::shared_ptr<posedetection_msgs::ObjectDetection const>& topicmsg, RobotBasePtr probot, bool b2DLocalization)
    {
        if( topicmsg->objects.size() == 0 )
            return;

        boost::mutex::scoped_lock lock(_mutexrobots);
        geometry_msgs::PoseStamped posestamped, poseout;
        Transform tnew;
        string strrobotbaselink = probot->GetLinks().at(0)->GetName();
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itobj,topicmsg->objects) {
            string objfilename = resolveName(itobj->type);
            if( objfilename.size() > 0 ) {
                boost::filesystem::path absfilename = boost::filesystem::path(objfilename);
                FOREACH(itbody,vbodies) {
                    if( boost::filesystem::equivalent(absfilename, boost::filesystem::path((*itbody)->GetXMLFilename())) ) {
                        posestamped.pose = GetPose(GetTransform(itobj->pose));
                        posestamped.header = topicmsg->header;

                        try {
                            _tflistener->transformPose(strrobotbaselink, posestamped, poseout);
                            tnew = GetTransform(poseout.pose).inverse() * (*itbody)->GetTransform();
                        }
                        catch(std::runtime_error& ex) {
                            try {
                                // try getting the latest value by passing a 0 timestamp
                                posestamped.header.stamp = ros::Time();
                                _tflistener->transformPose(strrobotbaselink, posestamped, poseout);
                                tnew = GetTransform(poseout.pose).inverse() * (*itbody)->GetTransform();
                            }
                            catch(std::runtime_error& ex) {
                                RAVELOG_WARNA("failed to get tf frames %s (body link:%s) for object %s\n",posestamped.header.frame_id.c_str(), strrobotbaselink.c_str(), itobj->type.c_str());
                                continue; //tnew = GetTransform(itobj->pose);
                            }
                        }

                        if( b2DLocalization ) {
                            Transform trobot = probot->GetTransform();
                            dReal zanglediffd2 = RaveAtan2(tnew.rot.w,tnew.rot.x) - RaveAtan2(trobot.rot.w,trobot.rot.x);
                            trobot.rot = quatMultiply(Vector(RaveCos(zanglediffd2),0,0,RaveSin(zanglediffd2)),trobot.rot);
                            trobot.trans.x = tnew.trans.x;
                            trobot.trans.y = tnew.trans.y;
                            _listUpdatedRobots.push_back(make_pair(probot,trobot));
                        }
                        else
                            _listUpdatedRobots.push_back(make_pair(probot,tnew));
                        return;
                    }
                }
            }
        }
    }

    bool SetLocalizationFromDetection(ostream& sout, istream& sinput)
    {
        string cmd, topic;
        RobotBasePtr probot;
        uint32_t queuesize = 1;
        bool b2DLocalization=true;
        geometry_msgs::PoseStamped posestamped; posestamped.pose = GetPose(Transform());
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "robot" ) {
                string robotname;
                sinput >> robotname;
                probot = GetEnv()->GetRobot(robotname);
            }
            else if( cmd == "topic" )
                sinput >> topic;
            else if( cmd == "queuesize" )
                sinput >> queuesize;
            else if( cmd == "6dlocalization" )
                b2DLocalization = false;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !probot )
            return false;
        ros::Subscriber sub = _ros->subscribe<posedetection_msgs::ObjectDetection>(topic,queuesize,boost::bind(&ROSBindings::UpdateRobotFromDetection,this,_1,probot, b2DLocalization));
        if( !sub )
            return false;
        _localizingrobots[probot] = boost::shared_ptr<ros::Subscriber>(new ros::Subscriber(sub));
        return true;
    }

    bool MoveBase(ostream& sout, istream& sinput)
    {
        string cmd;
        string action="move_base";
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        bool bHasGoal=false;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "frame_id" )
                sinput >> goal.target_pose.header.frame_id;
            else if( cmd == "action" )
                sinput >> action;
            else if( cmd == "goal" ) {
                Transform t;
                sinput >> t;
                goal.target_pose.pose.position.x = t.trans.x;
                goal.target_pose.pose.position.y = t.trans.y;
                goal.target_pose.pose.position.z = t.trans.z;
                goal.target_pose.pose.orientation.x = t.rot.y;
                goal.target_pose.pose.orientation.y = t.rot.z;
                goal.target_pose.pose.orientation.z = t.rot.w;
                goal.target_pose.pose.orientation.w = t.rot.x;
                bHasGoal = true;
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

        if( !bHasGoal ) {
            RAVELOG_WARN("no goal specified\n");
            return false;
        }
        MoveBaseClient ac(action, true);

        //wait for the action server to come up
        if(!ac.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("could not start action server");
            return false;
        }

        goal.target_pose.header.stamp = ros::Time::now();

        RAVELOG_INFO("Sending goal, press 'c' to cancel\n");
        ac.sendGoal(goal);
        while(!ac.waitForResult(ros::Duration(1.0)) ) {
            GetEnv()->UpdatePublishedBodies();
            // check for some non-blocking method
//            if( getchar() == 'c' ) {
//                RAVELOG_WARN("canceling goal\n");
//                ac.cancelGoal();
//                return false;
//            }
        }
        actionlib::SimpleClientGoalState state = ac.getState();
        RAVELOG_INFO(str(boost::format("goal status %s: %s\n")%state.toString()%state.getText()));
        return state == actionlib::SimpleClientGoalState::SUCCEEDED;
    }

    boost::shared_ptr<tf::TransformListener> _tflistener;
    boost::shared_ptr<ros::NodeHandle> _ros;
    map<RobotBasePtr,boost::shared_ptr<void> > _localizingrobots;
    list<pair<RobotBasePtr,Transform> > _listUpdatedRobots;
    boost::thread _threadros;
    boost::mutex _mutexrobots;
    bool _bDestroyThread;
};

#endif
