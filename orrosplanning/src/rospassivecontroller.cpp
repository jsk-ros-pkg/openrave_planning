// Software License Agreement (BSD License)
// Copyright (c) 2008, Willow Garage, Inc.
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
#include "plugindefs.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/static_assert.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <sensor_msgs/JointState.h>

class ROSPassiveController : public ControllerBase
{
public:
    ROSPassiveController(EnvironmentBasePtr penv, std::istream& ss) : ControllerBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nAllows OpenRAVE to listen to ROS messages and update the robot, this does not control the robot in any way.";
        _bDestroyThread = true;
        _nControlTransformation = 0;
        _jointstatetopic="/joint_states";
        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss ) {
                break;
            }

            if( cmd == "jointstate") {
                ss >> _jointstatetopic;
            }
            else {
                break;
            }
        }
    }
    virtual ~ROSPassiveController() {
        _Destroy();
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _Destroy();
        _probot = robot;
        _dofindices = dofindices;
        _nControlTransformation = nControlTransformation;

        int argc=0;
        ros::init(argc,NULL,"openrave_rospassivecontroller", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        if( !ros::master::check() ) {
            return false;
        }

        Reset(0);
        _node.reset(new ros::NodeHandle());
        _bDestroyThread = false;
        _threadros = boost::thread(boost::bind(&ROSPassiveController::_threadrosfn, this));

        _subjointstate = _node->subscribe(_jointstatetopic, 10, &ROSPassiveController::_jointstatecb, this);
        FOREACHC(itjoint,_probot->GetJoints()) {
            _vjointnames.push_back(make_pair((*itjoint)->GetName(), (*itjoint)->GetJointIndex()));
        }

        _probot->GetDOFValues(_vlastjointvalues);
        _vlastjointvel.resize(0);
        _vlastjointvel.resize(_vlastjointvalues.size(),0.0f);
        _vlastjointtorque = _vlastjointvel;
        return true;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const
    {
        return _dofindices;
    }
    virtual int IsControlTransformation() const
    {
        return _nControlTransformation;
    }

    virtual void Reset(int options)
    {
        _controllertime = 0;
        _starttime = ros::Time(0);
    }

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
    {
        return false;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        return false;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
    }

    virtual bool IsDone()
    {
        return true;
    }

    virtual dReal GetTime() const
    {
        return _controllertime;
    }

    virtual void GetVelocity(std::vector<dReal>& vel) const {
        boost::mutex::scoped_lock lock(_mutex);
        vel = _vlastjointvel;
    }

    virtual void GetTorque(std::vector<dReal>& torque) const {
        boost::mutex::scoped_lock lock(_mutex);
        torque = _vlastjointtorque;
    }

    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }

protected:
    virtual void _Destroy()
    {
        _bDestroyThread = true;
        _node.reset();
        _subjointstate.shutdown();
        _threadros.join(); // deadlock condition with environment
        _probot.reset();
        _vjointnames.clear();
    }

    virtual void _jointstatecb(const sensor_msgs::JointStateConstPtr& jstate)
    {
        boost::mutex::scoped_lock lock(_mutex);
        if( _bDestroyThread ) {
            return;
        }
        EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        _probot->GetDOFValues(_vlastjointvalues);
        //resize the vel and torque vectors if necessary:
        _vlastjointvel.resize(_vlastjointvalues.size(),0.0f);
        _vlastjointtorque.resize(_vlastjointvalues.size(),0.0f);
        for(size_t i = 0; i < jstate->name.size(); ++i) {
            KinBody::JointPtr pjoint = _probot->GetJoint(jstate->name[i]);
            if( !pjoint ) {
                RAVELOG_VERBOSE(str(boost::format("could not find joint %s\n")%jstate->name[i]));
                continue;
            }
            else {
                if( i < jstate->position.size() ) {
                    _vlastjointvalues.at(pjoint->GetJointIndex()) = jstate->position.at(i);
                }
                if( i < jstate->velocity.size() ) {
                    _vlastjointvel.at(pjoint->GetJointIndex()) = jstate->velocity.at(i);
                }
                if( i < jstate->effort.size() )  {
                    _vlastjointtorque.at(pjoint->GetJointIndex()) = jstate->effort.at(i);
                }
            }
        }
        if( _starttime.toSec() == 0 ) {
            _starttime = jstate->header.stamp;
        }
        _controllertime = (jstate->header.stamp-_starttime).toSec();
        _probot->SetDOFValues(_vlastjointvalues);
    }

    virtual void _threadrosfn()
    {
        while(!_bDestroyThread && ros::ok()) {
            ros::spinOnce();
            usleep(1000); // query every 1ms?
        }
    }

    bool _bDestroyThread;
    RobotBasePtr _probot;
    boost::shared_ptr<ros::NodeHandle> _node;
    boost::thread _threadros;
    ros::Subscriber _subjointstate;
    vector<dReal> _vlastjointvalues, _vlastjointvel, _vlastjointtorque;
    vector< pair<string,int> > _vjointnames;
    ros::Time _starttime;
    dReal _controllertime;
    std::vector<int> _dofindices;
    int _nControlTransformation;
    string _jointstatetopic;
    mutable boost::mutex _mutex;
};

ControllerBasePtr CreateROSPassiveController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new ROSPassiveController(penv,sinput));
}
