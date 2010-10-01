// Copyright (C) 2008-2009 Rosen Diankov
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
#ifndef RAVE_ROS_ROBOT_CONTROLLER
#define RAVE_ROS_ROBOT_CONTROLLER
#include <openrave_robot_control/openravecontroller.h>

#include <ros/node_handle.h>
#include <ros/master.h>
#include <ros/time.h>
#include <ros/session.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/enable_shared_from_this.hpp> 
#include <boost/format.hpp>

#include <rave/rave.h>

#include <sensor_msgs/JointState.h>
#include <openrave_robot_control/controller_session.h>
#include <openrave_robot_control/Query.h>
#include <openrave_robot_control/Wait.h>
#include <openrave_robot_control/Cancel.h>
#include <openrave_robot_control/Brake.h>
#include <openrave_robot_control/StartTrajectory.h>
#include <openrave_robot_control/StartVelocity.h>
#include <openrave_robot_control/StartCustomString.h>

using namespace OpenRAVE;
using namespace openrave_robot_control;

class OpenRAVEClientController : public ControllerBase
{
    enum ControllerState {
        None = 0,
        Servo, // done when servoed to position and the position is held
        Traj, // done when reaches last point
        Velocity // done when joints stop moving
    };

    class TrajectoryController : public boost::enable_shared_from_this<TrajectoryController>
    {
    public:
        TrajectoryController(RobotBasePtr probot, const string& strTrajectorySession) : _fCommandTime(0), _probot(probot), _strTrajectorySession(strTrajectorySession) {}
        virtual ~TrajectoryController() {
            Destroy();
        }

        void Init()
        {
            Destroy();

            _node.reset(new ros::NodeHandle());
            
            {
                controller_session::Request req;
                controller_session::Response res;
                req.requestaccess = controller_session::Request::Access_ForceControl;
                _session = ros::session::create_session(_strTrajectorySession.c_str(),req,res);
                if( !_session )
                    throw controller_exception(str(boost::format("failed to find controller session %s")%_strTrajectorySession));
            }

            Query::Request req;
            Query::Response res;
            if( !_session->call("Query",req,res) ) {
                throw controller_exception(str(boost::format("failed to query controller of session %s")%_strTrajectorySession));
            }
            if( res.jointnames.size() == 0 ) {
                throw controller_exception(str(boost::format("no joint names controller of session %s")%_strTrajectorySession));
            }
            vector<string> vrobotjoints(_probot->GetJoints().size());
            for(size_t i = 0; i < _probot->GetJoints().size(); ++i)
                vrobotjoints[i] = _probot->GetJoints().at(i)->GetName();

            stringstream ss;
            ss << "roscontroller: " << _strTrajectorySession;
            FOREACH(itname, res.jointnames) {
                vector<string>::iterator itindex = find(vrobotjoints.begin(), vrobotjoints.end(), *itname);
                if( itindex == vrobotjoints.end() ) {            
                    RAVELOG_ERROR("failed to find joint %s\n", itname->c_str());
                    _vjointmap.push_back(-1);
                }
                else {
                    size_t index = itindex-vrobotjoints.begin();
                    _vjointmap.push_back(index);
                }
                ss << " " << *itname << " (" << _vjointmap.back() << ")";
            }

            ss << endl;
            RAVELOG_DEBUG(ss.str().c_str());

            size_t pos = _strTrajectorySession.rfind('/');
            string mechstatetopic;
            if( pos == std::string::npos )
                mechstatetopic = "mechanism_state";
            else
                mechstatetopic = _strTrajectorySession.substr(0,pos+1)+string("mechanism_state");

            _submstate = _node->subscribe(mechstatetopic, 10, &TrajectoryController::mechanismstatecb, shared_from_this());
            if( !_submstate )
                RAVELOG_ERROR("failed to subscribe to %s\n", mechstatetopic.c_str());
        }

        bool Destroy()
        {
            if( !!_session ) {
                _session->terminate();
                _session.reset();
            }

            _submstate.shutdown();
            _node.reset();

            _vjointmap.clear();
            _listTrajectories.clear();
            return true;
        }

        virtual void mechanismstatecb(const sensor_msgs::JointStateConstPtr& mstate)
        {
            boost::mutex::scoped_lock lock(_mutex);
            _mstate = *mstate.get();
        }

        void GetJointPosition(vector<dReal>& vrobotvalues)
        {
            vector<dReal> vlower, vupper;
            _probot->GetDOFLimits(vlower,vupper);
            boost::mutex::scoped_lock lock(_mutex);
            if( _vjointmap.size() != _mstate.position.size() )
                return;
            
            for(size_t i = 0; i < _vjointmap.size(); ++i) {
                if( _vjointmap[i] >= 0 ) {
                    dReal flower = vlower[_vjointmap[i]], fupper = vupper[_vjointmap[i]];
                    dReal pos = _mstate.position[i];
                    while(pos > fupper ) {
                        if( pos-2*PI >= flower )
                            pos -= 2*PI;
                        else {
                            pos = fupper;
                            break;
                        }
                    }
                    while(pos < flower) {
                        if( pos+2*PI <= fupper )
                            pos += 2*PI;
                        else {
                            pos = flower;
                            break;
                        }
                    }
                    vrobotvalues[_vjointmap[i]] = pos;
                }
            }
        }

        void GetJointVelocity(vector<dReal>& vrobotvalues)
        {
            boost::mutex::scoped_lock lock(_mutex);
            for(size_t i = 0; i < _vjointmap.size(); ++i) {
                if( _vjointmap[i] >= 0 ) {
                    vrobotvalues[_vjointmap[i]] = _mstate.velocity.at(i);
                }
            }
        }

        void GetJointTorque(vector<dReal>& vrobotvalues)
        {
            boost::mutex::scoped_lock lock(_mutex);
            for(size_t i = 0; i < _vjointmap.size(); ++i) {
                if( _vjointmap[i] >= 0 && i < _mstate.effort.size() ) {
                    vrobotvalues.at(_vjointmap[i]) = _mstate.effort.at(i);
                }
            }
        }

        template<typename T>
        void ExtractControllerValues(T& v, const vector<dReal>& vrobotvalues)
        {
            v.resize(_vjointmap.size());
            vector<int>::iterator itindex = _vjointmap.begin();
            FOREACH(itv, v) {
                *itv = vrobotvalues[*itindex++];
            }
        }

        // trajectory services
        ros::session::abstractSessionHandle _session;
        list<uint32_t> _listTrajectories; ///< trajectories currently pending for completion
        dReal _fCommandTime;

    private:
        RobotBasePtr _probot;
        string _strTrajectorySession;
        boost::shared_ptr<ros::NodeHandle> _node;
        vector<int> _vjointmap;
        ros::Subscriber _submstate;
        sensor_msgs::JointState _mstate;
        mutable boost::mutex _mutex;
    };

public:
    OpenRAVEClientController(EnvironmentBasePtr penv) : ControllerBase(penv),
        _fCommandTime(0), _bIsDone(false), _bSendTimestamps(false), _bDestroyThread(false) {
        __description = ":Interface Author: Rosen Diankov\nA simple controller interface using ROS. See openrave_robot_control ROS package.";
        _iController = -1;
        _bSyncControllers = true;
        _threadTrajectories = boost::thread(boost::bind(&OpenRAVEClientController::_TrajectoryThread,this));
    }

    virtual ~OpenRAVEClientController() {
        Destroy();

        _bDestroyThread = true;
        _threadTrajectories.join();
    }

    /// args format: host port [proxytype index]
    /// where proxytype is actarray, pos2d, or ...
    /// the order specified is the order the degrees of freedom will be arranged
    virtual bool Init(RobotBasePtr robot, const std::string& args)
    {
        Destroy();
        _iController = -1;
        _bSyncControllers = true;
        _bSendTiming = true;
        _probot = robot;
        if( !_probot )
            return false;

        int argc=0;
        ros::init(argc,NULL,"openrave", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);

        if( !ros::master::check() ) {
            RAVELOG_WARN("failed to detect the ROS master\n");
            return false;
        }

        WriteLock lock(_mutexControllers);
        stringstream ss(args);

        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss )
                break;

            if( cmd == "joints" ) {
                string jointname;
                while(1) {
                    ss >> jointname;
                    if( !ss )
                        break;

                    // look for the correct index
                    int index = -1;
                    for(int i = 0; i < (int)_probot->GetJoints().size(); ++i) {
                        if( jointname == _probot->GetJoints().at(i)->GetName() ) {
                            index = i;
                            break;
                        }
                    }

                    if( index >= 0 )
                        _setEnabledJoints.insert(pair<string, int>(jointname,index));
                    else
                        RAVELOG_WARN("failed to find joint %s\n", jointname.c_str());
                }

                break;
            }
            else if( cmd == "trajectoryservice") {
                string servicedir;
                ss >> servicedir;
                _listControllers.push_back(boost::shared_ptr<TrajectoryController>(new TrajectoryController(_probot, servicedir)));
            }
            else if( cmd == "sendtiming") {
                ss >> _bSendTiming;
            }
            else break;

            if( !ss ) {
                RAVELOG_ERROR("failed\n");
                return false;
            }
        }

        if( _setEnabledJoints.size() == 0 ) {
            RAVELOG_DEBUG("controlling using all joints of the robot\n");

            for(int i = 0; i < _probot->GetDOF(); ++i)
                _setEnabledJoints.insert(pair<string,int>(_probot->GetJoints().at(i)->GetName(), i));
        }

        string trajfilename = GetEnv()->GetHomeDirectory() + string("/") + _probot->GetName() + string(".ros.traj");
        flog.open(trajfilename.c_str());
        if( !flog )
            RAVELOG_WARN(str(boost::format("failed to open %s\n")%trajfilename));
        else
            flog << GetXMLId() << " " << _probot->GetName() << endl << endl;

        FOREACH(it, _listControllers) {
            try {
                (*it)->Init();
            }
            catch(const controller_exception& err) {
                RAVELOG_ERROR("OpenRAVEClientController error: %s\n",err.what());
                (*it)->Destroy();
            }
        }

        return true;
    }

    virtual void Destroy()
    {
        {
            WriteLock lock(_mutexControllers);
            _listControllers.clear();
        }
        _probot.reset();
        _bIsDone = false;
        _setEnabledJoints.clear();
        if( flog.is_open() )
            flog.close();
    }

    virtual void Reset(int options)
    {
        Cancel();
        WriteLock lock(_mutexControllers);
        FOREACH(itcontroller, _listControllers)
            (*itcontroller)->Init();
    }

    virtual bool SetDesired(const std::vector<dReal>& values)
    {
        ReadLock lockc(_mutexControllers);
        boost::mutex::scoped_lock lock(_mutexTrajectories);

        // set a path between the current and desired positions
        bool bSuccess = true;
        StartTrajectory::Request req;
        StartTrajectory::Response res;
        
        vector<dReal> vnewvalues;
        
        {
            RobotBase::RobotStateSaver saver(_probot);
            _probot->SetJointValues(values,true);
            _probot->GetDOFValues(vnewvalues);
        }

        // check if values are sufficiently different
        if( _vcurvalues.size() != vnewvalues.size() )
            throw openrave_exception(str(boost::format("number of current values (%d) != desird values (%d)\n")%_vcurvalues.size()%vnewvalues.size()));
//            dReal frotthresh = 0.01, ftransthresh=0.001f;
//            bool bSendTraj = false;
//            for(size_t i = 0; i < _vcurvalues.size(); ++i) {
//                float fthresh = _probot->GetJointFromDOFIndex(i)->GetType() == KinBody::Joint::JointSlider ? ftransthresh : frotthresh;
//                if( RaveFabs(_vcurvalues[i] - vnewvalues[i]) > fthresh ) {
//                    bSendTraj = true;
//                }
//            }
//
//            if( !bSendTraj ) {
//                RAVELOG_VERBOSE("setdesired sent same joint value, ignoring\n");
//                return false;
//            }

        int controllerindex = 0;
        FOREACH(ittrajcontroller, _listControllers) {
            if( _iController != controllerindex++ && _iController >= 0 )
                continue;

            if( !(*ittrajcontroller)->_session )
                continue;
            //req.requesttiming = 1; // request back the timestamps
            _bIsDone = false;
            req.hastiming = 0;
            req.interpolation = StartTrajectory::Request::Interp_Linear;
            // do not add the current position, this will be done inside the controller when the right time comes
            req.traj.points.resize(1);
            (*ittrajcontroller)->ExtractControllerValues(req.traj.points.at(0).positions, vnewvalues);

            if( (*ittrajcontroller)->_session->call("StartTrajectory", req,res) )
                (*ittrajcontroller)->_listTrajectories.push_back(res.commandid);
            else {
                RAVELOG_ERROR("failed to start trajectory\n");
                bSuccess = false;
                continue;
            }

            RAVELOG_DEBUG("started trajectory %d\n", res.commandid);
            //res.timestamps // final timestamps
        }

        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( !ptraj )
            return false;

        ReadLock lockc(_mutexControllers);
        boost::mutex::scoped_lock lock(_mutexTrajectories);

        bool bSuccess = true;
        StartTrajectory::Request req;
        StartTrajectory::Response res;
        
        int controllerindex = 0;
        int commandid = 0;
        FOREACH(ittrajcontroller, _listControllers) {
            if( _iController != controllerindex++ && _iController >= 0 )
                continue;
            if( !(*ittrajcontroller)->_session )
                continue;

            //req.requesttiming = 1; // request back the timestamps
            _bIsDone = false;
            req.hastiming = _bSendTiming && ptraj->GetTotalDuration()>0;
            if( ptraj->GetTotalDuration()>0 ) {
                // resample every 0.01s
                dReal ftime = 0, sampletime = 0.01f;
                Trajectory::TPOINT tp;
                req.interpolation = StartTrajectory::Request::Interp_Cubic;
                req.traj.points.resize((ptraj->GetTotalDuration()/sampletime)+1);
                FOREACH(ittraj,req.traj.points) {
                    ptraj->SampleTrajectory(ftime,tp);
                    ittraj->time = tp.time;
                    (*ittrajcontroller)->ExtractControllerValues(ittraj->positions, tp.q);
                    if( (int)tp.qtorque.size() == ptraj->GetDOF() ) {
                        (*ittrajcontroller)->ExtractControllerValues(ittraj->torques, tp.qtorque);
                    }
                    ftime += sampletime;
                }
            }
            else {
                req.interpolation = StartTrajectory::Request::Interp_Linear;
                req.traj.points.resize(ptraj->GetPoints().size());
                typeof(req.traj.points.begin()) ittraj = req.traj.points.begin(); 
                FOREACHC(itpoint, ptraj->GetPoints()) {
                    ittraj->time = itpoint->time;
                    (*ittrajcontroller)->ExtractControllerValues(ittraj->positions, itpoint->q);
                    if( (int)itpoint->qtorque.size() == ptraj->GetDOF() ) {
                        (*ittrajcontroller)->ExtractControllerValues(ittraj->torques, itpoint->qtorque);
                    }
                    ++ittraj;
                }
            }

            if( (*ittrajcontroller)->_session->call("StartTrajectory", req,res) )
                (*ittrajcontroller)->_listTrajectories.push_back(res.commandid);
            else {
                RAVELOG_ERROR("failed to start trajectory\n");
                bSuccess = false;
                continue;
            }

            RAVELOG_DEBUG("started trajectory %d\n", res.commandid);
            commandid=res.commandid;
            //res.timestamps // final timestamps
        }

        if( !!flog ) {
            flog << endl << "trajectory: " << commandid << endl;
            ptraj->Write(flog, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        }

        return bSuccess;
    }

    virtual bool SetTorque(const vector<dReal>& vtorques)
    {
        ReadLock lockc(_mutexControllers);
        boost::mutex::scoped_lock lock(_mutexTrajectories);

        bool bSuccess = true;
        StartTorque::Request req;
        StartTorque::Response res;
        int controllerindex = 0;
        FOREACH(ittrajcontroller, _listControllers) {
            if( _iController != controllerindex++ && _iController >= 0 )
                continue;
            if( !(*ittrajcontroller)->_session )
                continue;

            (*ittrajcontroller)->ExtractControllerValues(req.torques, vtorques);

            if( (*ittrajcontroller)->_session->call("StartTorque", req,res) ) {
                (*ittrajcontroller)->_listTrajectories.push_back(res.commandid);
                RAVELOG_DEBUG(str(boost::format("started torque cmd %d on %s\n")%res.commandid%(*ittrajcontroller)->_session->GetSessionName()));
        }
            else {
                RAVELOG_ERROR("failed to start torque\n");
                bSuccess = false;
            }
        }

        return bSuccess;
    }

    virtual bool SetVelocity(const vector<dReal>& vvelocities)
    {
        ReadLock lockc(_mutexControllers);
        boost::mutex::scoped_lock lock(_mutexTrajectories);

        bool bSuccess = true;
        StartVelocity::Request req;
        StartVelocity::Response res;
        int controllerindex = 0;
        FOREACH(ittrajcontroller, _listControllers) {
            if( _iController != controllerindex++ && _iController >= 0 )
                continue;
            if( !(*ittrajcontroller)->_session )
                continue;

            (*ittrajcontroller)->ExtractControllerValues(req.velocities, vvelocities);

            if( (*ittrajcontroller)->_session->call("StartVelocity", req,res) ) {
                (*ittrajcontroller)->_listTrajectories.push_back(res.commandid);
                RAVELOG_DEBUG(str(boost::format("started velocity cmd %d on %s\n")%res.commandid%(*ittrajcontroller)->_session->GetSessionName()));
        }
            else {
                RAVELOG_ERROR("failed to start torque\n");
                bSuccess = false;
            }
        }

        return bSuccess;
    }

    virtual bool SetCustomStringCommand(const string& cmd)
    {
        ReadLock lockc(_mutexControllers);
        boost::mutex::scoped_lock lock(_mutexTrajectories);

        bool bSuccess = true;
        StartCustomString::Request req;
        req.input = cmd;
        StartCustomString::Response res;
        int controllerindex = 0;
        FOREACH(ittrajcontroller, _listControllers) {
            if( _iController != controllerindex++ && _iController >= 0 )
                continue;
            if( !(*ittrajcontroller)->_session )
                continue;

            if( (*ittrajcontroller)->_session->call("StartCustomString", req,res) ) {
                (*ittrajcontroller)->_listTrajectories.push_back(res.commandid);
                RAVELOG_DEBUG(str(boost::format("started custom string cmd %d on %s\n")%res.commandid%(*ittrajcontroller)->_session->GetSessionName()));
            }
            else {
                RAVELOG_ERROR("failed to start torque\n");
                bSuccess = false;
            }
        }
        
        return bSuccess;
    }
    
    /// cancel a command
    virtual bool Cancel(int commandid=0)
    {
        ReadLock lockc(_mutexControllers);
        boost::mutex::scoped_lock lock(_mutexTrajectories);

        Cancel::Request req; req.commandid = commandid;
        Cancel::Response res;
        bool bSuccess = true;
        int controllerindex = 0;
        FOREACH(ittrajcontroller, _listControllers) {
            if( _iController != controllerindex++ && _iController >= 0 )
                continue;
            if( !(*ittrajcontroller)->_session )
                continue;

            if( !(*ittrajcontroller)->_session->call("Cancel", req,res) ) {
                RAVELOG_WARN("controller failed to cancel for %s\n",(*ittrajcontroller)->_session->GetSessionName().c_str());
                bSuccess = false;
            }

            if( commandid == 0 )
                (*ittrajcontroller)->_listTrajectories.clear();
            else
                (*ittrajcontroller)->_listTrajectories.remove(commandid);
        }

        return bSuccess;
    }

    virtual bool SimulationStep(dReal fTimeElapsed)
    {
        if( !!_probot ) {
            vector<dReal> values;
            _probot->GetDOFValues(values);
            ReadLock lock(_mutexControllers);
            FOREACHC(it,_listControllers)
                (*it)->GetJointPosition(values);
            _vcurvalues = values;
            _probot->SetJointValues(values, true);
        }
        return IsDone();
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if( cmd == "cancel" ) {
            int commandid=0;
            is >> commandid;
            return Cancel(commandid);
        }
        else if( cmd == "settorque" ) {
            vector<dReal> vtorques(_probot->GetDOF(),0);
            while(!is.eof()) {
                int index=-1;
                is >> index;
                if( !is )
                    break;
                if( index < 0 || index >= _probot->GetDOF() ) {
                    RAVELOG_WARN("bad index on settorque command");
                    return false;
                }
                is >> vtorques[index];
                if( !is ) {
                    RAVELOG_WARN("settorque command parse error");
                    return false;
                }
            }

            return SetTorque(vtorques);
        }
        else if( cmd == "setvelocity" ) {
            vector<dReal> vvelocities(_probot->GetDOF(),0);
            while(!is.eof()) {
                int index=-1;
                is >> index;
                if( !is )
                    break;
                if( index < 0 || index >= _probot->GetDOF() ) {
                    RAVELOG_WARN("bad index on settorque command");
                    return false;
                }
                is >> vvelocities[index];
                if( !is ) {
                    RAVELOG_WARN("settorque command parse error");
                    return false;
                }
            }

            return SetVelocity(vvelocities);
        }
        else if( cmd == "setcustom" ) {
            stringbuf buf;
            is.get(buf,0);
            SetCustomStringCommand(buf.str());
        }
        else if( cmd == "synccontrollers" ) {
            is >> _bSyncControllers;
            _CheckDoneStatus();
        }
        else if( cmd == "setcontroller" ) {
            is >> _iController;
            _CheckDoneStatus();
        }
        else if( cmd == "reset" ) {
            _iController = -1;
            Cancel(0);
        }
        else
            throw openrave_exception(str(boost::format("command %s not supported")%cmd),ORE_CommandNotSupported);
        return false;
    }

    virtual bool IsDone() { return _bIsDone; }

    virtual ActuatorState GetActuatorState(int index)
    {
        return AS_Idle;
    }

    virtual dReal GetTime() const
    {
        return _fCommandTime;
    }
    virtual RobotBasePtr GetRobot() const { return _probot; }

    virtual void GetVelocity(std::vector<dReal>& vel) const
    {
        ReadLock lock(_mutexControllers);
        vel.resize(_probot->GetDOF());
        FOREACH(it, _listControllers)
            (*it)->GetJointVelocity(vel);
    }
    
    virtual void GetTorque(std::vector<dReal>& torque) const
    {
        ReadLock lock(_mutexControllers);
        torque.resize(_probot->GetDOF());
        FOREACH(it, _listControllers)
            (*it)->GetJointTorque(torque);
    }

private:
    void _TrajectoryThread()
    {
        while(!_bDestroyThread) {
            _CheckDoneStatus();
            if( ros::isInitialized() )
                ros::spinOnce();
            usleep(10000); // query every 10ms
        }
    }

    void _CheckDoneStatus()
    {
        Query::Request req;
        Query::Response res;

        // check if the first trajectory is done
        ReadLock lockc(_mutexControllers);
        if( _listControllers.size() == 0 )
            return;
        boost::mutex::scoped_lock lock(_mutexTrajectories);

        bool bPopTrajectory = true;

        if( _bSyncControllers ) {
            // check if done
            FOREACH(ittraj, _listControllers) {
                if( !(*ittraj)->_session )
                    continue;

                if( (*ittraj)->_listTrajectories.size() == 0 ) {
                    bPopTrajectory = false;
                    break;
                }
                req.commandid = (*ittraj)->_listTrajectories.front();
                if( !(*ittraj)->_session->call("Query", req,res) ) {
                    RAVELOG_ERROR("trajectory query failed\n");
                    bPopTrajectory = false;
                    continue;
                }

                _fCommandTime = res.commandtime;
                (*ittraj)->_fCommandTime = res.commandtime;

                if( !res.commanddone ) {
                    bPopTrajectory = false;
                    break;
                }
            }

            if( bPopTrajectory ) {
                size_t numtraj=0;
                FOREACH(ittraj, _listControllers) {
                    if( !(*ittraj)->_session )
                        continue;
                    (*ittraj)->_listTrajectories.pop_front();
                    numtraj = (*ittraj)->_listTrajectories.size();
                }
                RAVELOG_DEBUG(str(boost::format("robot trajectory finished, left: %d\n")%numtraj));
            }
        }
        else {
            // delete commands as they finish
            FOREACH(ittraj, _listControllers) {
                if( !(*ittraj)->_session )
                    continue;

                if( (*ittraj)->_listTrajectories.size() > 0 ) {
                    req.commandid = (*ittraj)->_listTrajectories.front();
                    if( !(*ittraj)->_session->call("Query", req,res) ) {
                        RAVELOG_ERROR("trajectory query failed\n");
                        continue;
                    }

                    (*ittraj)->_fCommandTime = res.commandtime;

                    if( res.commanddone )
                        (*ittraj)->_listTrajectories.pop_front();
                }
            }
        }

        bool bIsDone = true;
        int controllerindex = 0;
        FOREACH(ittraj, _listControllers) {
            if( _iController != controllerindex++ && _iController >= 0 )
                continue;
            if( !(*ittraj)->_session )
                continue;

            if( (*ittraj)->_listTrajectories.size() != 0) {
                bIsDone = false;
                break;
            }
            break;
        }

        _bIsDone = bIsDone;
    }

    RobotBasePtr _probot;           ///< robot owning this controller

    vector<dReal> _vecdesired;
    set< pair<string, int> > _setEnabledJoints; // set of enabled joints and their indices
    vector<dReal> _vcurvalues; ///< current robot values

    ofstream flog;
    int logid;
    float _fCommandTime; // time of the current command
    
    list<boost::shared_ptr<TrajectoryController> > _listControllers;

    mutable boost::mutex _mutexTrajectories;
    mutable boost::shared_mutex _mutexControllers;
    boost::thread _threadTrajectories;

    int _iController; ///< if < 0, send incoming commands to all controllers, otherwise send only to specified controller

    bool _bIsDone, _bSendTiming;
    bool _bSendTimestamps; ///< if true, will send timestamps along with traj
    bool _bDestroyThread; ///< if true, destroy the thread
    bool _bSyncControllers; ///< controllers will be started and stopped at the same time

    typedef boost::shared_lock<boost::shared_mutex> ReadLock;
    typedef boost::unique_lock<boost::shared_mutex> WriteLock;
};

#endif
