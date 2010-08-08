// Copyright (c) 2009-2010 Rosen Diankov
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
#include <openrave-core.h> // declare first

#include <cstdio>
#include <cstdlib>
#include <string>
#include <set>
#include <vector>

#include <sys/timeb.h>    // ftime(), struct timeb
#include <sys/time.h>

#include <ros/node_handle.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/static_assert.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/enable_shared_from_this.hpp> 
#include <boost/format.hpp>

#include <sensor_msgs/JointState.h>
#include <openrave_robot_control/controller_session.h>
#include <openrave_robot_control/Query.h>
#include <openrave_robot_control/Wait.h>
#include <openrave_robot_control/Cancel.h>
#include <openrave_robot_control/Brake.h>
#include <openrave_robot_control/StartTrajectory.h>
#include <openrave_robot_control/StartVelocity.h>
#include <openrave_robot_control/StartTorque.h>
#include <openrave_robot_control/StartCustomString.h>

#include <tf/transform_broadcaster.h>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

namespace openrave_robot_control {

using namespace std;
using namespace OpenRAVE;

inline uint32_t timeGetTime()
{
#ifdef _WIN32
    _timeb t;
    _ftime(&t);
#else
    timeb t;
    ftime(&t);
#endif

    return (uint32_t)(t.time*1000+t.millitm);
}

inline uint64_t GetMicroTime()
{
#ifdef _WIN32
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    return (uint64_t)t.tv_sec*1000000+t.tv_usec;
#endif
}

struct controller_exception : std::exception
{
    controller_exception() : std::exception(), _s("unknown exception") {}
    controller_exception(const string& s) : std::exception() { _s = "Controller: " + s; }
    virtual ~controller_exception() throw() {}
    char const* what() const throw() { return _s.c_str(); }
    string _s;
};

/// Robot Controller providing ROS interfaces
class OpenRAVEController : public boost::enable_shared_from_this<OpenRAVEController>
{
protected:
    enum StateType {
        State_Idle = openrave_robot_control::Query::Response::State_Idle,
        State_Moving = openrave_robot_control::Query::Response::State_Moving,
        State_Ready = openrave_robot_control::Query::Response::State_Ready,
        State_Stalled = openrave_robot_control::Query::Response::State_Stalled
    };
    enum AccessType {
        Access_Read = openrave_robot_control::controller_session::Request::Access_Read,
        Access_Control = openrave_robot_control::controller_session::Request::Access_Control,
        Access_ForceControl = openrave_robot_control::controller_session::Request::Access_ForceControl
    };
    enum CommandType
    {
        Command_Trajectory=1<<28,
        Command_Velocity=2<<28,
        Command_Torque=3<<28,
        Command_String=4<<28,
    };
    enum CommandStatus
    {
        Status_Failed=0,
        Status_Running=1,
        Status_Finished=2
    };

    template <class T> boost::shared_ptr<T> as() {
        return boost::dynamic_pointer_cast<T>(shared_from_this()); 
    }

    class Command
    {
    public:
        Command(uint32_t commandid) : _commandid(commandid), _bExecuting(false) {}
        virtual ~Command() {}

        /// \return true if the command is done
        virtual CommandStatus run() {
            if( !_bExecuting )
                _start();
            return Status_Running;
        }

        //virtual CommandType getType() const = 0;
        virtual float getCommandTime() { return _bExecuting ? (ros::Time::now()-_commandstart).toSec() : 0; }
        virtual uint32_t getCommandId() { return _commandid; }
        
    protected:
        /// sets up the robot to execute the command
        virtual void _start() {
            _commandstart = ros::Time::now();
            _bExecuting = true;
        }

    protected:
        uint32_t _commandid;
        ros::Time _commandstart;
        bool _bExecuting;
    };

    template <class StartCommand, class RunCommand, class FinishCommand>
    class BindCommand : public Command
    {
    public:
        BindCommand(uint32_t commandid, const StartCommand& startcommand, const RunCommand& runcommand, const FinishCommand& finishcommand)
            : Command(commandid), _startcommand(startcommand), _runcommand(runcommand), _finishcommand(finishcommand) {}
        virtual ~BindCommand() {
            if( _bExecuting )
                _finishcommand();
        }

        /// \return true if the command is done
        virtual CommandStatus run() {
            Command::run();
            return _runcommand(getCommandTime());
        }
        
    protected:
        /// sets up the robot to execute the command
        virtual void _start() {
            _startcommand();
            Command::_start();
        }

        StartCommand _startcommand;
        RunCommand _runcommand;
        FinishCommand _finishcommand;
    };

    template <class StartCommand, class RunCommand, class FinishCommand>
    Command* CreateBindCommand(uint32_t commandid, const StartCommand& startcommand, const RunCommand& runcommand, const FinishCommand& finishcommand)
    {
        return new BindCommand<StartCommand,RunCommand,FinishCommand>(commandid,startcommand,runcommand,finishcommand);
    }

    class SessionState
    {
    public:
        SessionState(AccessType access) : _access(access) {}
        virtual ~SessionState() {}
        AccessType _access;
    };

public:
    OpenRAVEController(const string& robotfile, const string& manipname, const vector<string>& vActiveJointNames, float fMaxVelMult) : _bShutdown(false), _robotfile(robotfile), _manipname(manipname), _fMaxVelMult(fMaxVelMult), _state(State_Idle), _bBraked(false), _nNextCommandId(1) {
        _vActiveJointNames = vActiveJointNames;
    }

    virtual ~OpenRAVEController()
    {
        shutdown();

        // destroy tf server
        _tfbroadcaster.reset();

        if( !!_probot && !!_penv )
            _penv->RemoveKinBody(_probot);
        _probot.reset();
        if( !!_penv )
            _penv->Destroy();
        _penv.reset();

        _srvSession.shutdown();
        _srvQuery.shutdown();
        _srvWait.shutdown();
        _srvCancel.shutdown();
        _srvBreak.shutdown();
        _srvStartTrajectory.shutdown();
        _srvStartVelocity.shutdown();
        _srvStartTorque.shutdown();
        _srvStartCustomString.shutdown();
        _pubmstate.shutdown();
        _ros.reset();
    }

    virtual void init()
    {
        _ros.reset(new ros::NodeHandle());
        _InitRobot(_robotfile,_manipname);

        _mstate.position.resize(GetDOF());
        _mstate.velocity.resize(GetDOF());
        _mstate.effort.resize(GetDOF());
        _mstate.name.resize(GetDOF());
        std::copy(_vActiveJointNames.begin(),_vActiveJointNames.end(),_mstate.name.begin());

        _srvSession = _ros->advertiseService("controller_session",&OpenRAVEController::session_callback,this);
        _srvQuery = _ros->advertiseService("Query",&OpenRAVEController::Query, this);
        _srvWait = _ros->advertiseService("Wait",&OpenRAVEController::Wait, this);
        _srvCancel = _ros->advertiseService("Cancel",&OpenRAVEController::Cancel, this);
        _srvBreak = _ros->advertiseService("Brake",&OpenRAVEController::Brake, this);
        _srvStartTrajectory = _ros->advertiseService("StartTrajectory",&OpenRAVEController::StartTrajectory, this);
        _srvStartVelocity = _ros->advertiseService("StartVelocity",&OpenRAVEController::StartVelocity, this);
        _srvStartTorque = _ros->advertiseService("StartTorque",&OpenRAVEController::StartTorque, this);
        _srvStartCustomString = _ros->advertiseService("StartCustomString",&OpenRAVEController::StartCustomString, this);

        _sessionname = _ros->resolveName("controller_session");
        _pubmstate = _ros->advertise<sensor_msgs::JointState>("mechanism_state",10); // tf server

        if( !!_probot )
            // start tf server
            _tfbroadcaster.reset(new tf::TransformBroadcaster());
    }

    virtual void shutdown()
    {
        _bShutdown = true;
        if( !!_ros )
            _ros->shutdown();
    }

    virtual bool session_callback(openrave_robot_control::controller_session::Request& req, openrave_robot_control::controller_session::Response& res)
    {
        if( req.sessionid != 0 ) {
            boost::mutex::scoped_lock lock(_mutexSession);

            if( req.sessionid == -1 ) {
                // destroy all sessions
                ROS_INFO("destroying all sessions");
                _clearCommands();
                _mapsessions.clear();
            }
            else {
                // destory the session
                if( _mapsessions.find(req.sessionid) == _mapsessions.end() )
                    return false;
                
                if( _mapsessions[req.sessionid]->_access == Access_Control )
                    _clearCommands();
                _mapsessions.erase(req.sessionid);
                ROS_INFO("destroyed openrave controller session: %d", req.sessionid);
            }

            return true;
        }

        boost::mutex::scoped_lock lock(_mutexSession);
        if( req.requestaccess == Access_ForceControl ) {
            FOREACH(itsession, _mapsessions) {
                if( itsession->second->_access == Access_ForceControl )
                    itsession->second->_access = Access_Read;
            }
            req.requestaccess = Access_Control;
        }
        else if( req.requestaccess == Access_Control ) {
            FOREACH(itsession, _mapsessions)
                if( itsession->second->_access == Access_Control )
                    return false;
        }
 
        int id = rand();
        while(id == 0 || _mapsessions.find(id) != _mapsessions.end())
            id = rand();

        _mapsessions[id].reset(new SessionState((AccessType)req.requestaccess));
        res.sessionid = id;

        ROS_INFO_STREAM(str(boost::format("started openrave session: %d, total: %d")%id%_mapsessions.size()));
        return true;
    }
    
    virtual bool Query(openrave_robot_control::Query::Request& req, openrave_robot_control::Query::Response& res)
    {
        boost::shared_ptr<SessionState> psession = _getState(req, false);
        if( !psession )
            return false;

        res.jointnames.resize(GetDOF());
        res.jointpositions.resize(GetDOF());
        {
            boost::mutex::scoped_lock lock(_mutexControl);
            for(int i = 0; i < GetDOF(); ++i) {
                res.jointnames[i] = _vActiveJointNames[i];
                res.jointpositions[i] = _mstate.position[i];
            }
        }

        boost::mutex::scoped_lock lock(_mutexCommands);
        res.state = _state;
        res.braked = _bBraked;
        if( req.commandid ) {
            // search for current commands
            FOREACH(itcmd, _listCommands) {
                if( (*itcmd)->getCommandId() == req.commandid ) {
                    res.commandid = req.commandid;
                    res.commandtime = (*itcmd)->getCommandTime();
                    break;
                }
            }

            if( res.commandid == 0 ) { // couldn't find, search finished commands
                map<int,float>::iterator it = _mapFinishedCommands.find(req.commandid);
                if( it != _mapFinishedCommands.end() ) {
                    res.commanddone = 1;
                    res.commandid = req.commandid;
                    res.commandtime = it->second;
                }
            }
        }
        else if( _listCommands.size() > 0 ) {
            res.commandid = _listCommands.front()->getCommandId();
            res.commandtime = _listCommands.front()->getCommandTime();
        }

        return true;
    }

    virtual bool Wait(openrave_robot_control::Wait::Request& req, openrave_robot_control::Wait::Response& res)
    {
        boost::shared_ptr<SessionState> psession = _getState(req, false);
        if( !psession )
            return false;

        ros::Time starttime = ros::Time::now();
        res.timedout = 0;

        boost::mutex::scoped_lock lock(_mutexCommands);
        do {
            if( req.commandid ) {
                if( _mapFinishedCommands.find(req.commandid) != _mapFinishedCommands.end() )
                    return true;
            }
            else {
                if( _listCommands.size() == 0 )
                    return true;
            }
            _conditionCommand.wait(_mutexCommands);
        } while( !_bShutdown && (req.timeout == 0 || (ros::Time::now()-starttime).toSec() < req.timeout ) );
        res.timedout = 1;
        return true;
    }

    virtual bool Cancel(openrave_robot_control::Cancel::Request& req, openrave_robot_control::Cancel::Response& res)
    {
        boost::shared_ptr<SessionState> psession = _getState(req, true);
        if( !psession )
            return false;

        if( req.commandid ) {
            boost::mutex::scoped_lock lock(_mutexCommands);
            FOREACH(it, _listCommands) {
                if( (*it)->getCommandId() == req.commandid ) {
                    _listCommands.erase(it);
                    _conditionCommand.notify_all();
                    if( _listCommands.size() == 0 )
                        _setIdle();
                    return true;
                }
            }
            return false;
        }
        else
            _clearCommands();
        return true;
    }

    /// anyone should be able to brake/pause the robot
    virtual bool Brake(openrave_robot_control::Brake::Request& req, openrave_robot_control::Brake::Response& res)
    {
        boost::shared_ptr<SessionState> psession = _getState(req, false);
        if( !psession )
            return false;

        _bBraked = !!req.brake;
        return false;
    }

    virtual bool StartTrajectory(openrave_robot_control::StartTrajectory::Request& req, openrave_robot_control::StartTrajectory::Response& res)
    {
        boost::shared_ptr<SessionState> psession = _getState(req, true);
        if( !psession ) {
            ROS_WARN("bad session");
            return false;
        }

        if( !_probot && !req.hastiming ) {
            ROS_ERROR("OpenRAVE robot is invalid and trajectory doesn't have time stamps, ignoring command...");
            return false;
        }

        TrajectoryBasePtr ptraj = _penv->CreateTrajectory(GetDOF());

        vector<dReal> vpoints(GetDOF());
        FOREACH(it, req.traj.points) {
            if( it->positions.size() != vpoints.size() ) {
                ROS_ERROR("received invalid point");
                return false;
            }
            std::copy(it->positions.begin(),it->positions.end(),vpoints.begin());
            Trajectory::TPOINT tp(vpoints,it->time);
            if( it->torques.size() == vpoints.size() ) {
                tp.qtorque.resize(it->torques.size());
                std::copy(it->torques.begin(),it->torques.end(),tp.qtorque.begin());
            }
            ptraj->AddPoint(tp);
        }

        {
            EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
            if( !ptraj->CalcTrajTiming(_probot, (Trajectory::InterpEnum)req.interpolation, !req.hastiming, true, _fMaxVelMult) ) {
                ROS_ERROR("failed to compute trajectory timing");
                return false;
            }
            ROS_DEBUG_STREAM(str(boost::format("trajectory %fs, timing=%d, maxvel=%f")%ptraj->GetTotalDuration()%((int)req.hastiming)%_fMaxVelMult));
        }

        if( req.requesttiming ) {
            res.timestamps.reserve(ptraj->GetPoints().size());
            FOREACH(itp, ptraj->GetPoints())
                res.timestamps.push_back(itp->time);
        }

        {
            boost::shared_ptr<Command>
                cmd(CreateBindCommand(_GetNewCommandId()|Command_Trajectory,
                                      boost::bind(&OpenRAVEController::_startTrajectoryCommand,this, ptraj),
                                      boost::bind(&OpenRAVEController::_runTrajectoryCommand, this, ptraj,_1),
                                      boost::bind(&OpenRAVEController::_finishTrajectoryCommand,this, ptraj)));
            ROS_DEBUG("adding trajectory command 0x%x", cmd->getCommandId());
            boost::mutex::scoped_lock lock(_mutexCommands);
            res.commandid = cmd->getCommandId();
            _listCommands.push_back(cmd);
            _state = State_Moving;
        }

        res.stamp = ros::Time::now();
        return true;
    }

    virtual bool StartVelocity(openrave_robot_control::StartVelocity::Request& req, openrave_robot_control::StartVelocity::Response& res)
    {
        boost::shared_ptr<SessionState> psession = _getState(req, true);
        if( !psession ) {
            ROS_WARN("bad session");
            return false;
        }
        if( (int)req.velocities.size() != GetDOF() ) {
            ROS_ERROR("velocities not correct size");
            return false;
        }

        vector<dReal> vvelocities(req.velocities.size());
        for(size_t i = 0; i < vvelocities.size(); ++i)
            vvelocities[i] = req.velocities[i];
        
        boost::shared_ptr<Command>
            cmd(CreateBindCommand(_GetNewCommandId()|Command_Velocity,
                                  boost::bind(&OpenRAVEController::_startVelocityCommand,this, vvelocities),
                                  boost::bind(&OpenRAVEController::_runVelocityCommand, this, vvelocities,_1),
                                  boost::bind(&OpenRAVEController::_finishVelocityCommand,this)));
        ROS_DEBUG("adding velocity command 0x%x", cmd->getCommandId());
        boost::mutex::scoped_lock lock(_mutexCommands);
        res.commandid = cmd->getCommandId();
        _listCommands.push_back(cmd);
        _state = State_Moving;

        res.stamp = ros::Time::now();
        return true;
    }

    virtual bool StartTorque(openrave_robot_control::StartTorque::Request& req, openrave_robot_control::StartTorque::Response& res)
    {
        boost::shared_ptr<SessionState> psession = _getState(req, true);
        if( !psession ) {
            ROS_WARN("bad session");
            return false;
        }
        if( (int)req.torques.size() != GetDOF() ) {
            ROS_ERROR("torques not correct size");
            return false;
        }

        vector<dReal> vtorques(req.torques.size());
        for(size_t i = 0; i < vtorques.size(); ++i)
            vtorques[i] = req.torques[i];
        
        boost::shared_ptr<Command>
            cmd(CreateBindCommand(_GetNewCommandId()|Command_Torque,
                                  boost::bind(&OpenRAVEController::_startTorqueCommand,this, vtorques),
                                  boost::bind(&OpenRAVEController::_runTorqueCommand, this, vtorques,_1),
                                  boost::bind(&OpenRAVEController::_finishTorqueCommand,this)));
        ROS_DEBUG("adding torque command 0x%x", cmd->getCommandId());
        boost::mutex::scoped_lock lock(_mutexCommands);
        res.commandid = cmd->getCommandId();
        _listCommands.push_back(cmd);
        _state = State_Moving;

        res.stamp = ros::Time::now();
        return true;
    }

    virtual bool StartCustomString(openrave_robot_control::StartCustomString::Request& req, openrave_robot_control::StartCustomString::Response& res)
    {
        boost::shared_ptr<SessionState> psession = _getState(req, true);
        if( !psession ) {
            ROS_WARN("bad session");
            return false;
        }

        boost::shared_ptr<Command>
            cmd(CreateBindCommand(_GetNewCommandId()|Command_String,
                                  boost::bind(&OpenRAVEController::_startCustomStringCommand,this, req.input),
                                  boost::bind(&OpenRAVEController::_runCustomStringCommand, this, req.input,_1),
                                  boost::bind(&OpenRAVEController::_finishCustomStringCommand,this)));
        ROS_DEBUG("adding torque command 0x%x", cmd->getCommandId());

        boost::mutex::scoped_lock lock(_mutexCommands);
        res.commandid = cmd->getCommandId();
        _listCommands.push_back(cmd);
        _state = State_Moving;

        res.stamp = ros::Time::now();
        return true;
    }

    virtual int GetDOF() { return _vActiveJointNames.size(); }
    virtual EnvironmentBasePtr GetEnv() { return _penv; }

protected:
    template <class MReq>
    boost::shared_ptr<SessionState> _getState(const MReq& req, bool bRequestControl)
    {
        if( !req.__connection_header )
            return boost::shared_ptr<SessionState>();

        ros::M_string::const_iterator it = req.__connection_header->find(_sessionname);
        if( it == req.__connection_header->end() )
            return boost::shared_ptr<SessionState>();

        boost::mutex::scoped_lock lock(_mutexSession);
        int sessionid = atoi(it->second.c_str());
        if( _mapsessions.find(sessionid) == _mapsessions.end() )
            return boost::shared_ptr<SessionState>();

        boost::shared_ptr<SessionState> psession = _mapsessions[sessionid];
        if( bRequestControl && psession->_access != Access_Control ) {
            // check if there's any controlling sessions, if not, grant access
            FOREACH(itsession, _mapsessions) {
                if( itsession->second->_access == Access_Control )
                    return boost::shared_ptr<SessionState>();
            }
            psession->_access = Access_Control;
        }
        return psession;
    }
    
    virtual void _InitEnvironment()
    {
        _penv = CreateEnvironment(true);
        if( !_penv )
            throw controller_exception("failed to initialize environment");
    }

    virtual bool _InitRobot(const string& robotfile, const string& manipname)
    {
        BOOST_ASSERT(!_probot && !_penv);
        _vlinknames.resize(0); _vPublishableLinks.resize(0);
        if( robotfile.size() == 0 )
            return false;

        try {
            // create the main environment
            _InitEnvironment();
            EnvironmentMutex::scoped_lock lock(_penv->GetMutex());
            _probot = _penv->ReadRobotXMLFile(robotfile);
            if( !_penv->AddRobot(_probot) )
                throw controller_exception("failed to add robot");

            ROS_INFO_STREAM(str(boost::format("found openrave robot %s, numjoints=%d")%_probot->GetName()%_probot->GetDOF()));

            if( manipname.size() > 0 ) {
                _vActiveJointNames.resize(0);
                // look for this manipulator
                stringstream ss(manipname);
                string manipname_only, desc;
                ss >> manipname_only >> desc;
                FOREACH(itmanip, _probot->GetManipulators()) {
                    if( (*itmanip)->GetName() == manipname_only ) {
                        if( desc == "gripper" ) {
                            FOREACH(itind, (*itmanip)->GetGripperIndices())
                                _vActiveJointNames.push_back(_probot->GetJointFromDOFIndex(*itind)->GetName());
                        }
                        else { // arm
                            FOREACH(itind, (*itmanip)->GetArmIndices())
                                _vActiveJointNames.push_back(_probot->GetJointFromDOFIndex(*itind)->GetName());
                        }
                        break;
                    }
                }
            }

            if( _vActiveJointNames.size() == 0 )
                throw controller_exception("no joint names specified");
                
            // look for these joints:
            vector<int> vrobotjoints;
            FOREACH(itname, _vActiveJointNames) {
                int index = _probot->GetJointIndex(*itname);
                if( index < -1 ) {
                    ROS_ERROR("could not find robot joint %s", itname->c_str());
                    throw controller_exception("failed to find robot joint");
                }
                vrobotjoints.push_back(index);
            }

            FOREACH(itlink, _probot->GetLinks())
                _vlinknames.push_back((*itlink)->GetName());

            BOOST_ASSERT((int)vrobotjoints.size()==GetDOF());
            _probot->SetActiveDOFs(vrobotjoints);

            // publish link frames
            set<int> setusedlinks;
            FOREACH(it, vrobotjoints) {
                KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);
                setusedlinks.insert(pjoint->GetFirstAttached()->GetIndex());
                setusedlinks.insert(pjoint->GetSecondAttached()->GetIndex());

                if( _probot->DoesAffect(*it, pjoint->GetFirstAttached()->GetIndex()) )
                    // first link is child link
                    _vPublishableLinks.push_back(make_pair(pjoint->GetSecondAttached()->GetIndex(), pjoint->GetFirstAttached()->GetIndex()));
                else
                    _vPublishableLinks.push_back(make_pair(pjoint->GetFirstAttached()->GetIndex(),pjoint->GetSecondAttached()->GetIndex()));
            }

            // check the passive joints!
            FOREACH(itpassive, _probot->GetPassiveJoints()) {
                if( (*itpassive)->IsStatic() ) {
                    if( !!(*itpassive)->GetFirstAttached() && !!(*itpassive)->GetSecondAttached() ) {
                        setusedlinks.insert((*itpassive)->GetFirstAttached()->GetIndex());
                        setusedlinks.insert((*itpassive)->GetSecondAttached()->GetIndex());
                        bool bFirstLinkIsChild = (*itpassive)->GetFirstAttached()->GetIndex() > (*itpassive)->GetSecondAttached()->GetIndex();
                        if( (*itpassive)->GetFirstAttached() == _probot->GetLinks().at(0) )
                            bFirstLinkIsChild = false;
                        else if( (*itpassive)->GetSecondAttached() == _probot->GetLinks().at(0) )
                            bFirstLinkIsChild = true;
                        else {
                            FOREACHC(it,_probot->GetDependencyOrderedJoints()) {
                                if( (*it)->GetFirstAttached() == (*itpassive)->GetFirstAttached() || (*it)->GetSecondAttached() == (*itpassive)->GetFirstAttached() ) {
                                    bFirstLinkIsChild = false;
                                    break;
                                }
                                else if( (*it)->GetFirstAttached() == (*itpassive)->GetSecondAttached() || (*it)->GetSecondAttached() == (*itpassive)->GetSecondAttached() ) {
                                    bFirstLinkIsChild = true;
                                    break;
                                }
                            }
                        }
                        
                        if( bFirstLinkIsChild )
                            _vPublishableLinks.push_back(make_pair((*itpassive)->GetSecondAttached()->GetIndex(), (*itpassive)->GetFirstAttached()->GetIndex()));
                        else
                            _vPublishableLinks.push_back(make_pair((*itpassive)->GetFirstAttached()->GetIndex(),(*itpassive)->GetSecondAttached()->GetIndex()));
                    }
                }
                if( (*itpassive)->GetMimicJointIndex() >= 0 ) {
                    FOREACH(it,vrobotjoints) {
                        if( (*itpassive)->GetMimicJointIndex() == *it ) {
                            setusedlinks.insert((*itpassive)->GetFirstAttached()->GetIndex());
                            setusedlinks.insert((*itpassive)->GetSecondAttached()->GetIndex());
                            if( _probot->DoesAffect(*it, (*itpassive)->GetFirstAttached()->GetIndex()) )
                                // first link is child link
                                _vPublishableLinks.push_back(make_pair((*itpassive)->GetSecondAttached()->GetIndex(), (*itpassive)->GetFirstAttached()->GetIndex()));
                            else
                                _vPublishableLinks.push_back(make_pair((*itpassive)->GetFirstAttached()->GetIndex(),(*itpassive)->GetSecondAttached()->GetIndex()));
                            break;
                        }
                    }
                }
            }

            // go through all dummy joints and prune more links
            int i = 0;
            while(i < _probot->GetDOF()) {
                if( find(vrobotjoints.begin(), vrobotjoints.end(), i) == vrobotjoints.end() ) {
                    KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(i);
                    bool bDummy = true;
                    if( pjoint->GetDOF() > 0 ) {
                        vector<dReal> vlower, vupper;
                        pjoint->GetLimits(vlower, vupper);
                        for(size_t j=0; j<vlower.size(); ++j) {
                            if( vlower[j] != vupper[j] ) {
                                bDummy = false;
                                break;
                            }
                        }
                    }

                    if( bDummy ) {
                        bool bHasFirst = setusedlinks.find(pjoint->GetFirstAttached()->GetIndex()) != setusedlinks.end();
                        bool bHasSecond = setusedlinks.find(pjoint->GetSecondAttached()->GetIndex()) != setusedlinks.end();
                        if( bHasFirst != bHasSecond ) {

                            if( _probot->DoesAffect(pjoint->GetJointIndex(), pjoint->GetFirstAttached()->GetIndex()) )
                                // first link is child link
                                _vStaticTransforms.push_back(boost::make_tuple(_getBtTransform(pjoint->GetSecondAttached()->GetTransform().inverse() * pjoint->GetFirstAttached()->GetTransform()), _vlinknames[pjoint->GetSecondAttached()->GetIndex()], _vlinknames[pjoint->GetFirstAttached()->GetIndex()]));
                            else
                                _vStaticTransforms.push_back(boost::make_tuple(_getBtTransform(pjoint->GetFirstAttached()->GetTransform().inverse() * pjoint->GetSecondAttached()->GetTransform()), _vlinknames[pjoint->GetFirstAttached()->GetIndex()], _vlinknames[pjoint->GetSecondAttached()->GetIndex()]));
                            setusedlinks.insert(pjoint->GetFirstAttached()->GetIndex());
                            setusedlinks.insert(pjoint->GetSecondAttached()->GetIndex());
                            i = 0;
                            continue;
                        }
                    }
                }

                ++i;
            }

            // publish sensor frames
            FOREACH(itsensor, _probot->GetAttachedSensors()) {
                // only send sensor data if name is valid
                if( (*itsensor)->GetName().size() > 0 ) {
                    ROS_INFO_STREAM(str(boost::format("publishing sensor transform %s")%(*itsensor)->GetName()));
                    _vStaticTransforms.push_back(boost::make_tuple(_getBtTransform((*itsensor)->GetRelativeTransform()),_vlinknames[(*itsensor)->GetAttachingLink()->GetIndex()],(*itsensor)->GetName()));
                }
            }

            // publish manipulator frames
            FOREACH(itmanip, _probot->GetManipulators()) {
                // only send sensor data if name is valid
                if( (*itmanip)->GetName().size() > 0 ) {
                    ROS_INFO_STREAM(str(boost::format("publishing manipulator transform %s")%(*itmanip)->GetName()));
                    _vStaticTransforms.push_back(boost::make_tuple(_getBtTransform((*itmanip)->GetGraspTransform()),_vlinknames[(*itmanip)->GetEndEffector()->GetIndex()],(*itmanip)->GetName()));
                }
            }
        }
        catch(controller_exception& err) {
            ROS_ERROR("failed to open OpenRAVE robot file %s: %s", robotfile.c_str(),err.what());
            if( !!_probot && !!_penv )
                _penv->RemoveKinBody(_probot);
            _probot.reset();
            if( !!_penv )
                _penv->Destroy();
            _penv.reset();
            return false;
        }

        return true;
    }

    virtual void _publishTF()
    {
        if( !_probot )
            return;

        vector<dReal> vjointvalues(GetDOF());
        ros::Time starttime;
        {
            boost::mutex::scoped_lock lock(_mutexControl);
            std::copy(_mstate.position.begin(),_mstate.position.end(),vjointvalues.begin());
            starttime = _mstate.header.stamp;
        }

        vector<Transform> vlinktrans;
        {
            EnvironmentMutex::scoped_lock lock(_penv->GetMutex());
            BOOST_ASSERT((int)vjointvalues.size()==_probot->GetActiveDOF());
            _probot->SetActiveDOFValues(vjointvalues);
            _probot->GetBodyTransformations(vlinktrans);
        }

        // publish link frames
        FOREACH(itpair, _vPublishableLinks) {
            BOOST_ASSERT(_vlinknames[itpair->first] != _vlinknames[itpair->second]);
            _tfbroadcaster->sendTransform(tf::StampedTransform(_getBtTransform(vlinktrans.at(itpair->first).inverse() * vlinktrans.at(itpair->second)), starttime, _vlinknames[itpair->first], _vlinknames[itpair->second]));
        }

        FOREACH(itstatic,_vStaticTransforms)
            _tfbroadcaster->sendTransform(tf::StampedTransform(boost::get<0>(*itstatic), starttime, boost::get<1>(*itstatic), boost::get<2>(*itstatic)));
    }
    virtual void _clearCommands()
    {
        boost::mutex::scoped_lock lock(_mutexCommands);
        _listCommands.clear();
        _conditionCommand.notify_all();
        _setIdle();
    }
    
    virtual void _setIdle()
    {
        _state = State_Idle;
    }

    virtual void _startTrajectoryCommand(TrajectoryBasePtr ptraj) {}
    virtual CommandStatus _runTrajectoryCommand(TrajectoryBasePtr ptraj, float fCommandTime) { return Status_Finished; }
    virtual void _finishTrajectoryCommand(TrajectoryBasePtr ptraj) {}

    virtual void _startVelocityCommand(vector<dReal> velocities) {}
    virtual CommandStatus _runVelocityCommand(vector<dReal> velocities, float fCommandTime) { return Status_Finished; }
    virtual void _finishVelocityCommand() {}

    virtual void _startTorqueCommand(vector<dReal> torques) {}
    virtual CommandStatus _runTorqueCommand(vector<dReal> torques, float fCommandTime) { return Status_Finished; }
    virtual void _finishTorqueCommand() {}

    virtual void _startCustomStringCommand(string command) {}
    virtual CommandStatus _runCustomStringCommand(string command, float fCommandTime) { return Status_Finished; }
    virtual void _finishCustomStringCommand() {}

    virtual uint32_t _GetNewCommandId() {
        uint32_t cmd = _nNextCommandId++;
        if( _nNextCommandId == 0x0fffffff )
            _nNextCommandId = 1;
        return cmd;
    }

    btTransform _getBtTransform(const Transform& t)
    {
        return btTransform(btQuaternion(t.rot.y,t.rot.z,t.rot.w,t.rot.x), btVector3(t.trans.x,t.trans.y,t.trans.z));
    }

    boost::shared_ptr<ros::NodeHandle> _ros;
    ros::Publisher _pubmstate;
    ros::ServiceServer _srvSession, _srvQuery, _srvWait, _srvCancel, _srvBreak, _srvStartTrajectory, _srvStartVelocity, _srvStartTorque, _srvStartCustomString;

    boost::mutex _mutexControl; ///< protects all calls that call into robot-specific control libraries and mechanism state
    boost::mutex _mutexCommands;
    boost::condition _conditionCommand; ///< notifies when a command is complete
    bool _bShutdown;

    string _sessionname;
    boost::mutex _mutexSession;
    map<int,boost::shared_ptr<SessionState> > _mapsessions;

    // robot model/kinematics information
    //@{
    string _robotfile, _manipname;
    EnvironmentBasePtr _penv;
    RobotBasePtr _probot;
    vector<string> _vActiveJointNames, _vlinknames;
    vector<pair<int, int> > _vPublishableLinks; ///< relative transforms of links to publish with tf, first link is child, second is parent
    vector<boost::tuple<btTransform,string,string> > _vStaticTransforms; ///< relative transforms of links to publish with tf, first link is child, second is parent
    //@}

    dReal _fMaxVelMult; ///< multiply by max speeds

    boost::shared_ptr<tf::TransformBroadcaster> _tfbroadcaster;

    // protected by _mutexCommands
    //@{
    list<boost::shared_ptr<Command> > _listCommands;
    map<int,float> _mapFinishedCommands;
    StateType _state; ///< command state of the robot
    bool _bBraked; ///< robot paused, brakes are on or e-stopped
    uint32_t _nNextCommandId;
    //@}

    // protected by _mutexControl
    //@{
    sensor_msgs::JointState _mstate;
    //@}
};

BOOST_STATIC_ASSERT(Trajectory::NONE==openrave_robot_control::StartTrajectory::Request::Interp_None);
BOOST_STATIC_ASSERT(Trajectory::LINEAR==openrave_robot_control::StartTrajectory::Request::Interp_Linear);
BOOST_STATIC_ASSERT(Trajectory::LINEAR_BLEND==openrave_robot_control::StartTrajectory::Request::Interp_LinearBlend);
BOOST_STATIC_ASSERT(Trajectory::CUBIC==openrave_robot_control::StartTrajectory::Request::Interp_Cubic);
BOOST_STATIC_ASSERT(Trajectory::QUINTIC==openrave_robot_control::StartTrajectory::Request::Interp_Quintic);

} // end namespace openrave_robot_control
