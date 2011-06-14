// Copyright (c) 2009 Rosen Diankov
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

#include <openrave_robot_control/openravecontroller.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <exception>
#include <sstream>

using namespace std;
using namespace OpenRAVE;
using namespace openrave_robot_control;

class SimulationController : public OpenRAVEController
{
protected:
    enum ArmStateType
    {
        Arm_None=0,
        Arm_Suspended=1,
        Arm_RealTimePosition=2,
        Arm_RealTimeVelocity=3,
        Arm_GravityCompensation=4
    };

public:
    
    SimulationController(const string& robotfile, const string& manipname, const vector<string>& vjointnames, float fMaxVelMult) : OpenRAVEController(robotfile, manipname, vjointnames, fMaxVelMult), _armstate(Arm_Suspended) {
    }

    virtual ~SimulationController() {
        _bShutdown = true; _threadControl.join();
    }

    virtual void init()
    {
        if( !!_probot )
            throw openrave_exception("robot is not initialized");
        OpenRAVEController::init();
        _threadControl = boost::thread(boost::bind(&SimulationController::_ControlThread,this));
    }

    virtual void shutdown()
    {
        _bShutdown = true;
        _threadControl.join();
        OpenRAVEController::shutdown();
    }

private:
    virtual void _ControlThread()
    {
        ros::Duration controltime(0.01); // 10ms
        vector<dReal> vnewvalues(GetDOF()), vprevvalues(GetDOF());

        while(!_bShutdown) {
            ros::Time starttime = ros::Time::now();

            try {
                {
                    boost::mutex::scoped_lock lock(_mutexControl);
                    EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
                    _probot->GetActiveDOFValues(vnewvalues);
                    for(size_t i = 0; i < vnewvalues.size(); ++i) {
                        _mstate.position.at(i) = vnewvalues.at(i);
                        // rough approximation of velocity
                        _mstate.velocity.at(i) = (vnewvalues.at(i)-vprevvalues.at(i))/controltime.toSec();
                    }
                    vprevvalues = vnewvalues;
                    _mstate.header.stamp = starttime;
                }
            
                _pubmstate.publish(_mstate);
                _publishTF();
            
                // process commands
                ArmStateType newstate = Arm_None;
                {
                    boost::mutex::scoped_lock lock(_mutexCommands);
                    EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());

                    if( _listCommands.size() > 0 ) {
                        if( _listCommands.front()->run() == Status_Finished ) {
                            ROS_DEBUG("command 0x%x finished", _listCommands.front()->getCommandId());
                            _mapFinishedCommands[_listCommands.front()->getCommandId()] = _listCommands.front()->getCommandTime();
                            _listCommands.pop_front();
                            if( _listCommands.size() == 0 )
                                _state = State_Ready;
                            _conditionCommand.notify_all();
                        }
                    }
                    
                    if( _listCommands.size() == 0 ) {
                        boost::mutex::scoped_lock lock(_mutexControl);
                        if( _state == State_Idle )
                            newstate = Arm_GravityCompensation;
                        else
                            newstate = Arm_Suspended;
                    }
                }

                _setstate(newstate);
            }
            catch(const openrave_exception& err) {
                // reset state to idle and try to reconnect
                ROS_WARN("resetting and setting to idle: %s",err.what());
                boost::mutex::scoped_lock lockcontrol(_mutexControl);
                _clearCommands();
                continue;
            }

            ros::Duration processingtime = ros::Time::now()-starttime;
            if( processingtime < controltime ) {
                usleep((controltime-processingtime).toNSec()/1000);
            }
        }

        _clearCommands();
    }

    virtual void _startTrajectoryCommand(TrajectoryBasePtr ptraj)
    {
        vector<dReal> vcurrentangles(GetDOF());
        {
            boost::mutex::scoped_lock lock(_mutexControl);
            std::copy(_mstate.position.begin(),_mstate.position.end(),vcurrentangles.begin());
        }

        bool bDifferent = false;
        vector<dReal>& vstartangles = ptraj->GetPoints().front().q;
        for(int i = 0; i < GetDOF(); ++i) {
            if( RaveFabs(vcurrentangles[i]-vstartangles[i]) > 0.01f ) {
                bDifferent = true;
                break;
            }
        }

        if( bDifferent ) {
            // have to add the current angles in the beginning, create a new trajectory
            EnvironmentMutex::scoped_lock lock(_penv->GetMutex());
            TrajectoryBasePtr ptesttraj = RaveCreateTrajectory(_penv,GetDOF());
            ptesttraj->AddPoint(Trajectory::TPOINT(vcurrentangles, 0));
            ptesttraj->AddPoint(Trajectory::TPOINT(vstartangles, 0));
            ptesttraj->CalcTrajTiming(_probot, ptraj->GetInterpMethod(), true, true, _fMaxVelMult);
            dReal fTimeStampOffset = ptesttraj->GetTotalDuration();
            
            //ROS_DEBUG("adding current angles to beginning of trajectory, offset=%f", fTimeStampOffset);

            FOREACH(itp, ptraj->GetPoints())
                itp->time += fTimeStampOffset;
            Trajectory::TPOINT startpoint(vcurrentangles,0); startpoint.qdot.resize(GetDOF(),0);
            ptraj->GetPoints().insert(ptraj->GetPoints().begin(), startpoint); // add to front
            ptraj->CalcTrajTiming(_probot, ptraj->GetInterpMethod(), false, true, _fMaxVelMult);
        }

        boost::mutex::scoped_lock lock(_mutexControl);
        _setstate(Arm_RealTimePosition);
        _state = State_Moving;
    }

    virtual CommandStatus _runTrajectoryCommand(TrajectoryBasePtr ptraj, float fCommandTime)
    {
        boost::mutex::scoped_lock lock(_mutexControl);
        vector<dReal> vcurrentangles(GetDOF());
        std::copy(_mstate.position.begin(),_mstate.position.end(),vcurrentangles.begin());

        Trajectory::TPOINT pt;
        ptraj->SampleTrajectory(fCommandTime, pt);
        _probot->SetActiveDOFValues(pt.q);
        //ROS_DEBUG("command time: %f/%f: %d",fCommandTime,ptraj->GetTotalDuration(),(int)ptraj->GetPoints().size());
        bool bDifferent = false;
        for(int i = 0; i < GetDOF(); ++i) {
            if( RaveFabs(vcurrentangles[i]-pt.q[i]) > 0.01f ) {
                bDifferent = true;
                break;
            }
        }
        
        return (fCommandTime >= ptraj->GetTotalDuration()&&!bDifferent) ? Status_Finished : Status_Running;
    }

    virtual void _finishTrajectoryCommand(TrajectoryBasePtr ptraj)
    {
    }

    virtual void _startVelocityCommand(vector<dReal> velocities)
    {
        boost::mutex::scoped_lock lock(_mutexControl);
        _setstate(Arm_RealTimeVelocity);
        _state = State_Moving;
    }

    virtual CommandStatus _runVelocityCommand(vector<dReal> velocities, float fCommandTime)
    {
        boost::mutex::scoped_lock lock(_mutexControl);
        return Status_Running;
    }

    virtual void _finishVelocityCommand()
    {
    }

    virtual void _startCustomStringCommand(string cmd)
    {
        boost::mutex::scoped_lock lock(_mutexControl);
        _state = State_Moving;
        ROS_DEBUG("custom command: %s",cmd.c_str());
    }

    virtual CommandStatus _runCustomStringCommand(string cmd, float fCommandTime)
    {
        return Status_Finished;
    }

    virtual void _finishCustomStringCommand()
    {
    }

    void _setstate(ArmStateType armstate)
    {
        _armstate = armstate;
    }

    boost::thread _threadControl;
    // protected by _mutexControl
    //@{
    ArmStateType _armstate;
    //@}
};

#include <signal.h>
void sigint_handler(int);
boost::shared_ptr<SimulationController> s_pcontroller;

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    if( !viewer )
        return;

    // attach it to the environment:
    penv->AddViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);
}

int main(int argc, char ** argv)
{
    signal(SIGINT,sigint_handler); // control C
        
    dReal fMaxVelMult = 1;

    // parse the command line options
    vector<string> vjointnames;
    string robotname, manipname, viewername;
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0 ) {
            // print the arguments and exit
            printf("robotlinks_filter_node [--robotfile openravefile] [--manipname manipulator name] [--maxvelmult multiplier]\n"
                   "  Start a node to control the PA10 arm and publish ROS interfaces.\n"
                   "  Currently the robot file specified has to be in OpenRAVE format\n");
            return 0;
        }
        if( strcmp(argv[i], "--robotfile") == 0 ) {
           robotname = argv[i+1];
            i += 2;
        }
        else if( strcmp(argv[i], "--manipgripper") == 0 ) {
            manipname = string(argv[i+1]) + string(" gripper");
            i += 2;
        }
        else if( strcmp(argv[i], "--manipname") == 0 || strcmp(argv[i], "--maniparm") == 0 ) {
            manipname = string(argv[i+1]) + string(" arm");
            i += 2;
        }
        else if( strcmp(argv[i], "--maxvelmult") == 0 ) {
            fMaxVelMult = atof(argv[i+1]);
            i += 2;
        }
        else if( strcmp(argv[i], "--jointname") == 0 ) {
            vjointnames.push_back(argv[i+1]);
            i += 2;
        }
        else if( strcmp(argv[i], "--viewer") == 0 ) {
            viewername = argv[i+1];
            i += 2;
        }
        else
            break;
    }

    ros::init(argc,argv,"simulationserver", ros::init_options::NoSigintHandler);
    if( !ros::master::check() )
        return 1;
    
    RaveInitialize(true);
    s_pcontroller.reset(new SimulationController(robotname, manipname, vjointnames, fMaxVelMult));
    s_pcontroller->init();

    boost::shared_ptr<boost::thread> thviewer;
    if( viewername.size() > 0 )
        thviewer.reset(new boost::thread(boost::bind(SetViewer,s_pcontroller->GetEnv(),viewername)));
    ros::spin();
        
    s_pcontroller.reset();
    if( !!thviewer )
        thviewer->join();
    return 0;
}

void sigint_handler(int)
{
    s_pcontroller->shutdown();
    ros::shutdown();
}
