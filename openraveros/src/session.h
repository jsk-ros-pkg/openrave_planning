// Software License Agreement (BSD License)
// Copyright (c) 2008, Rosen Diankov (rdiankov@cs.cmu.edu)
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

/**
@mainpage

@htmlinclude manifest.html

\author Rosen Diankov (rdiankov@cs.cmu.edu)

@b SessionServer creates and manages a roscpp session that wraps various OpenRAVE services.
 **/

#include "openraveros.h"
#include "rosserver.h"

#include <openraveros/openrave_session.h>

using namespace ros;

#define REFLECT_SERVICE(srvname) \
    bool srvname##_srv(srvname::Request& req, srvname::Response& res) \
    { \
        SessionState state = getstate(req); /* need separate copy in order to guarantee thread safety */ \
        if( !state._pserver ) { \
            RAVELOG_INFOA("failed to find session for service %s\n", #srvname); \
            return false; \
        } \
        return state._pserver->srvname##_srv(req,res); \
    }

class SessionServer : public boost::enable_shared_from_this<SessionServer>
{
    class SessionState
    {
    public:
        SessionState() {}
        SessionState(boost::shared_ptr<SessionServer> psessionserver) : _psessionserver(psessionserver) {}
        virtual ~SessionState() {
            _pserver.reset();
            _penv.reset();
        }

        boost::shared_ptr<SessionServer> _psessionserver;
        boost::shared_ptr<ROSServer> _pserver;
        EnvironmentBasePtr _penv;
    };

    string _sessionname;
public:
    SessionServer() : _ok(true) {
        _pParentEnvironment = RaveCreateEnvironment();
    }
    virtual ~SessionServer() {
        Destroy();
    }

    bool Init()
    {
        _ros.reset(new ros::NodeHandle());
        
        _srvSession = _ros->advertiseService("openrave_session",&SessionServer::session_callback,shared_from_this());
        _sessionname = _ros->resolveName("openrave_session");

        // advertise persistent services
        _mapservices["body_destroy"] = _ros->advertiseService("body_destroy",&SessionServer::body_destroy_srv,shared_from_this());
        _mapservices["body_enable"] = _ros->advertiseService("body_enable",&SessionServer::body_enable_srv,shared_from_this());
        _mapservices["body_getaabb"] = _ros->advertiseService("body_getaabb",&SessionServer::body_getaabb_srv,shared_from_this());
        _mapservices["body_getaabbs"] = _ros->advertiseService("body_getaabbs",&SessionServer::body_getaabbs_srv,shared_from_this());
        _mapservices["body_getdof"] = _ros->advertiseService("body_getdof",&SessionServer::body_getdof_srv,shared_from_this());
        _mapservices["body_getjointvalues"] = _ros->advertiseService("body_getjointvalues",&SessionServer::body_getjointvalues_srv,shared_from_this());
        _mapservices["body_setjointvalues"] = _ros->advertiseService("body_setjointvalues",&SessionServer::body_setjointvalues_srv,shared_from_this());
        _mapservices["body_settransform"] = _ros->advertiseService("body_settransform",&SessionServer::body_settransform_srv,shared_from_this());
        _mapservices["env_checkcollision"] = _ros->advertiseService("env_checkcollision",&SessionServer::env_checkcollision_srv,shared_from_this());
        _mapservices["env_closefigures"] = _ros->advertiseService("env_closefigures",&SessionServer::env_closefigures_srv,shared_from_this());
        _mapservices["env_createbody"] = _ros->advertiseService("env_createbody",&SessionServer::env_createbody_srv,shared_from_this());
        _mapservices["env_createplanner"] = _ros->advertiseService("env_createplanner",&SessionServer::env_createplanner_srv,shared_from_this());
        _mapservices["env_createproblem"] = _ros->advertiseService("env_createproblem",&SessionServer::env_createproblem_srv,shared_from_this());
        _mapservices["env_createrobot"] = _ros->advertiseService("env_createrobot",&SessionServer::env_createrobot_srv,shared_from_this());
        _mapservices["env_destroyproblem"] = _ros->advertiseService("env_destroyproblem",&SessionServer::env_destroyproblem_srv,shared_from_this());
        _mapservices["env_getbodies"] = _ros->advertiseService("env_getbodies",&SessionServer::env_getbodies_srv,shared_from_this());
        _mapservices["env_getbody"] = _ros->advertiseService("env_getbody",&SessionServer::env_getbody_srv,shared_from_this());
        _mapservices["env_getrobots"] = _ros->advertiseService("env_getrobots",&SessionServer::env_getrobots_srv,shared_from_this());
        _mapservices["env_loadplugin"] = _ros->advertiseService("env_loadplugin",&SessionServer::env_loadplugin_srv,shared_from_this());
        _mapservices["env_loadscene"] = _ros->advertiseService("env_loadscene",&SessionServer::env_loadscene_srv,shared_from_this());
        _mapservices["env_plot"] = _ros->advertiseService("env_plot",&SessionServer::env_plot_srv,shared_from_this());
        _mapservices["env_raycollision"] = _ros->advertiseService("env_raycollision",&SessionServer::env_raycollision_srv,shared_from_this());
        _mapservices["env_set"] = _ros->advertiseService("env_set",&SessionServer::env_set_srv,shared_from_this());
        _mapservices["env_triangulate"] = _ros->advertiseService("env_triangulate",&SessionServer::env_triangulate_srv,shared_from_this());
        _mapservices["env_wait"] = _ros->advertiseService("env_wait",&SessionServer::env_wait_srv,shared_from_this());
        _mapservices["planner_init"] = _ros->advertiseService("planner_init",&SessionServer::planner_init_srv,shared_from_this());
        _mapservices["planner_plan"] = _ros->advertiseService("planner_plan",&SessionServer::planner_plan_srv,shared_from_this());
        _mapservices["problem_sendcommand"] = _ros->advertiseService("problem_sendcommand",&SessionServer::problem_sendcommand_srv,shared_from_this());
        _mapservices["robot_controllersend"] = _ros->advertiseService("robot_controllersend",&SessionServer::robot_controllersend_srv,shared_from_this());
        _mapservices["robot_controllerset"] = _ros->advertiseService("robot_controllerset",&SessionServer::robot_controllerset_srv,shared_from_this());
        _mapservices["robot_getactivevalues"] = _ros->advertiseService("robot_getactivevalues",&SessionServer::robot_getactivevalues_srv,shared_from_this());
        _mapservices["robot_sensorgetdata"] = _ros->advertiseService("robot_sensorgetdata",&SessionServer::robot_sensorgetdata_srv,shared_from_this());
        _mapservices["robot_sensorsend"] = _ros->advertiseService("robot_sensorsend",&SessionServer::robot_sensorsend_srv,shared_from_this());
        _mapservices["robot_setactivedofs"] = _ros->advertiseService("robot_setactivedofs",&SessionServer::robot_setactivedofs_srv,shared_from_this());
        _mapservices["robot_setactivevalues"] = _ros->advertiseService("robot_setactivevalues",&SessionServer::robot_setactivevalues_srv,shared_from_this());
        _mapservices["robot_starttrajectory"] = _ros->advertiseService("robot_starttrajectory",&SessionServer::robot_starttrajectory_srv,shared_from_this());

        _ok = true;
        _threadviewer = boost::thread(boost::bind(&SessionServer::ViewerThread, this));
        return true;
    }

    void Destroy()
    {
        shutdown();
        _srvSession.shutdown();
        _mapservices.clear();
        _ros.reset();
        _threadviewer.join();
    }
    
    virtual void shutdown()
    {
        _ok = false;
        boost::mutex::scoped_lock lockcreate(_mutexViewer);
        if( !!_penvViewer ) {
            if( !!_pviewer ) {
                _penvViewer->Remove(_pviewer);
            }
        }
    }

    bool SetViewer(EnvironmentBasePtr penv, const string& viewer, const string& title)
    {
        boost::mutex::scoped_lock lock(_mutexViewer);
        
        // destroy the old viewer
        if( !!_penvViewer ) {
            if( !!_pviewer ) {
                _penvViewer->Remove(_pviewer);
            }
            _conditionViewer.wait(lock);
        }
         
        ROS_ASSERT(!_penvViewer);

        _strviewertitle = title;
        _strviewer = viewer;
        if( viewer.size() == 0 || !penv )
            return false;
            
        _penvViewer = penv;
        _conditionViewer.wait(lock);
        return !!_pviewer;
    }

    EnvironmentBasePtr GetParentEnvironment() const { return _pParentEnvironment; }

protected:
    // ross
    boost::shared_ptr<ros::NodeHandle> _ros;
    ros::ServiceServer _srvSession;
    map<string,ros::ServiceServer> _mapservices;

    map<int,SessionState> _mapsessions;
    boost::mutex _mutexsession;
    EnvironmentBasePtr _pParentEnvironment;

    // persistent viewer
    ViewerBasePtr _pviewer;
    boost::thread _threadviewer; ///< persistent thread (qtcoin openrave plugin needs this)
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;
    EnvironmentBasePtr _penvViewer;
    string _strviewer, _strviewertitle;

    bool _ok;

    virtual void ViewerThread()
    {
        while(_ok) {
            
            {
                usleep(1000);
                boost::mutex::scoped_lock lockcreate(_mutexViewer);
                if( _strviewer.size() == 0 || !_penvViewer ) {                    
                    continue;
                }

                _pviewer = RaveCreateViewer(_penvViewer,_strviewer);
                if( !!_pviewer ) {
                    _penvViewer->AddViewer(_pviewer);
                    _pviewer->ViewerSetSize(1024,768);
                }

                if( !_pviewer )
                    _penvViewer.reset();

                _conditionViewer.notify_all();

                if( !_pviewer )
                    continue;
            }

            if( _strviewertitle.size() > 0 )
                _pviewer->ViewerSetTitle(_strviewertitle);

            _pviewer->main(); // spin until quitfrommainloop is called
            
            boost::mutex::scoped_lock lockcreate(_mutexViewer);
            RAVELOG_DEBUGA("destroying viewer\n");
            ROS_ASSERT(_penvViewer == _pviewer->GetEnv());
            if( !!_pviewer ) {
                _penvViewer->Remove(_pviewer);
            }
            _pviewer.reset();
            usleep(100000); // wait a little
            _penvViewer.reset();
            _conditionViewer.notify_all();
        }
    }

    template <class MReq>
    SessionState getstate(const MReq& req)
    {
        if( !req.__connection_header )
            return SessionState(shared_from_this());

        ros::M_string::const_iterator it = req.__connection_header->find(_sessionname);
        if( it == req.__connection_header->end() )
            return SessionState(shared_from_this());

        boost::mutex::scoped_lock lock(_mutexsession);
        
        int sessionid = 0;
        stringstream ss(it->second); ss >> sessionid;
        if( _mapsessions.find(sessionid) == _mapsessions.end() )
            return SessionState(shared_from_this());
        return _mapsessions[sessionid];
    }

    bool session_callback(openrave_session::Request& req, openrave_session::Response& res)
    {
        if( req.sessionid != 0 ) {
            boost::mutex::scoped_lock lock(_mutexsession);

            if( req.sessionid == -1 ) {
                // destroy all sessions
                RAVELOG_DEBUGA("destroying all sessions\n");
                _mapsessions.clear();
            }
            else {
                // destory the session
                if( _mapsessions.find(req.sessionid) == _mapsessions.end() )
                    return false;
                
                _mapsessions.erase(req.sessionid);
                RAVELOG_INFOA("destroyed openrave session: %d\n", req.sessionid);
            }

            return true;
        }

        SessionState state(shared_from_this());
        int id = rand();
        {
            boost::mutex::scoped_lock lock(_mutexsession);
            while(id == 0 || _mapsessions.find(id) != _mapsessions.end())
                id = rand();
            _mapsessions[id] = state; // grab it
        }
        
        if( req.clone_sessionid ) {
            // clone the environment from clone_sessionid
            SessionState clonestate(shared_from_this());
            {
                boost::mutex::scoped_lock lock(_mutexsession);
                clonestate = _mapsessions[req.clone_sessionid];
            }

            if( !clonestate._penv )
                RAVELOG_INFOA("failed to find session %d\n", req.clone_sessionid);
            else 
                state._penv = clonestate._penv->CloneSelf(req.clone_options);
        }

        if( !state._penv ) {
            // cloning from parent
            RAVELOG_DEBUGA("cloning from parent\n");
            state._penv = _pParentEnvironment->CloneSelf(0);
        }

        state._pserver.reset(new ROSServer(id, boost::bind(&SessionServer::SetViewer,shared_from_this(),state._penv,_1,_2), state._penv, req.physicsengine, req.collisionchecker, req.viewer));
        state._penv->LoadProblem(state._pserver,"");

        boost::mutex::scoped_lock lock(_mutexsession);
        _mapsessions[id] = state;
        res.sessionid = id;

        RAVELOG_INFOA("started openrave session: %d, total: %d\n", id, _mapsessions.size());
        return true;
    }

    REFLECT_SERVICE(body_destroy)
    REFLECT_SERVICE(body_enable)
    REFLECT_SERVICE(body_getaabb)
    REFLECT_SERVICE(body_getaabbs)
    REFLECT_SERVICE(body_getdof)
    REFLECT_SERVICE(body_getjointvalues)
    REFLECT_SERVICE(body_setjointvalues)
    REFLECT_SERVICE(body_settransform)
    REFLECT_SERVICE(env_checkcollision)
    REFLECT_SERVICE(env_closefigures)
    REFLECT_SERVICE(env_createbody)
    REFLECT_SERVICE(env_createplanner)
    REFLECT_SERVICE(env_createproblem)
    REFLECT_SERVICE(env_createrobot)
    REFLECT_SERVICE(env_destroyproblem)
    REFLECT_SERVICE(env_getbodies)
    REFLECT_SERVICE(env_getbody)
    REFLECT_SERVICE(env_getrobots)
    REFLECT_SERVICE(env_loadplugin)
    REFLECT_SERVICE(env_loadscene)
    REFLECT_SERVICE(env_plot)
    REFLECT_SERVICE(env_raycollision)
    REFLECT_SERVICE(env_set)
    REFLECT_SERVICE(env_triangulate)
    REFLECT_SERVICE(env_wait)
    REFLECT_SERVICE(planner_init)
    REFLECT_SERVICE(planner_plan)
    REFLECT_SERVICE(problem_sendcommand)
    REFLECT_SERVICE(robot_controllersend)
    REFLECT_SERVICE(robot_controllerset)
    REFLECT_SERVICE(robot_getactivevalues)
    REFLECT_SERVICE(robot_sensorgetdata)
    REFLECT_SERVICE(robot_sensorsend)
    REFLECT_SERVICE(robot_setactivedofs)
    REFLECT_SERVICE(robot_setactivevalues)
    REFLECT_SERVICE(robot_starttrajectory)
};

// check that message constants match OpenRAVE constants
BOOST_STATIC_ASSERT((int)Clone_Bodies==(int)openrave_session::Request::CloneBodies);
BOOST_STATIC_ASSERT((int)Clone_Viewer==(int)openrave_session::Request::CloneViewer);
BOOST_STATIC_ASSERT((int)Clone_Simulation==(int)openrave_session::Request::CloneSimulation);
BOOST_STATIC_ASSERT((int)Clone_RealControllers==(int)openrave_session::Request::CloneRealControllers);

BOOST_STATIC_ASSERT((int)ActiveDOFs::DOF_X==(int)RobotBase::DOF_X);
BOOST_STATIC_ASSERT((int)ActiveDOFs::DOF_Y==(int)RobotBase::DOF_Y);
BOOST_STATIC_ASSERT((int)ActiveDOFs::DOF_Z==(int)RobotBase::DOF_Z);
BOOST_STATIC_ASSERT((int)ActiveDOFs::DOF_RotationAxis==(int)RobotBase::DOF_RotationAxis);
BOOST_STATIC_ASSERT((int)ActiveDOFs::DOF_Rotation3D==(int)RobotBase::DOF_Rotation3D);
BOOST_STATIC_ASSERT((int)ActiveDOFs::DOF_RotationQuat==(int)RobotBase::DOF_RotationQuat);

BOOST_STATIC_ASSERT((int)env_checkcollision::Request::CO_Distance==(int)CO_Distance);
BOOST_STATIC_ASSERT((int)env_checkcollision::Request::CO_UseTolerance==(int)CO_UseTolerance);
BOOST_STATIC_ASSERT((int)env_checkcollision::Request::CO_Contacts==(int)CO_Contacts);
BOOST_STATIC_ASSERT((int)env_checkcollision::Request::CO_RayAnyHit==(int)CO_RayAnyHit);
