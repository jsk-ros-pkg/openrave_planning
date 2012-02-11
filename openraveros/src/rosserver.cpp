// -*- coding: utf-8 -*-
// Copyright (c) 2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "openraveros.h"

class ROSServer : public ModuleBase
{
    inline boost::shared_ptr<ROSServer> shared_server() {
        return boost::static_pointer_cast<ROSServer>(shared_from_this());
    }
    inline boost::shared_ptr<ROSServer const> shared_server_const() const {
        return boost::static_pointer_cast<ROSServer const>(shared_from_this());
    }

public:
    ROSServer(EnvironmentBasePtr penv, std::istream& sinput) : ModuleBase(penv), _nNextFigureId(1), _nNextPlannerId(1), _nNextModuleId(1), _iWorkerId(0) {
        __description = ":Interface Author: Rosen Diankov\n\nOffers many services that can control this environment through ROS. When calling Environment::AddModule, can pass in the namespace to advertise the services on. Note that these are services just for the environment that instantiates this module! Check out the openraveros_tutorials package for how to use it (http://www.ros.org/wiki/openraveros_tutorials)";
        _bDestroyThreads = false;
        _bWorking = false;
        _fSimulationTimestep = 0.01;
    }

    virtual ~ROSServer() {
        Destroy();
    }

    virtual void Destroy()
    {
        _bDestroyThreads = true;
        Reset();
        {
            boost::mutex::scoped_lock lock(_mutexWorker);
            _condHasWork.notify_all();
            _ros.reset();
            _mapservices.clear();
        }
        // have to maintain a certain destruction order
        _threadviewer.join();
        _workerthread.join();
        _threadros.join();

        _bDestroyThreads = false;
        ModuleBase::Destroy();
    }

    virtual void Reset()
    {
        // destroy environment specific state: modules, planners, figures
        {
            boost::mutex::scoped_lock lock(_mutexWorker);
            _mapplanners.clear();
            _mapFigureIds.clear();
            _listWorkers.clear();

            {
                boost::mutex::scoped_lock lock(_mutexModules);
                FOREACH(itprob, _mapmodules)
                itprob->second->GetEnv()->Remove(itprob->second);
                _mapmodules.clear();
            }
        }

        // wait for worker thread to stop
        while(_bWorking) {
            _conditionWorkers.notify_all();
            usleep(1000);
        }

        if( !!_pviewer ) {
            _pviewer->quitmainloop();
            _pviewer.reset();
        }
    }

    virtual int main(const std::string& args)
    {
        Destroy();
        _bDestroyThreads = false;
        _bWorking = false;
        _threadros.join();

        stringstream ss(args);
        int argc=0;
        ros::init(argc,NULL,"openraverosserver", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        if( !ros::master::check() ) {
            RAVELOG_WARN("failed to create ros\n");
            return -1;
        }
        _ros.reset(new ros::NodeHandle());

        _threadros = boost::thread(boost::bind(&ROSServer::_threadrosfn, this));
        _workerthread = boost::thread(boost::bind(&ROSServer::_WorkerThread,this));

        string ns;
        ss >> ns;
        if( ns.size() > 0 ) {
            if( ns.at(ns.size()-1) != '/' ) {
                ns.push_back('/');
            }
        }
        else {
            ns="openrave/";
        }
        _mapservices["body_destroy"] = _ros->advertiseService(ns+string("body_destroy"),&ROSServer::body_destroy_srv,this);
        _mapservices["body_enable"] = _ros->advertiseService(ns+string("body_enable"),&ROSServer::body_enable_srv,this);
        _mapservices["body_getaabb"] = _ros->advertiseService(ns+string("body_getaabb"),&ROSServer::body_getaabb_srv,this);
        _mapservices["body_getaabbs"] = _ros->advertiseService(ns+string("body_getaabbs"),&ROSServer::body_getaabbs_srv,this);
        _mapservices["body_getdof"] = _ros->advertiseService(ns+string("body_getdof"),&ROSServer::body_getdof_srv,this);
        _mapservices["body_getjointvalues"] = _ros->advertiseService(ns+string("body_getjointvalues"),&ROSServer::body_getjointvalues_srv,this);
        _mapservices["body_setjointvalues"] = _ros->advertiseService(ns+string("body_setjointvalues"),&ROSServer::body_setjointvalues_srv,this);
        _mapservices["body_settransform"] = _ros->advertiseService(ns+string("body_settransform"),&ROSServer::body_settransform_srv,this);
        _mapservices["env_checkcollision"] = _ros->advertiseService(ns+string("env_checkcollision"),&ROSServer::env_checkcollision_srv,this);
        _mapservices["env_closefigures"] = _ros->advertiseService(ns+string("env_closefigures"),&ROSServer::env_closefigures_srv,this);
        _mapservices["env_createbody"] = _ros->advertiseService(ns+string("env_createbody"),&ROSServer::env_createbody_srv,this);
        _mapservices["env_createplanner"] = _ros->advertiseService(ns+string("env_createplanner"),&ROSServer::env_createplanner_srv,this);
        _mapservices["env_createmodule"] = _ros->advertiseService(ns+string("env_createmodule"),&ROSServer::env_createmodule_srv,this);
        _mapservices["env_createrobot"] = _ros->advertiseService(ns+string("env_createrobot"),&ROSServer::env_createrobot_srv,this);
        _mapservices["env_destroymodule"] = _ros->advertiseService(ns+string("env_destroymodule"),&ROSServer::env_destroymodule_srv,this);
        _mapservices["env_getbodies"] = _ros->advertiseService(ns+string("env_getbodies"),&ROSServer::env_getbodies_srv,this);
        _mapservices["env_getbody"] = _ros->advertiseService(ns+string("env_getbody"),&ROSServer::env_getbody_srv,this);
        _mapservices["env_getrobots"] = _ros->advertiseService(ns+string("env_getrobots"),&ROSServer::env_getrobots_srv,this);
        _mapservices["env_loadplugin"] = _ros->advertiseService(ns+string("env_loadplugin"),&ROSServer::env_loadplugin_srv,this);
        _mapservices["env_loadscene"] = _ros->advertiseService(ns+string("env_loadscene"),&ROSServer::env_loadscene_srv,this);
        _mapservices["env_plot"] = _ros->advertiseService(ns+string("env_plot"),&ROSServer::env_plot_srv,this);
        _mapservices["env_raycollision"] = _ros->advertiseService(ns+string("env_raycollision"),&ROSServer::env_raycollision_srv,this);
        _mapservices["env_set"] = _ros->advertiseService(ns+string("env_set"),&ROSServer::env_set_srv,this);
        _mapservices["env_triangulate"] = _ros->advertiseService(ns+string("env_triangulate"),&ROSServer::env_triangulate_srv,this);
        _mapservices["env_wait"] = _ros->advertiseService(ns+string("env_wait"),&ROSServer::env_wait_srv,this);
        _mapservices["planner_init"] = _ros->advertiseService(ns+string("planner_init"),&ROSServer::planner_init_srv,this);
        _mapservices["planner_plan"] = _ros->advertiseService(ns+string("planner_plan"),&ROSServer::planner_plan_srv,this);
        _mapservices["module_sendcommand"] = _ros->advertiseService(ns+string("module_sendcommand"),&ROSServer::module_sendcommand_srv,this);
        _mapservices["robot_controllersend"] = _ros->advertiseService(ns+string("robot_controllersend"),&ROSServer::robot_controllersend_srv,this);
        _mapservices["robot_controllerset"] = _ros->advertiseService(ns+string("robot_controllerset"),&ROSServer::robot_controllerset_srv,this);
        _mapservices["robot_getactivevalues"] = _ros->advertiseService(ns+string("robot_getactivevalues"),&ROSServer::robot_getactivevalues_srv,this);
        _mapservices["robot_sensorgetdata"] = _ros->advertiseService(ns+string("robot_sensorgetdata"),&ROSServer::robot_sensorgetdata_srv,this);
        _mapservices["robot_sensorsend"] = _ros->advertiseService(ns+string("robot_sensorsend"),&ROSServer::robot_sensorsend_srv,this);
        _mapservices["robot_setactivedofs"] = _ros->advertiseService(ns+string("robot_setactivedofs"),&ROSServer::robot_setactivedofs_srv,this);
        _mapservices["robot_setactivevalues"] = _ros->advertiseService(ns+string("robot_setactivevalues"),&ROSServer::robot_setactivevalues_srv,this);
        _mapservices["robot_starttrajectory"] = _ros->advertiseService(ns+string("robot_starttrajectory"),&ROSServer::robot_starttrajectory_srv,this);
        return 0;
    }

    virtual void _threadrosfn()
    {
        while(!_bDestroyThreads && ros::ok()) {
            ros::spinOnce();
            usleep(1000); // query every 1ms?
        }
    }

    /// worker thread called from the main environment thread
    virtual void _WorkerThread()
    {
        RAVELOG_DEBUG("starting ros worker thread\n");
        list<boost::function<void()> > listlocalworkers;

        while(!_bDestroyThreads) {
            {
                boost::mutex::scoped_lock lock(_mutexWorker);
                _condHasWork.wait(lock);
                if( _bDestroyThreads ) {
                    break;
                }
                if( _listWorkers.size() == 0 ) {
                    _conditionWorkers.notify_all();
                    continue;
                }

                _bWorking = true;
                listlocalworkers.swap(_listWorkers);
                _iWorkerId++;
            }

            // transfer the current workers to a temporary list so
            FOREACH(it, listlocalworkers) {
                try {
                    (*it)();
                }
                catch(const openrave_exception& ex) {
                    RAVELOG_FATAL("%s\n",ex.what());
                }
            }
            listlocalworkers.clear();

            boost::mutex::scoped_lock lock(_mutexWorker);
            _bWorking = false;
            _iWorkerId++;
            _conditionWorkers.notify_all();
        }

        _bWorking = false;
        RAVELOG_DEBUG("stopping ros worker thread\n");
    }

    virtual void AddWorker(const boost::function<void()>& fn, bool bWait=true)
    {
        boost::mutex::scoped_lock lock(_mutexWorker);
        int iWorkerId = _iWorkerId;

        _listWorkers.push_back(fn);
        _condHasWork.notify_all();
        if( bWait ) {
            while(!_bDestroyThreads) {
                _conditionWorkers.wait(lock);
                if( _iWorkerId >= iWorkerId+2 ) {
                    break;
                }
                _condHasWork.notify_all();
            }
        }
    }

    /// viewer thread assuming you can create different viewers in their own therads
    virtual void ViewerThread(const string& strviewer)
    {
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _pviewer = RaveCreateViewer(GetEnv(),strviewer);
            if( !!_pviewer ) {
                GetEnv()->AddViewer(_pviewer);
                _pviewer->SetSize(1024,768);
            }
            _conditionViewer.notify_all();
        }

        if( !_pviewer ) {
            return;
        }
        _pviewer->main(); // spin until quitmainloop is called
        RAVELOG_DEBUGA("destroying viewer\n");
        _pviewer.reset();
    }

    bool SetPhysicsEngine(const string& physicsengine)
    {
        PhysicsEngineBasePtr p = RaveCreatePhysicsEngine(GetEnv(),physicsengine);
        if( !p ) {
            return false;
        }
        GetEnv()->SetPhysicsEngine(p);
        return true;
    }

    bool SetCollisionChecker(const string& collisionchecker)
    {
        CollisionCheckerBasePtr p = RaveCreateCollisionChecker(GetEnv(),collisionchecker);
        if( !p ) {
            return false;
        }
        GetEnv()->SetCollisionChecker(p);
        return true;
    }

    bool SetViewer(const string& viewer)
    {
        if( viewer.size() == 0 ) {
            return true;
        }
        if( !!_setviewer ) {
            stringstream ss;
            ss << "OpenRAVE " << OPENRAVE_VERSION_STRING << " - session " << RaveGetEnvironmentId(GetEnv());
            return _setviewer(viewer,ss.str());
        }

        _threadviewer.join(); // wait for the viewer

        if( viewer.size() > 0 ) {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _threadviewer = boost::thread(boost::bind(&ROSServer::ViewerThread, this, viewer));
            _conditionViewer.wait(lock);

            if( !_pviewer ) {
                RAVELOG_WARNA(str(boost::format("failed to create viewer %s\n")%viewer));
                _threadviewer.join();
                return false;
            }
            else {
                RAVELOG_INFO(str(boost::format("viewer %s successfully attached\n")%viewer));
            }
        }

        return true;
    }

    //////////////
    // services //
    //////////////

    bool body_destroy_srv(openraveros::body_destroy::Request& req, openraveros::body_destroy::Response& res)
    {
        KinBodyPtr pbody = _FromROSBody(req.bodyid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return GetEnv()->Remove(pbody);
    }

    bool body_enable_srv(openraveros::body_enable::Request& req, openraveros::body_enable::Response& res)
    {
        KinBodyPtr pbody = _FromROSBody(req.bodyid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        pbody->Enable(req.enable);
        return true;
    }

    bool body_getaabb_srv(openraveros::body_getaabb::Request& req, openraveros::body_getaabb::Response& res)
    {
        KinBodyPtr pbody = _FromROSBody(req.bodyid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        OpenRAVE::AABB ab = pbody->ComputeAABB();
        res.box.center[0] = ab.pos.x; res.box.center[1] = ab.pos.y; res.box.center[2] = ab.pos.z;
        res.box.extents[0] = ab.extents.x; res.box.extents[1] = ab.extents.y; res.box.extents[2] = ab.extents.z;
        return true;
    }

    bool body_getaabbs_srv(openraveros::body_getaabbs::Request& req, openraveros::body_getaabbs::Response& res)
    {
        KinBodyPtr pbody = _FromROSBody(req.bodyid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        res.boxes.resize(pbody->GetLinks().size()); int index = 0;
        FOREACHC(itlink, pbody->GetLinks()) {
            OpenRAVE::AABB ab = (*itlink)->ComputeAABB();
            openraveros::AABB& resbox = res.boxes[index++];
            resbox.center[0] = ab.pos.x; resbox.center[1] = ab.pos.y; resbox.center[2] = ab.pos.z;
            resbox.extents[0] = ab.extents.x; resbox.extents[1] = ab.extents.y; resbox.extents[2] = ab.extents.z;
        }
        return true;
    }

    bool body_getdof_srv(openraveros::body_getdof::Request& req, openraveros::body_getdof::Response& res)
    {
        KinBodyPtr pbody = _FromROSBody(req.bodyid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        res.dof = pbody->GetDOF();
        return true;
    }

    bool body_getjointvalues_srv(openraveros::body_getjointvalues::Request& req, openraveros::body_getjointvalues::Response& res)
    {
        KinBodyPtr pbody = _FromROSBody(req.bodyid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        if( req.indices.size() > 0 ) {
            vector<dReal> vtemp;
            pbody->GetDOFValues(vtemp);

            res.values.resize(req.indices.size());
            for(size_t i = 0; i < req.indices.size(); ++i) {
                BOOST_ASSERT( req.indices[i] < vtemp.size() );
                res.values[i] = vtemp[req.indices[i]];
            }
        }
        else {
            vector<dReal> vtemp;
            pbody->GetDOFValues(vtemp);

            res.values.resize(vtemp.size());
            for(size_t i = 0; i < res.values.size(); ++i)
                res.values[i] = vtemp[i];
        }

        return true;
    }

    bool body_setjointvalues_srv(openraveros::body_setjointvalues::Request& req, openraveros::body_setjointvalues::Response& res)
    {
        KinBodyPtr pbody = _FromROSBody(req.bodyid);
        BOOST_ASSERT( pbody->GetDOF() > 0 );

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        vector<dReal> vvalues;

        if( req.indices.size() > 0 ) {
            if( req.indices.size() != req.jointvalues.size() ) {
                RAVELOG_WARNA("indices (%d) different size than joint values (%d)\n", req.indices.size(), req.jointvalues.size());
                return false;
            }

            pbody->GetDOFValues(vvalues);
            for(uint32_t i = 0; i < req.indices.size(); ++i)
                vvalues[req.indices[i]] = req.jointvalues[i];
        }
        else {
            if( pbody->GetDOF() != (int)req.jointvalues.size() ) {
                RAVELOG_WARNA("body dof (%d) not equal to jointvalues (%d)", pbody->GetDOF(), req.jointvalues.size());
                return false;
            }

            vvalues.reserve(req.jointvalues.size());
            FOREACHC(it,req.jointvalues)
            vvalues.push_back(*it);
        }

        pbody->SetJointValues(vvalues, true);

        if( pbody->IsRobot() ) {
            // if robot, should turn off any trajectory following
            RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);
            if( !!probot->GetController() ) {
                probot->GetDOFValues(vvalues); // reget the values since they'll go through the joint limits
                probot->GetController()->SetDesired(vvalues);
            }
        }

        return true;
    }

    bool body_settransform_srv(openraveros::body_settransform::Request& req, openraveros::body_settransform::Response& res)
    {
        KinBodyPtr pbody = _FromROSBody(req.bodyid);
        Transform t = _FromROSAffineTransform(req.transform);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        pbody->SetTransform(t);

        if( pbody->IsRobot() ) {
            RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);
            if( !!probot->GetController() )
                probot->GetController()->SetPath(TrajectoryBasePtr());
        }

        return true;
    }

    bool env_checkcollision_srv(openraveros::env_checkcollision::Request& req, openraveros::env_checkcollision::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

        vector<KinBodyConstPtr> setignore;
        FOREACH(it, req.excludedbodyids) {
            KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
            if( !pbody )
                return false;
            setignore.push_back(pbody);
        }

        boost::shared_ptr<CollisionReport> report(new CollisionReport());
        {
            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            int oldopts = GetEnv()->GetCollisionChecker()->GetCollisionOptions();
            if( !GetEnv()->GetCollisionChecker()->SetCollisionOptions(req.options) )
                RAVELOG_WARNA("failed to set collision options\n");

            vector<KinBody::LinkConstPtr> empty;
            if( req.linkid < 0 )
                res.collision = GetEnv()->CheckCollision(pbody, setignore, empty, report);
            else {
                if( req.linkid >= (int)pbody->GetLinks().size() )
                    return false;
                res.collision = GetEnv()->CheckCollision(pbody->GetLinks().at(req.linkid), setignore, empty, report);
            }

            if( !res.collision && req.checkselfcollision ) {
                res.collision = pbody->CheckSelfCollision(report);
            }

            GetEnv()->GetCollisionChecker()->SetCollisionOptions(oldopts);
        }

        res.collidingbodyid = 0;

        if( res.collision ) {
            KinBody::LinkConstPtr plinkcol = report->plink1;
            if( !!report->plink2 && report->plink2->GetParent() != pbody && !pbody->IsAttached(report->plink2->GetParent()) ) {
                plinkcol = report->plink2;
            }

            if( !!plinkcol ) {
                res.collidingbodyid = plinkcol->GetParent()->GetEnvironmentId();
                res.collidinglink = plinkcol->GetIndex();
            }

            RAVELOG_DEBUGA(str(boost::format("collision %s:%s with %s:%s\n")%
                               (!!report->plink1 ? report->plink1->GetParent()->GetName() : "(NULL)")%
                               (!!report->plink1 ? report->plink1->GetName() : "(NULL)")%
                               (!!report->plink2 ? report->plink2->GetParent()->GetName() : "(NULL)")%
                               (!!report->plink2 ? report->plink2->GetName() : "(NULL)")));
        }

        if( req.options & CO_Distance )
            res.mindist = report->minDistance;
        if( req.options & CO_Contacts ) {
            res.contacts.resize(report->contacts.size());
            int negnormals = 0;
            if( !!report->plink1 ) {
                if( req.linkid < 0 )
                    negnormals = report->plink1->GetParent() != pbody;
                else
                    negnormals = report->plink1->GetParent() != pbody || report->plink1->GetIndex() != req.linkid;
            }

            int index = 0;
            FOREACHC(itc, report->contacts) {
                openraveros::Contact& c = res.contacts[index++];
                Vector vnorm = negnormals ? -itc->norm : itc->norm;
                c.position[0] = itc->pos.x; c.position[1] = itc->pos.y; c.position[2] = itc->pos.z;
                c.normal[0] = vnorm.x; c.normal[1] = vnorm.y; c.normal[2] = vnorm.z;
            }
        }

        return true;
    }

    bool env_closefigures_srv(openraveros::env_closefigures::Request& req, openraveros::env_closefigures::Response& res)
    {
        bool bSuccess = true;

        if( req.figureids.size() > 0 ) {
            FOREACH(itid, req.figureids) {
                if( !_mapFigureIds.erase(*itid) )
                    bSuccess = false;
            }
        }
        else // destroy everything
            _mapFigureIds.clear();

        return bSuccess;
    }

    bool env_createbody_srv(openraveros::env_createbody::Request& req, openraveros::env_createbody::Response& res)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        KinBodyPtr pbody;

        if( req.file.size() > 0 ) {
            pbody = GetEnv()->ReadKinBodyXMLFile(req.file);
        }
        else {
            pbody = RaveCreateKinBody(GetEnv());
        }
        if( !pbody ) {
            return false;
        }
        pbody->SetName(req.name);

        GetEnv()->AddKinBody(pbody);
        res.bodyid = pbody->GetEnvironmentId();
        return true;
    }

    bool env_createplanner_srv(openraveros::env_createplanner::Request& req, openraveros::env_createplanner::Response& res)
    {
        PlannerBasePtr pplanner = RaveCreatePlanner(GetEnv(),req.plannertype);
        if( !pplanner )
            return false;

        _mapplanners[++_nNextPlannerId] = pplanner;
        res.plannerid = _nNextPlannerId;
        return true;
    }

    void _AddModuleWorker(int& retval, ModuleBasePtr module, const string& args  )
    {
        retval = module->GetEnv()->AddModule(module, args);
    }

    bool env_createmodule_srv(openraveros::env_createmodule::Request& req, openraveros::env_createmodule::Response& res)
    {
        ModuleBasePtr pmodule = RaveCreateModule(GetEnv(),req.xmlid);
        if( !pmodule ) {
            return false;
        }
        boost::mutex::scoped_lock lock(_mutexModules);

        if( req.destroyduplicates ) {
            map<int, ModuleBasePtr >::iterator itmodule = _mapmodules.begin();
            while(itmodule != _mapmodules.end()) {
                if( itmodule->second->GetXMLId() == req.xmlid ) {
                    RAVELOG_INFO(str(boost::format("deleting duplicate module %s\n")%req.xmlid));
                    if( !GetEnv()->Remove(itmodule->second) ) {
                        RAVELOG_WARN(str(boost::format("failed to remove module %s\n")%itmodule->second->GetXMLId()));
                    }
                    _mapmodules.erase(itmodule++);
                }
                else {
                    ++itmodule;
                }
            }
        }

        int retval=0;
        AddWorker(boost::bind(&ROSServer::_AddModuleWorker,shared_server(),boost::ref(retval), pmodule,req.args),true);

        if( retval != 0 ) {
            RAVELOG_WARNA(str(boost::format("failed to load module %s with args %s\n")%req.xmlid%req.args));
            return false;
        }

        _mapmodules[++_nNextModuleId] = pmodule;
        res.id = _nNextModuleId;
        return true;
    }

    bool env_createrobot_srv(openraveros::env_createrobot::Request& req, openraveros::env_createrobot::Response& res)
    {
        RobotBasePtr probot;
        if( req.file.size() > 0 ) {
            probot = GetEnv()->ReadRobotXMLFile(req.file);
        }
        else {
            probot = RaveCreateRobot(GetEnv(),req.type);
        }
        if( !probot ) {
            return false;
        }
        probot->SetName(req.name);

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        GetEnv()->AddRobot(probot);;
        res.bodyid = probot->GetEnvironmentId();
        return true;
    }

    bool env_destroymodule_srv(openraveros::env_destroymodule::Request& req, openraveros::env_destroymodule::Response& res)
    {
        boost::mutex::scoped_lock lock(_mutexModules);
        map<int, ModuleBasePtr >::iterator itmodule = _mapmodules.find(req.id);
        if( itmodule == _mapmodules.end() ) {
            return false;
        }
        ModuleBasePtr prob = itmodule->second;
        _mapmodules.erase(itmodule);

        if( !GetEnv()->Remove(itmodule->second) ) {
            RAVELOG_WARNA("failed to remove module\n");
            return false;
        }
        return true;
    }

    bool env_getbodies_srv(openraveros::env_getbodies::Request& req, openraveros::env_getbodies::Response& res)
    {
        vector<KinBodyPtr> vbodies;
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        GetEnv()->GetBodies(vbodies);

        if( req.bodyid != 0 ) {
            KinBodyPtr pfound;
            FOREACH(it, vbodies) {
                if( (*it)->GetEnvironmentId() == req.bodyid ) {
                    pfound = *it;
                    break;
                }
            }

            if( !pfound )
                return false;

            // add only one body
            vbodies.resize(0); vbodies.push_back(pfound);
        }

        res.bodies.resize(vbodies.size()); int index = 0;
        FOREACH(itbody, vbodies) {
            openraveros::BodyInfo& info = res.bodies[index++];
            _GetROSBodyInfo(*itbody, info, req.options);
        }

        return true;
    }

    bool env_getbody_srv(openraveros::env_getbody::Request& req, openraveros::env_getbody::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetKinBody(req.name);
        if( !pbody )
            return false;
        res.bodyid = pbody->GetEnvironmentId();
        return true;
    }

    bool env_getrobots_srv(openraveros::env_getrobots::Request& req, openraveros::env_getrobots::Response& res)
    {
        vector<RobotBasePtr> vrobots;
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        GetEnv()->GetRobots(vrobots);

        if( req.bodyid != 0 ) {
            RobotBasePtr pfound;
            FOREACH(it, vrobots) {
                if( (*it)->GetEnvironmentId() == req.bodyid ) {
                    pfound = *it;
                    break;
                }
            }

            if( !pfound )
                return false;

            // add only one body
            vrobots.resize(0); vrobots.push_back(pfound);
        }

        res.robots.resize(vrobots.size()); int index = 0;
        FOREACH(itrobot, vrobots) {
            BOOST_ASSERT( (*itrobot)->IsRobot() );
            openraveros::RobotInfo& info = res.robots[index++];
            _GetROSRobotInfo(*itrobot, info, req.options);
        }

        return true;
    }

    bool env_loadplugin_srv(openraveros::env_loadplugin::Request& req, openraveros::env_loadplugin::Response& res)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return RaveLoadPlugin(req.filename);
    }

    bool env_loadscene_srv(openraveros::env_loadscene::Request& req, openraveros::env_loadscene::Response& res)
    {
        if( req.resetscene )
            GetEnv()->Reset();
        if( req.filename.size() > 0 )
            return GetEnv()->Load(req.filename);

        return true;
    }

    bool env_plot_srv(openraveros::env_plot::Request& req, openraveros::env_plot::Response& res)
    {
        bool bOneColor = req.colors.size() != req.points.size();
        float falpha = max(0.0f, 1.0f-req.transparency);
        falpha = min(1.0f,falpha);
        RaveVector<float> vOneColor(1.0f,0.5f,0.5f,falpha);
        if( req.colors.size() >= 3 )
            vOneColor = RaveVector<float>(req.colors[0], req.colors[1], req.colors[2],falpha);

        GraphHandlePtr figure;
        switch(req.drawtype) {
        case openraveros::env_plot::Request::Draw_Point:
            if( bOneColor ) {
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor, 0);
            }
            else {
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 0);
            }
            break;
        case openraveros::env_plot::Request::Draw_LineStrip:
            if( bOneColor ) {
                figure = GetEnv()->drawlinestrip(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor);
            }
            else {
                figure = GetEnv()->drawlinestrip(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0]);
            }
            break;
        case openraveros::env_plot::Request::Draw_LineList:
            if( bOneColor ) {
                figure = GetEnv()->drawlinelist(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor);
            }
            else {
                figure = GetEnv()->drawlinelist(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0]);
            }
            break;
        case openraveros::env_plot::Request::Draw_TriList:
            //if( bOneColor )
            figure = GetEnv()->drawtrimesh(&req.points[0],3*sizeof(req.points[0]), NULL, req.points.size()/9, vOneColor);
            //else
            //figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 0);
            break;
        case openraveros::env_plot::Request::Draw_Sphere:
            if( bOneColor ) {
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor, 1);
            }
            else {
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 1);
            }
            break;
        default:
            return false;
        }

        if( !figure ) {
            return false;
        }
        _mapFigureIds[++_nNextFigureId] = figure;
        res.figureid = _nNextFigureId;
        return true;
    }

    bool env_raycollision_srv(openraveros::env_raycollision::Request& req, openraveros::env_raycollision::Response& res)
    {
        KinBodyPtr pbody;
        if( req.bodyid != 0 )
            pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);

        res.collision.reserve(req.rays.size()); res.collision.resize(0);

        if( req.request_contacts ) {
            res.contacts.reserve(req.rays.size());
            res.contacts.resize(0);
        }

        if( req.request_bodies ) {
            res.hitbodies.reserve(req.rays.size());
            res.hitbodies.resize(0);
        }

        boost::shared_ptr<CollisionReport> report(new CollisionReport());
        {
            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            int oldopts = GetEnv()->GetCollisionChecker()->GetCollisionOptions();
            if( !GetEnv()->GetCollisionChecker()->SetCollisionOptions(req.request_contacts ? CO_Contacts : 0) )
                RAVELOG_WARNA("failed to set collision options\n");

            FOREACHC(itray, req.rays) {
                RAY r;
                r.pos = Vector(itray->position[0], itray->position[1], itray->position[2]);
                r.dir = Vector(itray->direction[0], itray->direction[1], itray->direction[2]);

                uint8_t bCollision = 0;

                if( r.dir.lengthsqr3() > 1e-7 ) {
                    r.dir.normalize3();

                    if( !!pbody )
                        bCollision = GetEnv()->CheckCollision(r,pbody,report);
                    else
                        bCollision = GetEnv()->CheckCollision(r,report);
                }
                else
                    RAVELOG_WARNA("ray has zero direction\n");

                res.collision.push_back(bCollision);
                if( bCollision && req.request_contacts ) {
                    openraveros::Contact rosc;
                    if( report->contacts.size() > 0 ) {
                        CollisionReport::CONTACT& c = report->contacts.front();
                        rosc.position[0] = c.pos.x; rosc.position[1] = c.pos.y; rosc.position[2] = c.pos.z;
                        rosc.normal[0] = c.norm.x; rosc.normal[1] = c.norm.y; rosc.normal[2] = c.norm.z;
                    }
                    res.contacts.push_back(rosc);
                }
                else
                    res.contacts.push_back(openraveros::Contact());


                if( bCollision && req.request_bodies ) {
                    KinBody::LinkConstPtr plink = !!report->plink1 ? report->plink1 : report->plink2;
                    res.hitbodies.push_back(!!plink ? plink->GetParent()->GetEnvironmentId() : 0);
                }
                else
                    res.hitbodies.push_back(0);
            }

            GetEnv()->GetCollisionChecker()->SetCollisionOptions(oldopts);
        }

        return true;
    }

    bool env_set_srv(openraveros::env_set::Request& req, openraveros::env_set::Response& res)
    {
        if( req.setmask & openraveros::env_set::Request::Set_Simulation ) {
            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            switch(req.sim_action) {
            case openraveros::env_set::Request::SimAction_Start:
                if( req.sim_timestep > 0 ) {
                    _fSimulationTimestep = req.sim_timestep;
                }
                GetEnv()->StartSimulation(_fSimulationTimestep);
                break;
            case openraveros::env_set::Request::SimAction_Stop:
                GetEnv()->StopSimulation();
                break;
            case openraveros::env_set::Request::SimAction_Timestep:
                _fSimulationTimestep = req.sim_timestep;
                GetEnv()->StartSimulation(_fSimulationTimestep);
                break;
            }
        }
        if( req.setmask & openraveros::env_set::Request::Set_PhysicsEngine ) {
            SetPhysicsEngine(req.physicsengine);
        }
        if( req.setmask & openraveros::env_set::Request::Set_CollisionChecker ) {
            SetCollisionChecker(req.collisionchecker);
        }
        if( req.setmask & openraveros::env_set::Request::Set_Gravity ) {
            GetEnv()->GetPhysicsEngine()->SetGravity(Vector(req.gravity[0],req.gravity[1],req.gravity[2]));
        }
        if( req.setmask & openraveros::env_set::Request::Set_DebugLevel ) {
            map<string,DebugLevel> mlevels;
            mlevels["fatal"] = Level_Fatal;
            mlevels["error"] = Level_Error;
            mlevels["info"] = Level_Info;
            mlevels["warn"] = Level_Warn;
            mlevels["debug"] = Level_Debug;
            mlevels["verbose"] = Level_Verbose;
            int level = GetEnv()->GetDebugLevel();
            if( mlevels.find(req.debuglevel) != mlevels.end() ) {
                level = mlevels[req.debuglevel];
            }
            else {
                stringstream ss(req.debuglevel);
                int nlevel;
                ss >> nlevel;
                if( !!ss ) {
                    level = nlevel;
                }
            }
            GetEnv()->SetDebugLevel(level);
        }
        if( req.setmask & openraveros::env_set::Request::Set_Viewer ) {
            if( !!_pviewer ) {
                GetEnv()->Remove(_pviewer);
            }
            SetViewer(req.viewer);
        }
        if( req.setmask & openraveros::env_set::Request::Set_ViewerDims ) {
            if( !!GetEnv()->GetViewer() ) {
                GetEnv()->GetViewer()->SetSize(req.viewerwidth,req.viewerheight);
            }
        }

        return true;
    }

    bool env_triangulate_srv(openraveros::env_triangulate::Request& req, openraveros::env_triangulate::Response& res)
    {
        set<int> setobjids;
        FOREACH(it, req.bodyids)
        setobjids.insert(*it);

        KinBody::Link::TRIMESH trimesh;

        {
            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

            vector<KinBodyPtr> vbodies;
            GetEnv()->GetBodies(vbodies);

            FOREACH(itbody, vbodies) {
                if( (setobjids.find((*itbody)->GetEnvironmentId()) == setobjids.end()) ^ !req.inclusive )
                    continue;
                GetEnv()->Triangulate(trimesh, *itbody);
            }
        }

        res.points.resize(3*trimesh.vertices.size());
        for(size_t i = 0; i < trimesh.vertices.size(); ++i) {
            Vector& v = trimesh.vertices[i];
            res.points[3*i+0] = v.x;
            res.points[3*i+1] = v.y;
            res.points[3*i+2] = v.z;
        }

        res.indices.resize(trimesh.indices.size());
        for(size_t i = 0; i < trimesh.indices.size(); ++i)
            res.indices[i] = (uint32_t)trimesh.indices[i];

        return true;
    }

    bool env_wait_srv(openraveros::env_wait::Request& req, openraveros::env_wait::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() ) {
            return false;
        }
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);
        ControllerBasePtr pcontroller = probot->GetController();
        if( !pcontroller ) {
            return false;
        }
        uint32_t timeout = (uint32_t)(req.timeout*1000.0f);
        while( !pcontroller->IsDone() ) {
            usleep(400);
            if( timeout > 0 ) {
                if( --timeout == 0 )
                    break;
            }
            if( _bDestroyThreads ) {
                return false;
            }
        }

        res.isdone = pcontroller->IsDone();
        return true;
    }

    bool planner_init_srv(openraveros::planner_init::Request& req, openraveros::planner_init::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.robotid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);

        map<int, boost::shared_ptr<PlannerBase> >::iterator itplanner = _mapplanners.find(req.plannerid);
        if( itplanner == _mapplanners.end() )
            return false;

        boost::shared_ptr<PlannerBase::PlannerParameters> params;
        // fill with request
        RAVELOG_ERRORA("need to fill with params!\n");

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return itplanner->second->InitPlan(probot, params);
    }

    bool planner_plan_srv(openraveros::planner_plan::Request& req, openraveros::planner_plan::Response& res)
    {
        map<int, PlannerBasePtr >::iterator itplanner = _mapplanners.find(req.plannerid);
        if( itplanner == _mapplanners.end() ) {
            return false;
        }
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(),"");
        RAVELOG_DEBUG("starting to plan");
        if( !(itplanner->second->PlanPath(traj)&PS_HasSolution) ) {
            RAVELOG_DEBUG("plan failed\n");
            return false;
        }
        _GetROSTrajectory(res.trajectory, traj);
        return true;
    }

    bool module_sendcommand_srv(openraveros::module_sendcommand::Request& req, openraveros::module_sendcommand::Response& res)
    {
        boost::mutex::scoped_lock lock(_mutexModules);
        map<int, ModuleBasePtr >::iterator itmodule = _mapmodules.find(req.id);
        if( itmodule == _mapmodules.end() ) {
            return false;
        }
        try {
            stringstream sout;
            stringstream sin(req.cmd);
            if( itmodule->second->SendCommand(sout,sin) ) {
                res.output = sout.str();
                return true;
            }
        }
        catch(const openrave_exception& ex) {
            RAVELOG_FATAL("%s\n",ex.what());
        }

        return false;
    }

    bool robot_controllersend_srv(openraveros::robot_controllersend::Request& req, openraveros::robot_controllersend::Response& res)
    {
        RobotBasePtr probot = _FromROSRobot(req.robotid);
        ControllerBasePtr pcontroller = probot->GetController();
        if( !pcontroller ) {
            return false;
        }
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        stringstream sout;
        stringstream sin(req.cmd);
        if( pcontroller->SendCommand(sout,sin) ) {
            res.output = sout.str();
            return true;
        }
        return false;
    }

    bool robot_controllerset_srv(openraveros::robot_controllerset::Request& req, openraveros::robot_controllerset::Response& res)
    {
        RobotBasePtr probot = _FromROSRobot(req.robotid);
        ControllerBasePtr pcontroller = RaveCreateController(GetEnv(),req.controllername);
        if( !pcontroller ) {
            return false;
        }
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return probot->SetController(pcontroller,req.indices, req.controltransformation);
    }

    bool robot_getactivevalues_srv(openraveros::robot_getactivevalues::Request& req, openraveros::robot_getactivevalues::Response& res)
    {
        RobotBasePtr probot = _FromROSRobot(req.robotid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        if( req.indices.size() > 0 ) {
            vector<dReal> vtemp;
            probot->GetActiveDOFValues(vtemp);

            res.values.resize(req.indices.size());
            for(size_t i = 0; i < req.indices.size(); ++i) {
                if( req.indices[i] >= vtemp.size() )
                    return false;
                res.values[i] = vtemp[req.indices[i]];
            }
        }
        else {
            vector<dReal> vtemp;
            probot->GetActiveDOFValues(vtemp);

            res.values.resize(vtemp.size());
            for(size_t i = 0; i < res.values.size(); ++i)
                res.values[i] = vtemp[i];
        }

        return true;
    }

    bool robot_sensorgetdata_srv(openraveros::robot_sensorgetdata::Request& req, openraveros::robot_sensorgetdata::Response& res)
    {
        RobotBasePtr probot = _FromROSRobot(req.robotid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        if( req.sensorindex >= probot->GetAttachedSensors().size() )
            return false;

        SensorBasePtr psensor = probot->GetAttachedSensors().at(req.sensorindex)->GetSensor();
        if( !psensor )
            return false;

        SensorBase::SensorDataPtr pdata = psensor->CreateSensorData();
        if( !pdata ) {
            RAVELOG_ERRORA(str(boost::format("Robot %s, failed to create sensor %s data\n")%probot->GetName()%probot->GetAttachedSensors()[req.sensorindex]->GetName()));
            return false;
        }

        if( !psensor->GetSensorData(pdata) ) {
            RAVELOG_ERRORA(str(boost::format("Robot %s, failed to get sensor %s data\n")%probot->GetName()%probot->GetAttachedSensors().at(req.sensorindex)->GetName()));
            return false;
        }

        // serialize the data
        switch(pdata->GetType()) {
        case SensorBase::ST_Laser: {
            res.type = "laser";
            boost::shared_ptr<SensorBase::LaserSensorData> plaserdata = boost::static_pointer_cast<SensorBase::LaserSensorData>(pdata);
            int index;

            res.laserrange.resize(3*plaserdata->ranges.size()); index = 0;
            FOREACH(itpos, plaserdata->ranges) {
                res.laserrange[3*index+0] = itpos->x;
                res.laserrange[3*index+1] = itpos->y;
                res.laserrange[3*index+2] = itpos->z;
                ++index;
            }

            res.laserpos.resize(3*plaserdata->positions.size()); index = 0;
            FOREACH(itpos, plaserdata->positions) {
                res.laserpos[3*index+0] = itpos->x;
                res.laserpos[3*index+1] = itpos->y;
                res.laserpos[3*index+2] = itpos->z;
                ++index;
            }

            res.laserint.resize(plaserdata->intensity.size()); index = 0;
            FOREACH(itint, plaserdata->intensity)
            res.laserint[index++] = *itint;

            break;
        }
        case SensorBase::ST_Camera: {
            res.type = "camera";
            boost::shared_ptr<SensorBase::CameraSensorData> pcameradata = boost::static_pointer_cast<SensorBase::CameraSensorData>(pdata);

            if( psensor->GetSensorGeometry()->GetType() != SensorBase::ST_Camera ) {
                RAVELOG_WARNA("sensor geometry not a camera type\n");
                return false;
            }

            boost::shared_ptr<SensorBase::CameraGeomData> pgeom = boost::static_pointer_cast<SensorBase::CameraGeomData>(psensor->GetSensorGeometry());
            if( (int)pcameradata->vimagedata.size() != pgeom->width*pgeom->height*3 ) {
                RAVELOG_WARNA("image data wrong size %d != %d\n", pcameradata->vimagedata.size(), pgeom->width*pgeom->height*3);
                return false;
            }

            res.caminfo.width = pgeom->width;
            res.caminfo.height = pgeom->height;
            for(int i = 0; i < 5; ++i)
                res.caminfo.D[i] = 0;
            res.caminfo.K[0] = pgeom->KK.fx; res.caminfo.K[1] = 0; res.caminfo.K[2] = pgeom->KK.cx;
            res.caminfo.K[3] = 0; res.caminfo.K[4] = pgeom->KK.fy; res.caminfo.K[5] = pgeom->KK.cy;
            res.caminfo.K[6] = 0; res.caminfo.K[7] = 0; res.caminfo.K[8] = 1;

            TransformMatrix tKK;
            for(int i = 0; i < 3; ++i) {
                tKK.m[4*i+0] = res.caminfo.K[3*i+0];
                tKK.m[4*i+1] = res.caminfo.K[3*i+1];
                tKK.m[4*i+2] = res.caminfo.K[3*i+2];
            }

            TransformMatrix tP = tKK * pcameradata->__trans.inverse();

            res.caminfo.R[0] = 1; res.caminfo.R[1] = 0; res.caminfo.R[2] = 0;
            res.caminfo.R[3] = 0; res.caminfo.R[4] = 1; res.caminfo.R[5] = 0;
            res.caminfo.R[6] = 0; res.caminfo.R[7] = 0; res.caminfo.R[8] = 1;

            for(int i = 0; i < 3; ++i) {
                res.caminfo.P[4*i+0] = tP.m[4*i+0];
                res.caminfo.P[4*i+1] = tP.m[4*i+1];
                res.caminfo.P[4*i+2] = tP.m[4*i+2];
                res.caminfo.P[4*i+3] = tP.trans[i];
            }

            res.camimage.header.stamp = ros::Time(pcameradata->__stamp/1000000,(uint32_t)(1000*(pcameradata->__stamp%1000000)));
            res.camimage.width = pgeom->width;
            res.camimage.height = pgeom->height;
            res.camimage.step = pgeom->width * 3;

            res.camimage.data = pcameradata->vimagedata;
            break;
        }
        default:
            RAVELOG_ERRORA("sensor type not serialized\n");
        }

        return true;
    }

    bool robot_sensorsend_srv(openraveros::robot_sensorsend::Request& req, openraveros::robot_sensorsend::Response& res)
    {
        RobotBasePtr probot = _FromROSRobot(req.robotid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        RobotBase::AttachedSensorPtr sensor = probot->GetAttachedSensors().at(req.sensorindex);

        stringstream sout;
        stringstream sin;
        sin << req.cmd << " " << req.args;
        if( sensor->GetSensor()->SendCommand(sout,sin)) {
            res.out = sout.str();
            return true;
        }

        RAVELOG_ERROR(str(boost::format("Robot %s sensor %d failed: \"%s\"\n")%probot->GetName()%req.sensorindex%req.cmd));
        return false;
    }

    bool robot_setactivedofs_srv(openraveros::robot_setactivedofs::Request& req, openraveros::robot_setactivedofs::Response& res)
    {
        RobotBasePtr probot = _FromROSRobot(req.robotid);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _FromROSActiveDOFs(probot, req.active);
        return true;
    }

    bool robot_setactivevalues_srv(openraveros::robot_setactivevalues::Request& req, openraveros::robot_setactivevalues::Response& res)
    {
        RobotBasePtr probot = _FromROSRobot(req.robotid);
        if( probot->GetActiveDOF() == 0 ) {
            return false;
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        vector<dReal> vvalues;

        if( req.indices.size() > 0 ) {
            if( req.indices.size() != req.values.size() ) {
                throw OPENRAVE_EXCEPTION_FORMAT("indices (%d) different size than joint values (%d)", req.indices.size()%req.values.size(), ORE_InvalidArguments);
            }

            probot->GetActiveDOFValues(vvalues);
            for(size_t i = 0; i < req.indices.size(); ++i) {
                vvalues[req.indices[i]] = req.values[i];
            }
        }
        else {
            if( probot->GetActiveDOF() != (int)req.values.size() ) {
                throw OPENRAVE_EXCEPTION_FORMAT("body dof (%d) not equal to values (%d)", probot->GetDOF()%req.values.size(), ORE_InvalidArguments);
            }

            vvalues.reserve(req.values.size());
            FOREACHC(it,req.values) {
                vvalues.push_back(*it);
            }
        }

        probot->SetActiveDOFValues(vvalues, true);

        if( !!probot->GetController() ) {
            probot->GetDOFValues(vvalues); // reget the values since they'll go through the joint limits
            probot->GetController()->SetDesired(vvalues);
        }

        return true;
    }

    bool robot_starttrajectory_srv(openraveros::robot_starttrajectory::Request& req, openraveros::robot_starttrajectory::Response& res)
    {
        RobotBasePtr probot = _FromROSRobot(req.robotid);
        OPENRAVE_ASSERT_FORMAT(!!probot,"robot id %d",req.robotid,ORE_InvalidArguments);
        return probot->GetController()->SetPath(_FromROSTrajectory(req.trajectory));
    }

private:
    void _FromROSActiveDOFs(RobotBasePtr probot, const openraveros::ActiveDOFs& active)
    {
        probot->SetActiveDOFs(active.indices, active.affine, Vector(active.rotationaxis[0], active.rotationaxis[1], active.rotationaxis[2]));
    }

    void _GetROSActiveDOFs(RobotBaseConstPtr probot, openraveros::ActiveDOFs& active)
    {
        active.affine = probot->GetAffineDOF();
        active.indices = probot->GetActiveDOFIndices();
        active.rotationaxis[0] = probot->GetAffineRotationAxis().x;
        active.rotationaxis[1] = probot->GetAffineRotationAxis().y;
        active.rotationaxis[2] = probot->GetAffineRotationAxis().z;
    }

    RobotBasePtr _FromROSRobot(int robotid) {
        RobotBasePtr probot = RaveInterfaceCast<RobotBase>(GetEnv()->GetBodyFromEnvironmentId(robotid));
        if(!probot) {
            RAVELOG_WARN(str(boost::format("robot id %d not found")%robotid));
        }
        return probot;
    }

    KinBodyPtr _FromROSBody(int bodyid) {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(bodyid);
        if(!pbody) {
            RAVELOG_WARN(str(boost::format("body id %d not found")%bodyid));
        }
        return pbody;
    }

    void _GetROSConfigurationSpecification(openraveros::ConfigurationSpecification& rosspec, const ConfigurationSpecification& spec)
    {
        rosspec.groups.resize(spec._vgroups.size());
        for(size_t i = 0; i < spec._vgroups.size(); ++i) {
            rosspec.groups[i].name = spec._vgroups[i].name;
            rosspec.groups[i].interpolation = spec._vgroups[i].interpolation;
            rosspec.groups[i].dof = spec._vgroups[i].dof;
            rosspec.groups[i].offset = spec._vgroups[i].offset;
        }
    }

    void _FromROSConfigurationSpecification(ConfigurationSpecification& spec, const openraveros::ConfigurationSpecification& rosspec)
    {
        spec._vgroups.resize(rosspec.groups.size());
        for(size_t i = 0; i < rosspec.groups.size(); ++i) {
            spec._vgroups[i].name = rosspec.groups[i].name;
            spec._vgroups[i].interpolation = rosspec.groups[i].interpolation;
            spec._vgroups[i].dof = rosspec.groups[i].dof;
            spec._vgroups[i].offset = rosspec.groups[i].offset;
        }
    }

    TrajectoryBasePtr _FromROSTrajectory(const openraveros::Trajectory& rostraj)
    {
        TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(),rostraj.xmlid);
        ConfigurationSpecification spec;
        _FromROSConfigurationSpecification(spec,rostraj.spec);
        traj->Init(spec);
        traj->Insert(0,rostraj.points);
        return traj;
    }

    void _GetROSTrajectory(openraveros::Trajectory& rostraj, TrajectoryBaseConstPtr traj)
    {
        rostraj.xmlid = traj->GetXMLId();
        _GetROSConfigurationSpecification(rostraj.spec, traj->GetConfigurationSpecification());
        traj->GetWaypoints(0,traj->GetNumWaypoints(),rostraj.points);
    }

    openraveros::AffineTransformMatrix _GetROSAffineTransform(const TransformMatrix& tm)
    {
        openraveros::AffineTransformMatrix am;
        am.m[0] = tm.m[0]; am.m[3] = tm.m[1]; am.m[6] = tm.m[2];
        am.m[1] = tm.m[4]; am.m[4] = tm.m[5]; am.m[7] = tm.m[6];
        am.m[2] = tm.m[8]; am.m[5] = tm.m[9]; am.m[8] = tm.m[10];
        am.m[9] = tm.trans.x; am.m[10] = tm.trans.y; am.m[11] = tm.trans.z;
        return am;
    }

    TransformMatrix _FromROSAffineTransform(const openraveros::AffineTransformMatrix& am)
    {
        TransformMatrix tm;
        tm.m[0] = am.m[0]; tm.m[1] = am.m[3]; tm.m[2] = am.m[6];
        tm.m[4] = am.m[1]; tm.m[5] = am.m[4]; tm.m[6] = am.m[7];
        tm.m[8] = am.m[2]; tm.m[9] = am.m[5]; tm.m[10] = am.m[8];
        tm.trans.x = am.m[9]; tm.trans.y = am.m[10]; tm.trans.z = am.m[11];
        return tm;
    }

    openraveros::AffineTransformMatrix _GetROSAffineIdentity()
    {
        openraveros::AffineTransformMatrix am;
        am.m[0] = 1; am.m[3] = 0; am.m[6] = 0; am.m[9] = 0;
        am.m[1] = 0; am.m[4] = 1; am.m[7] = 0; am.m[10] = 0;
        am.m[2] = 0; am.m[5] = 0; am.m[8] = 1; am.m[11] = 0;
        return am;
    }

    void _GetROSBodyInfo(KinBodyConstPtr pbody, openraveros::BodyInfo& info, uint32_t options)
    {
        info.bodyid = pbody->GetEnvironmentId();
        info.transform = _GetROSAffineTransform(pbody->GetTransform());
        info.enabled = pbody->IsEnabled();
        info.dof = pbody->GetDOF();

        if( options & openraveros::BodyInfo::Req_JointValues ) {
            vector<dReal> vvalues; pbody->GetDOFValues(vvalues);
            info.jointvalues.resize(vvalues.size());
            for(size_t i = 0; i < vvalues.size(); ++i)
                info.jointvalues[i] = vvalues[i];
        }
        if( options & openraveros::BodyInfo::Req_Links ) {
            vector<Transform> vlinks; pbody->GetLinkTransformations(vlinks);
            info.links.resize(vlinks.size());
            for(size_t i = 0; i < vlinks.size(); ++i)
                info.links[i] = _GetROSAffineTransform(vlinks[i]);
        }
        if( options & openraveros::BodyInfo::Req_LinkNames ) {
            info.linknames.resize(pbody->GetLinks().size());
            for(size_t i = 0; i < pbody->GetLinks().size(); ++i)
                info.linknames[i] = pbody->GetLinks().at(i)->GetName();
        }
        if( options & openraveros::BodyInfo::Req_JointLimits ) {
            vector<dReal> vlower, vupper;
            pbody->GetDOFLimits(vlower,vupper);
            BOOST_ASSERT( vlower.size() == vupper.size() );
            info.lowerlimit.resize(vlower.size());
            info.upperlimit.resize(vupper.size());
            for(size_t i = 0; i < vlower.size(); ++i) {
                info.lowerlimit[i] = vlower[i];
                info.upperlimit[i] = vupper[i];
            }
        }
        if( options & openraveros::BodyInfo::Req_Names ) {
            info.filename = pbody->GetXMLFilename();
            info.name = pbody->GetName();
            info.type = pbody->GetXMLId();
        }
        if( options & openraveros::BodyInfo::Req_JointNames ) {
            info.jointnames.resize(pbody->GetJoints().size());
            for(size_t i = 0; i < pbody->GetJoints().size(); ++i)
                info.jointnames[i] = pbody->GetJoints().at(i)->GetName();
        }
    }

    void _GetROSRobotInfo(RobotBasePtr probot, openraveros::RobotInfo& info, uint32_t options)
    {
        _GetROSBodyInfo(probot,info.bodyinfo,options);

        info.activedof = probot->GetActiveDOF();
        info.activemanip = probot->GetActiveManipulatorIndex();

        if( options & openraveros::RobotInfo::Req_Manipulators ) {
            info.manips.resize(probot->GetManipulators().size());
            int index = 0;
            FOREACHC(itmanip, probot->GetManipulators()) {
                openraveros::Manipulator& rosmanip = info.manips[index++];
                rosmanip.baselink = !!(*itmanip)->GetBase() ? (*itmanip)->GetBase()->GetIndex() : -1;
                rosmanip.eelink = !!(*itmanip)->GetEndEffector() ? (*itmanip)->GetEndEffector()->GetIndex() : -1;
                rosmanip.tgrasp = _GetROSAffineTransform((*itmanip)->GetTransform());

                rosmanip.handjoints.resize((*itmanip)->GetGripperIndices().size());
                std::copy((*itmanip)->GetGripperIndices().begin(),(*itmanip)->GetGripperIndices().end(),rosmanip.handjoints.begin());

                rosmanip.armjoints.resize((*itmanip)->GetArmIndices().size());
                std::copy((*itmanip)->GetArmIndices().begin(),(*itmanip)->GetArmIndices().end(),rosmanip.armjoints.begin());
                if( !!(*itmanip)->GetIkSolver() ) {
                    rosmanip.iksolvername = (*itmanip)->GetIkSolver()->GetXMLId();
                }
                rosmanip.name = (*itmanip)->GetName();
            }
        }
        if( options & openraveros::RobotInfo::Req_Sensors ) {
            info.sensors.resize(probot->GetAttachedSensors().size()); int index = 0;
            FOREACHC(its, probot->GetAttachedSensors()) {
                openraveros::AttachedSensor& rossensor = info.sensors[index++];
                rossensor.name = (*its)->GetName();
                rossensor.attachedlink = (*its)->GetAttachingLink()->GetIndex();
                rossensor.trelative = _GetROSAffineTransform((*its)->GetRelativeTransform());

                if( !!(*its)->GetSensor() ) {
                    rossensor.tglobal = _GetROSAffineTransform((*its)->GetSensor()->GetTransform());
                    rossensor.type = (*its)->GetSensor()->GetXMLId();
                }
                else {
                    rossensor.tglobal = _GetROSAffineIdentity();
                    rossensor.type = "";
                }
            }
        }
        if( options & openraveros::RobotInfo::Req_ActiveDOFs ) {
            _GetROSActiveDOFs(probot, info.active);
        }
        if( options & openraveros::RobotInfo::Req_ActiveLimits ) {
            vector<dReal> vlower, vupper;
            probot->GetActiveDOFLimits(vlower,vupper);
            BOOST_ASSERT( vlower.size() == vupper.size() );
            info.activelowerlimit.resize(vlower.size());
            info.activeupperlimit.resize(vupper.size());
            for(size_t i = 0; i < vlower.size(); ++i) {
                info.activelowerlimit[i] = vlower[i];
                info.activeupperlimit[i] = vupper[i];
            }
        }
    }

    boost::function<bool(const string&,const string&)> _setviewer;
    map<int, PlannerBasePtr > _mapplanners;
    map<int, ModuleBasePtr > _mapmodules;
    map<int, GraphHandlePtr > _mapFigureIds;
    int _nNextFigureId, _nNextPlannerId, _nNextModuleId;
    float _fSimulationTimestep;
    bool _bDestroyThreads, _bWorking;

    /// viewer control variables
    ViewerBasePtr _pviewer;
    boost::thread _threadviewer;
    boost::mutex _mutexViewer, _mutexModules;
    boost::condition _conditionViewer;

    /// workers
    boost::thread _workerthread;
    boost::mutex _mutexWorker;
    list< boost::function<void()> > _listWorkers;
    boost::condition _conditionWorkers, _condHasWork;
    int _iWorkerId;

    // ros service stuff
    boost::shared_ptr<ros::NodeHandle> _ros;
    boost::thread _threadros;
    std::map<std::string,ros::ServiceServer> _mapservices;
};

ModuleBasePtr CreateROSServer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ModuleBasePtr(new ROSServer(penv,sinput));
}
