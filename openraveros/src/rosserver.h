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
// author: Rosen Diankov

#include "openraveros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

class ROSServer : public ProblemInstance
{
    inline boost::shared_ptr<ROSServer> shared_server() { return boost::static_pointer_cast<ROSServer>(shared_from_this()); }
    inline boost::shared_ptr<ROSServer const> shared_server_const() const { return boost::static_pointer_cast<ROSServer const>(shared_from_this()); }

public:
 ROSServer(int nSessionId, const boost::function<bool(const string&,const string&)>& setviewer, EnvironmentBasePtr penv, const string& physicsengine, const string& collisionchecker, const string& viewer)
     : ProblemInstance(penv), _setviewer(setviewer), _nSessionId(nSessionId), _nNextFigureId(1), _nNextPlannerId(1), _nNextProblemId(1), _iWorkerId(0) {
        _bThreadDestroyed = false;
        _bCloseClient = false;
        _bWorking = false;
        _fSimulationTimestep = 0.01;

        SetPhysicsEngine(physicsengine);
        SetCollisionChecker(collisionchecker);
        SetViewer(viewer);
    }
    virtual ~ROSServer() {
        Destroy();
    }
    
    virtual void Destroy()
    {
        _bCloseClient = true;
        Reset();
        // have to maintain a certain destruction order
        _threadviewer.join();
        _workerthread.join();
        
        _bCloseClient = false;
    }

    virtual void Reset()
    {
        // destroy environment specific state: problems, planners, figures
        {
            boost::mutex::scoped_lock lock(_mutexWorker);
            _mapplanners.clear();
            _mapFigureIds.clear();
            _listWorkers.clear();

            {
                boost::mutex::scoped_lock lock(_mutexProblems);
                FOREACH(itprob, _mapproblems)
                    itprob->second->GetEnv()->Remove(itprob->second);
                _mapproblems.clear();
            }
        }
        
        // wait for worker thread to stop
        while(_bWorking) {
            _conditionWorkers.notify_all();
            usleep(1000);
        }
    }

    virtual int main(const std::string& cmd)
    {
        Destroy();
        _bThreadDestroyed = false;
        _bCloseClient = false;
        _bWorking = false;
        _workerthread = boost::thread(boost::bind(&ROSServer::_WorkerThread,shared_server()));
        return 0;
    }

    /// worker thread called from the main environment thread
    virtual void _WorkerThread()
    {
        RAVELOG_DEBUG("starting ros worker thread\n");
        list<boost::function<void()> > listlocalworkers;

        while(!_bCloseClient) {
            {
                boost::mutex::scoped_lock lock(_mutexWorker);
                _condHasWork.wait(lock);
                if( _bCloseClient )
                    break;
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
                    RAVELOG_FATALA("%s\n",ex.what());
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
            while(!_bCloseClient) {
                _conditionWorkers.wait(lock);
                if( _iWorkerId >= iWorkerId+2 )
                    break;
                _condHasWork.notify_all();
            }
        }
    }

    /// viewer thread assuming you can create different viewers in their own therads
    virtual void ViewerThread(const string& strviewer)
    {
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _pviewer = GetEnv()->CreateViewer(strviewer);
            if( !!_pviewer ) {
                GetEnv()->AttachViewer(_pviewer);
                _pviewer->ViewerSetSize(1024,768);
            }
            _conditionViewer.notify_all();
        }

        if( !_pviewer )
            return;

        _pviewer->main(); // spin until quitfrommainloop is called
        RAVELOG_DEBUGA("destroying viewer\n");
        _pviewer.reset();
    }

    bool SetPhysicsEngine(const string& physicsengine)
    {
        PhysicsEngineBasePtr p = GetEnv()->CreatePhysicsEngine(physicsengine);
        if( !p )
            return false;

        GetEnv()->SetPhysicsEngine(p);
        return true;
    }

    bool SetCollisionChecker(const string& collisionchecker)
    {
        CollisionCheckerBasePtr p = GetEnv()->CreateCollisionChecker(collisionchecker);
        if( !p )
            return false;

        GetEnv()->SetCollisionChecker(p);
        return true;
    }

    bool SetViewer(const string& viewer)
    {
        if( viewer.size() == 0 )
            return true;

        if( !!_setviewer ) {
            stringstream ss;
            ss << "OpenRAVE - session " << _nSessionId;
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
            else
                RAVELOG_INFOA(str(boost::format("viewer %s successfully attached\n")%viewer));
        }

        return true;
    }

    //////////////
    // services //
    //////////////

    bool body_destroy_srv(body_destroy::Request& req, body_destroy::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return GetEnv()->Remove(pbody);
    }

    bool body_enable_srv(body_enable::Request& req, body_enable::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        pbody->Enable(req.enable);
        return true;
    }

    bool body_getaabb_srv(body_getaabb::Request& req, body_getaabb::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        OpenRAVE::AABB ab = pbody->ComputeAABB();
        res.box.center[0] = ab.pos.x; res.box.center[1] = ab.pos.y; res.box.center[2] = ab.pos.z;
        res.box.extents[0] = ab.extents.x; res.box.extents[1] = ab.extents.y; res.box.extents[2] = ab.extents.z;
        return true;
    }

    bool body_getaabbs_srv(body_getaabbs::Request& req, body_getaabbs::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

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

    bool body_getdof_srv(body_getdof::Request& req, body_getdof::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        res.dof = pbody->GetDOF();
        return true;
    }

    bool body_getjointvalues_srv(body_getjointvalues::Request& req, body_getjointvalues::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

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

    bool body_setjointvalues_srv(body_setjointvalues::Request& req, body_setjointvalues::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

        if( pbody->GetDOF() == 0 )
            return true;

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

    bool body_settransform_srv(body_settransform::Request& req, body_settransform::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody )
            return false;

        Transform t = FromAffineTransform(req.transform);
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        pbody->SetTransform(t);

        if( pbody->IsRobot() ) {
            RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);
            if( !!probot->GetController() )
                probot->GetController()->SetPath(TrajectoryBasePtr());
        }

        return true;
    }

    bool env_checkcollision_srv(env_checkcollision::Request& req, env_checkcollision::Response& res)
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
                               (!!report->plink1?report->plink1->GetParent()->GetName():"(NULL)")%
                               (!!report->plink1?report->plink1->GetName():"(NULL)")%
                               (!!report->plink2?report->plink2->GetParent()->GetName():"(NULL)")%
                               (!!report->plink2?report->plink2->GetName():"(NULL)")));
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

    bool env_closefigures_srv(env_closefigures::Request& req, env_closefigures::Response& res)
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

    bool env_createbody_srv(env_createbody::Request& req, env_createbody::Response& res)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        KinBodyPtr pbody = GetEnv()->CreateKinBody();

        if( req.file.size() > 0 ) {
            if( !pbody->InitFromFile(req.file, std::list<std::pair<std::string,std::string> >()) )
                return false;
        }

        pbody->SetName(req.name);

        if( !GetEnv()->AddKinBody(pbody) )
            return false;

        res.bodyid = pbody->GetEnvironmentId();
        return true;
    }

    bool env_createplanner_srv(env_createplanner::Request& req, env_createplanner::Response& res)
    {
        PlannerBasePtr pplanner = GetEnv()->CreatePlanner(req.plannertype);
        if( !pplanner )
            return false;

        _mapplanners[++_nNextPlannerId] = pplanner;
        res.plannerid = _nNextPlannerId;
        return true;
    }

    void LoadProblemWorker(int& retval, ProblemInstancePtr prob, const string& args  )
    {
        retval = prob->GetEnv()->LoadProblem(prob, args);
    }

    bool env_createproblem_srv(env_createproblem::Request& req, env_createproblem::Response& res)
    {
        ProblemInstancePtr pproblem = GetEnv()->CreateProblem(req.problemtype);
        if( !pproblem )
            return false;

        boost::mutex::scoped_lock lock(_mutexProblems);

        if( req.destroyduplicates ) {
            map<int, ProblemInstancePtr >::iterator itprob = _mapproblems.begin();
            while(itprob != _mapproblems.end()) {
                if( itprob->second->GetXMLId() == req.problemtype ) {
                    RAVELOG_INFOA(str(boost::format("deleting duplicate problem %s\n")%req.problemtype));
                    if( !GetEnv()->Remove(itprob->second) )
                        RAVELOG_WARNA(str(boost::format("failed to remove problem %s\n")%itprob->second->GetXMLId()));
                    
                    _mapproblems.erase(itprob++);
                }
                else
                    ++itprob;
            }
        }

        int retval=0;
        AddWorker(boost::bind(&ROSServer::LoadProblemWorker,shared_server(),boost::ref(retval), pproblem,req.args),true);

        if( retval != 0 ) {
            RAVELOG_WARNA(str(boost::format("failed to load problem %s with args %s\n")%req.problemtype%req.args));
            return false;
        }
        
        _mapproblems[++_nNextProblemId] = pproblem;
        res.problemid = _nNextProblemId;
        return true;
    }

    bool env_createrobot_srv(env_createrobot::Request& req, env_createrobot::Response& res)
    {
        RobotBasePtr probot = GetEnv()->CreateRobot(req.type);
        if( !probot )
            return false;

        if( req.file.size() > 0 ) {
            if( !probot->InitFromFile(req.file,std::list<std::pair<std::string,std::string> >()) )
                return false;
        }

        probot->SetName(req.name);

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        if( !GetEnv()->AddRobot(probot) )
            return false;

        res.bodyid = probot->GetEnvironmentId();
        return true;
    }

    bool env_destroyproblem_srv(env_destroyproblem::Request& req, env_destroyproblem::Response& res)
    {
        boost::mutex::scoped_lock lock(_mutexProblems);
        map<int, ProblemInstancePtr >::iterator itprob = _mapproblems.find(req.problemid);
        if( itprob == _mapproblems.end() )
            return false;
        
        ProblemInstancePtr prob = itprob->second;
        _mapproblems.erase(itprob);

        if( !GetEnv()->Remove(itprob->second) ) {
            RAVELOG_WARNA("failed to remove problem\n");
            return false;
        }
        return true;
    }

    void FillKinBodyInfo(KinBodyPtr pbody, BodyInfo& info, uint32_t options)
    {
        info.bodyid = pbody->GetEnvironmentId();
        info.transform = GetAffineTransform(pbody->GetTransform());
        info.enabled = pbody->IsEnabled();
        info.dof = pbody->GetDOF();

        if( options & BodyInfo::Req_JointValues ) {
            vector<dReal> vvalues; pbody->GetDOFValues(vvalues);
            info.jointvalues.resize(vvalues.size());
            for(size_t i = 0; i < vvalues.size(); ++i)
                info.jointvalues[i] = vvalues[i];
        }
        if( options & BodyInfo::Req_Links ) {
            vector<Transform> vlinks; pbody->GetBodyTransformations(vlinks);
            info.links.resize(vlinks.size());
            for(size_t i = 0; i < vlinks.size(); ++i)
                info.links[i] = GetAffineTransform(vlinks[i]);
        }
        if( options & BodyInfo::Req_LinkNames ) {
            info.linknames.resize(pbody->GetLinks().size());
            for(size_t i = 0; i < pbody->GetLinks().size(); ++i)
                info.linknames[i] = pbody->GetLinks().at(i)->GetName();
        }
        if( options & BodyInfo::Req_JointLimits ) {
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
        if( options & BodyInfo::Req_Names ) {
            info.filename = pbody->GetXMLFilename();
            info.name = pbody->GetName();
            info.type = pbody->GetXMLId();
        }
        if( options & BodyInfo::Req_JointNames ) {
            info.jointnames.resize(pbody->GetJoints().size());
            for(size_t i = 0; i < pbody->GetJoints().size(); ++i)
                info.jointnames[i] = pbody->GetJoints().at(i)->GetName();
        }
    }

    bool env_getbodies_srv(env_getbodies::Request& req, env_getbodies::Response& res)
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
            FillKinBodyInfo(*itbody, info, req.options);
        }
        
        return true;
    }

    bool env_getbody_srv(env_getbody::Request& req, env_getbody::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetKinBody(req.name);
        if( !pbody )
            return false;
        res.bodyid = pbody->GetEnvironmentId();
        return true;
    }

    void FillActiveDOFs(RobotBasePtr probot, openraveros::ActiveDOFs& active)
    {
        active.affine = probot->GetAffineDOF();
        active.joints.resize(probot->GetActiveDOFIndices().size());
        for(size_t i = 0; i < probot->GetActiveDOFIndices().size(); ++i)
            active.joints[i] = probot->GetActiveDOFIndices()[i];
        active.rotationaxis[0] = probot->GetAffineRotationAxis().x;
        active.rotationaxis[1] = probot->GetAffineRotationAxis().y;
        active.rotationaxis[2] = probot->GetAffineRotationAxis().z;
    }

    void FillRobotInfo(RobotBasePtr probot, RobotInfo& info, uint32_t options)
    {
        FillKinBodyInfo(probot,info.bodyinfo,options);

        info.activedof = probot->GetActiveDOF();
        info.activemanip = probot->GetActiveManipulatorIndex();

        if( options & RobotInfo::Req_Manipulators ) {
            info.manips.resize(probot->GetManipulators().size()); int index = 0;
            FOREACHC(itmanip, probot->GetManipulators()) {
                openraveros::Manipulator& rosmanip = info.manips[index++];
                rosmanip.baselink = !!(*itmanip)->GetBase() ? (*itmanip)->GetBase()->GetIndex() : -1;
                rosmanip.eelink = !!(*itmanip)->GetEndEffector() ? (*itmanip)->GetEndEffector()->GetIndex() : -1;
                rosmanip.tgrasp = GetAffineTransform((*itmanip)->GetGraspTransform());
                
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
        if( options & RobotInfo::Req_Sensors ) {
            info.sensors.resize(probot->GetAttachedSensors().size()); int index = 0;
            FOREACHC(its, probot->GetAttachedSensors()) {
                openraveros::AttachedSensor& rossensor = info.sensors[index++];
                rossensor.name = (*its)->GetName();
                rossensor.attachedlink = (*its)->GetAttachingLink()->GetIndex();
                rossensor.trelative = GetAffineTransform((*its)->GetRelativeTransform());

                if( !!(*its)->GetSensor() ) {
                    rossensor.tglobal = GetAffineTransform((*its)->GetSensor()->GetTransform());
                    rossensor.type = (*its)->GetSensor()->GetXMLId();
                }
                else {
                    rossensor.tglobal = GetAffineIdentity();
                    rossensor.type = "";
                }
            }
        }
        if( options & RobotInfo::Req_ActiveDOFs ) {
            FillActiveDOFs(probot, info.active);
        }
        if( options & RobotInfo::Req_ActiveLimits ) {
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

    bool env_getrobots_srv(env_getrobots::Request& req, env_getrobots::Response& res)
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
            FillRobotInfo(*itrobot, info, req.options);
        }
        
        return true;
    }

    bool env_loadplugin_srv(env_loadplugin::Request& req, env_loadplugin::Response& res)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return GetEnv()->LoadPlugin(req.filename);
    }

    bool env_loadscene_srv(env_loadscene::Request& req, env_loadscene::Response& res)
    {
        if( req.resetscene )
            GetEnv()->Reset();        
        if( req.filename.size() > 0 )
            return GetEnv()->Load(req.filename);

        return true;
    }

    bool env_plot_srv(env_plot::Request& req, env_plot::Response& res)
    {
        bool bOneColor = req.colors.size() != req.points.size();
        float falpha = max(0.0f, 1.0f-req.transparency);
        falpha = min(1.0f,falpha);
        RaveVector<float> vOneColor(1.0f,0.5f,0.5f,falpha);
        if( req.colors.size() >= 3 )
            vOneColor = RaveVector<float>(req.colors[0], req.colors[1], req.colors[2],falpha);
        
        EnvironmentBase::GraphHandlePtr figure;
        switch(req.drawtype) {
        case env_plot::Request::Draw_Point:
            if( bOneColor )
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor, 0);
            else
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 0);
            break;
        case env_plot::Request::Draw_LineStrip:
            if( bOneColor )
                figure = GetEnv()->drawlinestrip(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor);
            else
                figure = GetEnv()->drawlinestrip(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0]);
            break;
        case env_plot::Request::Draw_LineList:
            if( bOneColor )
                figure = GetEnv()->drawlinelist(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor);
            else
                figure = GetEnv()->drawlinelist(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0]);
            break;
        case env_plot::Request::Draw_TriList:
            //if( bOneColor )
            figure = GetEnv()->drawtrimesh(&req.points[0],3*sizeof(req.points[0]), NULL, req.points.size()/9, vOneColor);
            //else
                //figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 0);
            break;
        case env_plot::Request::Draw_Sphere:
            if( bOneColor )
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor, 1);
            else
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 1);
            break;
        default:
            return false;
        }

        if( !figure )
            return false;
        
        _mapFigureIds[++_nNextFigureId] = figure;
        res.figureid = _nNextFigureId;
        return true;
    }

    bool env_raycollision_srv(env_raycollision::Request& req, env_raycollision::Response& res)
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

    bool env_set_srv(env_set::Request& req, env_set::Response& res)
    {
        if( req.setmask & env_set::Request::Set_Simulation ) {
            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            switch(req.sim_action) {
            case env_set::Request::SimAction_Start:
                if( req.sim_timestep > 0 )
                    _fSimulationTimestep = req.sim_timestep;
                GetEnv()->StartSimulation(_fSimulationTimestep);
                break;
            case env_set::Request::SimAction_Stop:
                GetEnv()->StopSimulation();
                break;
            case env_set::Request::SimAction_Timestep:
                _fSimulationTimestep = req.sim_timestep;
                GetEnv()->StartSimulation(_fSimulationTimestep);
                break;
            }
        }
        if( req.setmask & env_set::Request::Set_PhysicsEngine )
            SetPhysicsEngine(req.physicsengine);
        if( req.setmask & env_set::Request::Set_CollisionChecker )
            SetCollisionChecker(req.collisionchecker);
        if( req.setmask & env_set::Request::Set_Gravity ) {
            GetEnv()->GetPhysicsEngine()->SetGravity(Vector(req.gravity[0],req.gravity[1],req.gravity[2]));
        }
        if( req.setmask & env_set::Request::Set_DebugLevel ) {
            map<string,DebugLevel> mlevels;
            mlevels["fatal"] = Level_Fatal;
            mlevels["error"] = Level_Error;
            mlevels["info"] = Level_Info;
            mlevels["warn"] = Level_Warn;
            mlevels["debug"] = Level_Debug;
            mlevels["verbose"] = Level_Verbose;
            DebugLevel level = GetEnv()->GetDebugLevel();
            if( mlevels.find(req.debuglevel) != mlevels.end() )
                level = mlevels[req.debuglevel];
            else {
                stringstream ss(req.debuglevel);
                int nlevel;
                ss >> nlevel;
                if( !!ss )
                    level = (DebugLevel)nlevel;
            }
            GetEnv()->SetDebugLevel(level);
        }
        if( req.setmask & env_set::Request::Set_Viewer ) {
            GetEnv()->AttachViewer(ViewerBasePtr());
            SetViewer(req.viewer);
        }
        if( req.setmask & env_set::Request::Set_ViewerDims ) {
            if( !!GetEnv()->GetViewer() )
                GetEnv()->GetViewer()->ViewerSetSize(req.viewerwidth,req.viewerheight);
        }

        return true;
    }

    bool env_triangulate_srv(env_triangulate::Request& req, env_triangulate::Response& res)
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

    bool env_wait_srv(env_wait::Request& req, env_wait::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);
        ControllerBasePtr pcontroller = probot->GetController();
        if( !pcontroller )
            return false;

        uint32_t timeout = (uint32_t)(req.timeout*1000.0f);
        while( !pcontroller->IsDone() ) {
            usleep(400);
            if( timeout > 0 ) {
                if( --timeout == 0 )
                    break;
            }
            if( _bCloseClient )
                return false;
        }

        res.isdone = pcontroller->IsDone();
        return true;
    }

    bool planner_init_srv(planner_init::Request& req, planner_init::Response& res)
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

    bool planner_plan_srv(planner_plan::Request& req, planner_plan::Response& res)
    {
        map<int, PlannerBasePtr >::iterator itplanner = _mapplanners.find(req.plannerid);
        if( itplanner == _mapplanners.end() )
            return false;

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        TrajectoryBasePtr traj = GetEnv()->CreateTrajectory(itplanner->second->GetParameters()->GetDOF());
        
        RAVELOG_INFOA("starting to plan");
        if( !itplanner->second->PlanPath(traj) ) {
            RAVELOG_INFOA("plan failed\n");
            return false;
        }
        
        RAVELOG_INFOA("plan succeeded\n");

        //FillActiveDOFs(itplanner->second->GetRobot(), res.trajectory.active);
        res.trajectory.points.resize(traj->GetPoints().size()); int index = 0;
        FOREACHC(itpoint, traj->GetPoints()) {
            openraveros::TrajectoryPoint& rospt = res.trajectory.points[index++];
            rospt.timestamp = itpoint->time;
            rospt.position.resize(itpoint->q.size());
            for(size_t i = 0; i < itpoint->q.size(); ++i)
                rospt.position[i] = itpoint->q[i];
            rospt.velocity.resize(itpoint->qdot.size());
            for(size_t i = 0; i < itpoint->qdot.size(); ++i)
                rospt.velocity[i] = itpoint->qdot[i];
            rospt.transform = GetAffineTransform(itpoint->trans);
        }

        return true;
    }

    bool problem_sendcommand_srv(problem_sendcommand::Request& req, problem_sendcommand::Response& res)
    {
        boost::mutex::scoped_lock lock(_mutexProblems);
        map<int, ProblemInstancePtr >::iterator itprob = _mapproblems.find(req.problemid);
        if( itprob == _mapproblems.end() )
            return false;

        try {
            stringstream sout;
            stringstream sin(req.cmd);
            if( itprob->second->SendCommand(sout,sin) ) {
                res.output = sout.str();
                return true;
            }
        }
        catch(const openrave_exception& ex) {
            RAVELOG_FATAL("%s\n",ex.what());
        }

        return false;
    }
    
    bool robot_controllersend_srv(robot_controllersend::Request& req, robot_controllersend::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);
        ControllerBasePtr pcontroller = probot->GetController();
        if( !pcontroller )
            return false;

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        stringstream sout;
        stringstream sin(req.cmd);
        if( pcontroller->SendCommand(sout,sin) ) {
            res.output = sout.str();
            return true;
        }
        return false;
    }

    bool robot_controllerset_srv(robot_controllerset::Request& req, robot_controllerset::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);

        ControllerBasePtr pcontroller = GetEnv()->CreateController(req.controllername);
        if( !pcontroller )
            return false;
        
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        if( probot->SetController(pcontroller,req.controllerargs) )
            return true;
        return false;
    }

    bool robot_getactivevalues_srv(robot_getactivevalues::Request& req, robot_getactivevalues::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);

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

    bool robot_sensorgetdata_srv(robot_sensorgetdata::Request& req, robot_sensorgetdata::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);

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

            TransformMatrix tP = tKK * pcameradata->t.inverse();

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

    bool robot_sensorsend_srv(robot_sensorsend::Request& req, robot_sensorsend::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);

        try {
            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            RobotBase::AttachedSensorPtr sensor = probot->GetAttachedSensors().at(req.sensorindex);    
        
            stringstream sout;
            stringstream sin;
            sin << req.cmd << " " << req.args;
            if( sensor->GetSensor()->SendCommand(sout,sin)) {
                res.out = sout.str();
                return true;
            }

            RAVELOG_ERRORA(str(boost::format("Robot %s sensor %d failed: \"%s\"\n")%probot->GetName()%req.sensorindex%req.cmd));
        }
        catch(const openrave_exception& ex) {
            RAVELOG_ERRORA(str(boost::format("Robot %s sensor %d exception: %s\n")%probot->GetName()%req.sensorindex%ex.what()));
        }

        return false;
    }

    void RobotSetActiveDOFs(RobotBasePtr probot, openraveros::ActiveDOFs& active)
    {
        vector<int> vjointindices(active.joints.size());
        for(size_t i = 0; i < active.joints.size(); ++i)
            vjointindices[i] = active.joints[i];
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        probot->SetActiveDOFs(vjointindices, active.affine, Vector(active.rotationaxis[0], active.rotationaxis[1], active.rotationaxis[2]));
    }
    
    bool robot_setactivedofs_srv(robot_setactivedofs::Request& req, robot_setactivedofs::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);
        RobotSetActiveDOFs(probot, req.active);
        return true;
    }

    bool robot_setactivevalues_srv(robot_setactivevalues::Request& req, robot_setactivevalues::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);
        if( probot->GetActiveDOF() == 0 )
            return false;

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        vector<dReal> vvalues;

        if( req.indices.size() > 0 ) {
            if( req.indices.size() != req.values.size() ) {
                RAVELOG_WARNA(str(boost::format("indices (%d) different size than joint values (%d)\n")%req.indices.size()%req.values.size()));
                return false;
            }

            probot->GetActiveDOFValues(vvalues);
            for(size_t i = 0; i < req.indices.size(); ++i)
                vvalues[req.indices[i]] = req.values[i];
        }
        else {
            if( probot->GetActiveDOF() != (int)req.values.size() ) {
                RAVELOG_WARNA(str(boost::format("body dof (%d) not equal to values (%d)")%probot->GetDOF()%req.values.size()));
                return false;
            }

            vvalues.reserve(req.values.size());
            FOREACHC(it,req.values)
                vvalues.push_back(*it);
        }

        probot->SetActiveDOFValues(vvalues, true);

        if( !!probot->GetController() ) {
            probot->GetDOFValues(vvalues); // reget the values since they'll go through the joint limits
            probot->GetController()->SetDesired(vvalues);
        }

        return true;
    }

    bool robot_starttrajectory_srv(robot_starttrajectory::Request& req, robot_starttrajectory::Response& res)
    {
        KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(req.bodyid);
        if( !pbody || !pbody->IsRobot() )
            return false;
        RobotBasePtr probot = boost::static_pointer_cast<RobotBase>(pbody);

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        RobotBase::RobotStateSaver saver(probot);

        openraveros::ActiveDOFs active = req.trajectory.active;
        vector<int> vjointindices(active.joints.size());
        for(size_t i = 0; i < active.joints.size(); ++i)
            vjointindices[i] = active.joints[i];
        
        probot->SetActiveDOFs(vjointindices, active.affine, Vector(active.rotationaxis[0], active.rotationaxis[1], active.rotationaxis[2]));

        TrajectoryBasePtr pfulltraj = GetEnv()->CreateTrajectory(probot->GetDOF());
        TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(probot->GetActiveDOF());
        
        TrajectoryBase::TPOINT pt; pt.q.resize(probot->GetActiveDOF());
        pt.trans = probot->GetTransform();

        bool bOverwriteTransforms = !(req.options & robot_starttrajectory::Request::Traj_UseTransforms);
        bool bUseTorques = !!(req.options & robot_starttrajectory::Request::Traj_UseTorques);
        bool bAutoCalcTiming = !(req.options & robot_starttrajectory::Request::Traj_UseTimestamps);

        FOREACH(it, req.trajectory.points) {
            BOOST_ASSERT( it->position.size() == pt.q.size() );
            std::copy(it->position.begin(),it->position.end(),pt.q.begin());

            pt.qdot.resize(it->velocity.size());
            std::copy(it->velocity.begin(),it->velocity.end(),pt.qdot.begin());

            if( bUseTorques ) {
                pt.qtorque.resize(it->torque.size());
                std::copy(it->torque.begin(),it->torque.end(),pt.qtorque.begin());
            }

            if( !bOverwriteTransforms )
                pt.trans = FromAffineTransform(it->transform);
            if( !bAutoCalcTiming )
                pt.time = it->timestamp;
            ptraj->AddPoint(pt);
        }

        probot->GetFullTrajectoryFromActive(pfulltraj, ptraj, bOverwriteTransforms);
        if( !pfulltraj->CalcTrajTiming(probot, pfulltraj->GetInterpMethod(), bAutoCalcTiming, false) )
            return false;

        probot->SetMotion(pfulltraj);
        return true;
    }

private:
    AffineTransformMatrix GetAffineTransform(const TransformMatrix& tm)
    {
        AffineTransformMatrix am;
        am.m[0] = tm.m[0]; am.m[3] = tm.m[1]; am.m[6] = tm.m[2];
        am.m[1] = tm.m[4]; am.m[4] = tm.m[5]; am.m[7] = tm.m[6];
        am.m[2] = tm.m[8]; am.m[5] = tm.m[9]; am.m[8] = tm.m[10];
        am.m[9] = tm.trans.x; am.m[10] = tm.trans.y; am.m[11] = tm.trans.z;
        return am;
    }

    TransformMatrix FromAffineTransform(const AffineTransformMatrix& am)
    {
        TransformMatrix tm;
        tm.m[0] = am.m[0]; tm.m[1] = am.m[3]; tm.m[2] = am.m[6];
        tm.m[4] = am.m[1]; tm.m[5] = am.m[4]; tm.m[6] = am.m[7];
        tm.m[8] = am.m[2]; tm.m[9] = am.m[5]; tm.m[10] = am.m[8];
        tm.trans.x = am.m[9]; tm.trans.y = am.m[10]; tm.trans.z = am.m[11];
        return tm;
    }

    AffineTransformMatrix GetAffineIdentity()
    {
        AffineTransformMatrix am;
        am.m[0] = 1; am.m[3] = 0; am.m[6] = 0; am.m[9] = 0;
        am.m[1] = 0; am.m[4] = 1; am.m[7] = 0; am.m[10] = 0;
        am.m[2] = 0; am.m[5] = 0; am.m[8] = 1; am.m[11] = 0;
        return am;
    }

    boost::function<bool(const string&,const string&)> _setviewer;
    map<int, PlannerBasePtr > _mapplanners;
    map<int, ProblemInstancePtr > _mapproblems;
    map<int, EnvironmentBase::GraphHandlePtr > _mapFigureIds;
    int _nSessionId;
    int _nNextFigureId, _nNextPlannerId, _nNextProblemId;
    float _fSimulationTimestep;
    bool _bThreadDestroyed, _bCloseClient, _bWorking;

    /// viewer control variables
    ViewerBasePtr _pviewer;
    boost::thread _threadviewer;
    boost::mutex _mutexViewer, _mutexProblems;
    boost::condition _conditionViewer;
    
    /// workers
    boost::thread _workerthread;
    boost::mutex _mutexWorker;
    list< boost::function<void()> > _listWorkers;
    boost::condition _conditionWorkers, _condHasWork;
    int _iWorkerId;
};
