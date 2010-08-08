// Software License Agreement (BSD License)
// Copyright (c) 2008, Rosen Diankov
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
#ifndef COLLISIONMAP_MOCAP_SYSTEM
#define COLLISIONMAP_MOCAP_SYSTEM

#include "rossensorsystem.h"
#include <mapping_msgs/CollisionMap.h>

class CollisionMapSystem : public ROSSensorSystem<mapping_msgs::CollisionMap>
{
public:
    static boost::shared_ptr<void> RegisterXMLReader(EnvironmentBasePtr penv)
    {
        return RegisterXMLReaderId(penv,"collisionmap");
    }

    CollisionMapSystem(EnvironmentBasePtr penv)
        : ROSSensorSystem<mapping_msgs::CollisionMap>("collisionmap",penv), _fPrunePadding(0), _nNextId(1), _bPruneCollisions(false)
    {
    }
    virtual ~CollisionMapSystem() {
    }

    virtual bool Init(istream& sinput)
    {
        _listtopics.clear();
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "topic" ) {
                string topic;
                sinput >> topic;
                _listtopics.push_back(topic);
            }
            else if( cmd == "prunecollisions")
                sinput >> _bPruneCollisions >> _fPrunePadding;
            else if( cmd == "robot" || cmd == "bodyoffset" ) {
                string name;
                sinput >> name;
                _pbodyoffset = GetEnv()->GetKinBody(name);
                if( !_pbodyoffset )
                    RAVELOG_WARN(str(boost::format("failed to get %s body\n")%name));
            }
            else if( cmd == "expirationtime") {
                double expirationtime;
                sinput >> expirationtime;
                if( !!sinput )
                    _expirationtime = (uint64_t)(expirationtime*1000000);
            }
            else
                break;
            
            if( !sinput ) {
                RAVELOG_ERRORA("failed\n");
                return false;
            }
        }

        if( _listtopics.size() == 0 )
            _listtopics.push_back("collision_map");
        
        return startsubscriptions();
    }

private:
    virtual bool startsubscriptions()
    {
        if( !ROSSensorSystem<mapping_msgs::CollisionMap>::startsubscriptions() )
            return false;

        _tflistener.reset(new tf::TransformListener(*_ros));
        return true;
    }

    virtual void Destroy()
    {
        ROSSensorSystem<mapping_msgs::CollisionMap>::Destroy();
        _tflistener.reset();
    }

    virtual void newdatacb(const boost::shared_ptr<mapping_msgs::CollisionMap const>& topicmsg)
    {
        KinBodyPtr pbody;
        {
            EnvironmentMutex::scoped_lock envlock(GetEnv()->GetMutex());

            Transform tcollision;
            string strbodybaselink;
            bool bHasBodyTransform = false;

            if( !!_pbodyoffset ) {
                bHasBodyTransform = true;
                tcollision = _pbodyoffset->GetTransform();
                strbodybaselink = _pbodyoffset->GetLinks().front()->GetName();
            }

            if( bHasBodyTransform && !!_tflistener ) {
                tf::StampedTransform bttransform;

                try {
                    _tflistener->lookupTransform(strbodybaselink, topicmsg->header.frame_id, topicmsg->header.stamp, bttransform);
                    tcollision = tcollision * GetTransform(bttransform);
                }
                catch(tf::TransformException& ex) {
                    try {
                        _tflistener->lookupTransform(strbodybaselink, topicmsg->header.frame_id, ros::Time(), bttransform);
                        tcollision = tcollision * GetTransform(bttransform);
                    }
                    catch(tf::TransformException& ex) {
                        RAVELOG_WARNA("failed to get tf frames %s (body link:%s)\n", topicmsg->header.frame_id.c_str(), strbodybaselink.c_str());
                        return;
                    }
                }
            }

            list<KinBodyPtr> listcheckbodies;
            if( _bPruneCollisions ) {
                vector<KinBodyPtr> vallbodies;
                GetEnv()->GetBodies(vallbodies);
                FOREACH(itbody,vallbodies) {
                    if( !!(*itbody)->GetManageData() && (*itbody)->GetManageData()->GetSystem() != shared_system() )
                        listcheckbodies.push_back(*itbody);
                }
                // add all grabbed bodies
                vector<RobotBasePtr> vallrobots;
                GetEnv()->GetRobots(vallrobots);
                FOREACH(itrobot,vallrobots) {
                    (*itrobot)->GetGrabbed(vallbodies);
                    FOREACH(itbody,vallbodies) {
                        if( find(listcheckbodies.begin(),listcheckbodies.end(),*itbody) == listcheckbodies.end() )
                            listcheckbodies.push_back(*itbody);
                    }
                }
            }

            pbody = GetEnv()->CreateKinBody();

            _vobbs.resize(0); _vobbs.resize(topicmsg->boxes.size());
            vector<OBB>::iterator itobb = _vobbs.begin();
            TransformMatrix tm;
            FOREACH(itmsgab, topicmsg->boxes) {
                if( _bPruneCollisions && listcheckbodies.size() > 0 ) {
                    if( !_pbodytestbox ) {
                        vector<AABB> vabs(1);
                        vabs[0].extents = Vector(itmsgab->extents.x+_fPrunePadding, itmsgab->extents.y+_fPrunePadding, itmsgab->extents.z+_fPrunePadding);
                        _pbodytestbox = GetEnv()->CreateKinBody();
                        _pbodytestbox->InitFromBoxes(vabs,false);
                        _pbodytestbox->SetName(str(boost::format("testbox%d")%RaveRandomInt()));
                        GetEnv()->AddKinBody(_pbodytestbox);
                    }
                    Transform t;
                    t.rot = quatFromAxisAngle(Vector(itmsgab->axis.x,itmsgab->axis.y,itmsgab->axis.z),itmsgab->angle);
                    t.trans = Vector(itmsgab->center.x, itmsgab->center.y, itmsgab->center.z);
                    _pbodytestbox->SetTransform(t);
                    bool bPrune=false;
                    FOREACH(itcolbody,listcheckbodies) {
                        if( GetEnv()->CheckCollision(KinBodyConstPtr(_pbodytestbox),KinBodyConstPtr(*itcolbody)) ) {
                            bPrune = true;
                            break;
                        }
                    }
                    if( bPrune )
                        continue;
                }
                itobb->pos = Vector(itmsgab->center.x, itmsgab->center.y, itmsgab->center.z);
                itobb->extents = Vector(itmsgab->extents.x, itmsgab->extents.y, itmsgab->extents.z);
                tm = matrixFromAxisAngle(Vector(itmsgab->axis.x,itmsgab->axis.y,itmsgab->axis.z),itmsgab->angle);
                itobb->right = Vector(tm.m[0],tm.m[4],tm.m[8]);
                itobb->up = Vector(tm.m[1],tm.m[5],tm.m[9]);
                itobb->dir = Vector(tm.m[2],tm.m[6],tm.m[10]);
                ++itobb;
            }

            if( !!_pbodytestbox ) {
                GetEnv()->RemoveKinBody(_pbodytestbox);
                _pbodytestbox.reset();
            }

            if( !pbody->InitFromBoxes(_vobbs, true) ) {
                RAVELOG_ERRORA("failed to create collision map\n");
                return;
            }

            // append an id to the body
            pbody->SetName(str(boost::format("CollisionMap%d")%_nNextId++));

            // add the new kinbody
            if( !GetEnv()->AddKinBody(pbody) ) {
                RAVELOG_ERRORA(str(boost::format("failed to add body %s\n")%pbody->GetName()));
                return;
            }
            pbody->SetTransform(tcollision);
        }

        {
            EnvironmentMutex::scoped_lock envlock(GetEnv()->GetMutex());
            boost::mutex::scoped_lock lock(_mutex);

            // remove all unlocked bodies
            BODIES::iterator itbody = _mapbodies.begin();
            while(itbody != _mapbodies.end()) {
                if( !itbody->second->IsLocked() ) {
                    KinBody::LinkPtr plink = itbody->second->GetOffsetLink();
                    GetEnv()->RemoveKinBody(plink->GetParent());
                    _mapbodies.erase(itbody++);
                }
                else
                    ++itbody;
            }
        }

        boost::shared_ptr<XMLData> pdata(new XMLData(_xmlid));
        pdata->strOffsetLink = pbody->GetLinks().front()->GetName();
        boost::shared_ptr<BodyData> b = boost::static_pointer_cast<BodyData>(AddKinBody(pbody, pdata));
        if( !b ) {
            RAVELOG_ERRORA("removing/destroying kinbody\n");
            GetEnv()->RemoveKinBody(pbody);
        }
    }

    Transform GetTransform(const btTransform& bt)
    {
        btQuaternion q = bt.getRotation();
        btVector3 o = bt.getOrigin();
        return Transform(Vector(q.w(),q.x(),q.y(),q.z()),Vector(o.x(),o.y(),o.z()));
    }

    boost::shared_ptr<tf::TransformListener> _tflistener;
    KinBodyPtr _pbodyoffset, _pbodytestbox;
    dReal _fPrunePadding;
    vector<OBB> _vobbs;
    int _nNextId;
    bool _bPruneCollisions;
};

#endif
