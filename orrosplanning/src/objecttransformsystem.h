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
#ifndef OBJECTTRANSFORM_SENSOR_SYSTEM
#define OBJECTTRANSFORM_SENSOR_SYSTEM

#include <tf/transform_listener.h>
#include <posedetection_msgs/ObjectDetection.h>
#include "rossensorsystem.h"

/// used to update objects through a mocap system
class ObjectTransformSystem : public ROSSensorSystem<posedetection_msgs::ObjectDetection>
{
public:
    static boost::shared_ptr<void> RegisterXMLReader(EnvironmentBasePtr penv)
    {
        return RegisterXMLReaderId(penv,"objecttransform");
    }

    ObjectTransformSystem(EnvironmentBasePtr penv)
        : ROSSensorSystem<posedetection_msgs::ObjectDetection>("objecttransform",penv), _nNextId(1)
    {
        __description = ":Interface Author: Rosen Diankov\n\nAdd objects from the ROS network using the posedetection_msgs/ObjectDetection message. The type field in the message is interpreted as an OpenRAVE-loadable object file.";
    }
    virtual ~ObjectTransformSystem() {
        _tflistener.reset();
    }

    virtual bool Init(istream& sinput)
    {
        _fThreshSqr = 0.05*0.05f;
        _listtopics.clear();
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "topic") {
                string topic;
                sinput >> topic;
                _listtopics.push_back(topic);
            }
            else if( cmd == "thresh" ) {
                sinput >> _fThreshSqr;
                _fThreshSqr *= _fThreshSqr;
            }
            else if( cmd == "robot" || cmd == "bodyoffset") {
                string name;
                sinput >> name;
                _pbodyoffset = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "matrixoffset") {
                TransformMatrix tmat;
                sinput >> tmat;
                _toffset = tmat;
            }
            else if( cmd == "expirationtime") {
                double expirationtime;
                sinput >> expirationtime;
                if( !!sinput )
                    _expirationtime = (uint64_t)(expirationtime*1000000);
            }
            else break;

            if( !sinput ) {
                RAVELOG_ERRORA("failed\n");
                return false;
            }
        }
        if( _listtopics.size() == 0 )
            _listtopics.push_back("ObjectDetection");
        return startsubscriptions();
    }

    virtual bool startsubscriptions()
    {
        if( !ROSSensorSystem<posedetection_msgs::ObjectDetection>::startsubscriptions() )
            return false;

        _tflistener.reset(new tf::TransformListener(*_ros));
        return true;
    }

    virtual void Destroy()
    {
        ROSSensorSystem<posedetection_msgs::ObjectDetection>::Destroy();
        _tflistener.reset();
    }

private:

    virtual void newdatacb(const boost::shared_ptr<posedetection_msgs::ObjectDetection const>& topicmsg)
    {
        list< SNAPSHOT > listbodies;
        list< pair<string,Transform> > listnewobjs;

        if( topicmsg->header.stamp == ros::Time(0) )
            RAVELOG_WARN(str(boost::format("incoming topic has 0 timestamp, bodies might not appear!\n")));

        {
            EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
            boost::mutex::scoped_lock lock(_mutex);
            BODIES mapbodies = _mapbodies;
            geometry_msgs::PoseStamped posestamped, poseout;
            Transform tbodyoffset;
            string strbodybaselink;
            bool bHasBodyTransform = false;

            if( !!_pbodyoffset && topicmsg->objects.size() > 0 ) {
                bHasBodyTransform = true;
                tbodyoffset = _pbodyoffset->GetTransform();
                strbodybaselink = _pbodyoffset->GetLinks().at(0)->GetName();
            }

            FOREACHC(itobj, topicmsg->objects) {
                boost::shared_ptr<BodyData> b;
                
                Transform tnew;

                // if on body, have to find the global transformation
                if( bHasBodyTransform && !!_tflistener ) {
                    posestamped.pose = GetPose(_toffset * GetTransform(itobj->pose));
                    posestamped.header = topicmsg->header;
                    
                    try {
                        _tflistener->transformPose(strbodybaselink, posestamped, poseout);
                        tnew = tbodyoffset * GetTransform(poseout.pose);
                    }
                    catch(std::runtime_error& ex) {
                        try {
                            // try getting the latest value by passing a 0 timestamp
                            posestamped.header.stamp = ros::Time();
                            _tflistener->transformPose(strbodybaselink, posestamped, poseout);
                            tnew = tbodyoffset * GetTransform(poseout.pose);
                        }
                        catch(std::runtime_error& ex) {
                            RAVELOG_WARNA("failed to get tf frames %s (body link:%s) for object %s\n",posestamped.header.frame_id.c_str(), strbodybaselink.c_str(), itobj->type.c_str());
                            continue;//tnew = GetTransform(itobj->pose);
                        }
                    }
                }
                else
                    tnew = GetTransform(itobj->pose);

                FOREACH(it, mapbodies) {
                    if( it->second->GetSid() == itobj->type ) {                            
                        // same type matched, need to check proximity
                        if( (it->second->GetRecentTransform().trans-tnew.trans).lengthsqr3() > _fThreshSqr )
                            continue;

                        b = it->second;
                        mapbodies.erase(it);
                        break;
                    }
                }

                if( !b ) {
                    listnewobjs.push_back(pair<string,Transform>(itobj->type,tnew));
                }
                else {
                    if( !b->IsEnabled() )
                        continue;
                    
                    listbodies.push_back(SNAPSHOT(b, tnew));
                }
            }
        }
        _UpdateBodies(listbodies);
        listbodies.clear();

        // try to add the left-over objects
        if( listnewobjs.size() > 0 ) {
            string data;
            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            FOREACH(itobj, listnewobjs) {
                try {
                    KinBodyPtr pbody = GetEnv()->ReadKinBodyXMLFile(resolveName(itobj->first));
                    if( !pbody ) {
                        RAVELOG_ERRORA(str(boost::format("failed to create object %s\n")%itobj->first.c_str()));
                        continue;
                    }

                    // append an id to the body
                    pbody->SetName(str(boost::format("%s%d")%pbody->GetName()%(_nNextId++)));
                    if( !GetEnv()->AddKinBody(pbody) ) {
                        RAVELOG_ERRORA(str(boost::format("failed to add body %s\n")%pbody->GetName()));
                        continue;
                    }

                    boost::shared_ptr<BodyData> b = boost::static_pointer_cast<BodyData>(AddKinBody(pbody, XMLReadableConstPtr()));
                    if( !b ) {
                        // create a dummy manage file and try again
                        boost::shared_ptr<XMLData> pdummydata(new XMLData(_xmlid));
                        pdummydata->sid = itobj->first;
                        b = boost::static_pointer_cast<BodyData>(AddKinBody(pbody, pdummydata));
                        if( !b ) {
                            GetEnv()->Remove(pbody);
                            continue;
                        }
                    }

                    SetRecentTransform(b,itobj->second);
                    
                    // put somewhere at infinity until UpdateBodies thread gets to it
                    pbody->SetTransform(Transform(Vector(1,0,0,0), Vector(10000,10000,10000)));
                    listbodies.push_back(SNAPSHOT(b,itobj->second));
                }
                catch (openrave_exception& e) {
                    RAVELOG_ERROR("failed to create object: %s (%s)\n",e.what());
                    continue;
                }
            }

            if( listbodies.size() > 0 ) {
                _UpdateBodies(listbodies);
            }
        }
    }

    boost::shared_ptr<tf::TransformListener> _tflistener;
    KinBodyPtr _pbodyoffset;
    Transform _toffset; ///< offset from tf frame
    dReal _fThreshSqr;
    int _nNextId;
};

#endif
