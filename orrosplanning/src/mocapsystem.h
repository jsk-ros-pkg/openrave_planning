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
// author: Rosen Diankov
#ifndef PHASESPACE_MOCAP_SYSTEM
#define PHASESPACE_MOCAP_SYSTEM

#include "rossensorsystem.h"
#include "mocap_msgs/MocapSnapshot.h"

// used to update objects through a mocap system
class ROSMocapSystem : public ROSSensorSystem<mocap_msgs::MocapSnapshot>
{
public:
    static boost::shared_ptr<void> RegisterXMLReader(EnvironmentBasePtr penv)
    {
        return RegisterXMLReaderId(penv,"phasespace");
    }

    ROSMocapSystem(EnvironmentBasePtr penv)
        : ROSSensorSystem<mocap_msgs::MocapSnapshot>("phasespace", penv)
    {
    }

    virtual bool Init(istream& sinput)
    {
        _listtopics.clear();
        _listtopics.push_back("robot_msgs_snapshot");
        return ROSSensorSystem<mocap_msgs::MocapSnapshot>::Init(sinput);
    }
    
private:
    virtual void newdatacb(const boost::shared_ptr<mocap_msgs::MocapSnapshot const>& topicmsg)
    {
        list< SNAPSHOT > listbodies;
        list< const mocap_msgs::MocapBody* > listnewbodies;

        {
            boost::mutex::scoped_lock lock(_mutex);

            for (unsigned int i=0; i<topicmsg->get_bodies_size(); i++) {
                const mocap_msgs::MocapBody& psbody = topicmsg->bodies[i];

                boost::shared_ptr<BodyData> b;
                Transform tnew = GetTransform(psbody.pose);

                FOREACH(it, _mapbodies) {
                    if( it->second->GetId() == psbody.id ) {
                        b = it->second;
                        break;
                    }
                }

                if( !b ) {
                    listnewbodies.push_back(&psbody);
                }
                else {
                    if( !b->IsEnabled() )
                        continue;
                    
                    listbodies.push_back(SNAPSHOT(b, tnew));
                }
            }
        }
        
        _UpdateBodies(listbodies);

        // try to add the left-over objects
    }

    Transform GetTransform(const geometry_msgs::Transform& pose)
    {
        return Transform(Vector(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z), Vector(pose.translation.x, pose.translation.y, pose.translation.z));
    }
};

#endif
