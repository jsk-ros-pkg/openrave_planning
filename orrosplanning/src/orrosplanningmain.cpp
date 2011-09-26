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

#include "objecttransformsystem.h"
#include "rosbindings.h"
#include "collisionmapsystem.h"

#include <rave/plugin.h>

ControllerBasePtr CreateROSPassiveController(EnvironmentBasePtr penv, std::istream& sinput);

static list< boost::shared_ptr<void> >* s_listRegisteredReaders = NULL;
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( !s_listRegisteredReaders ) {
        s_listRegisteredReaders = new list< boost::shared_ptr<void> >();
        s_listRegisteredReaders->push_back(ObjectTransformSystem::RegisterXMLReader(penv));
        s_listRegisteredReaders->push_back(CollisionMapSystem::RegisterXMLReader(penv));
    }
    switch(type) {
    case PT_SensorSystem:
        if( interfacename == "objecttransform" ) {
            boost::shared_ptr<ObjectTransformSystem> psys(new ObjectTransformSystem(penv));
            if( !psys->Init(sinput) ) {
                RAVELOG_WARN(str(boost::format("failed to init %s\n")%interfacename));
            }
            return psys;
        }
        else if( interfacename == "collisionmap" ) {
            boost::shared_ptr<CollisionMapSystem> psys(new CollisionMapSystem(penv));
            if( !psys->Init(sinput) ) {
                RAVELOG_WARN(str(boost::format("failed to init %s\n")%interfacename));
            }
            return psys;
        }
        break;
    case PT_ProblemInstance:
        if( interfacename == "rosbindings" ) {
            return InterfaceBasePtr(new ROSBindings(penv));
        }
        break;
    case PT_Controller:
        if( interfacename == "rospassivecontroller" ) {
            return CreateROSPassiveController(penv,sinput);
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_SensorSystem].push_back("ObjectTransform");
    info.interfacenames[OpenRAVE::PT_SensorSystem].push_back("CollisionMap");
    info.interfacenames[OpenRAVE::PT_ProblemInstance].push_back("ROSBindings");
    info.interfacenames[OpenRAVE::PT_Controller].push_back("ROSPassiveController");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    delete s_listRegisteredReaders;
    s_listRegisteredReaders = NULL;
}
