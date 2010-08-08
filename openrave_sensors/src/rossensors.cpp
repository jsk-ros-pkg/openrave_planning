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
#include "plugindefs.h"

#include "roslaser2d.h"
#include "rossensorpublisher.h"

#include <rave/plugin.h>

static list< boost::shared_ptr<void> > s_listRegisteredReaders;
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( s_listRegisteredReaders.size() == 0 ) {
        s_listRegisteredReaders.push_back(penv->RegisterXMLReader(PT_Sensor,"roslaser2d",ROSLaser2D::CreateXMLReader));
    }
    switch(type) {
        case PT_Sensor:
            if( interfacename == "roslaser2d")
                return InterfaceBasePtr(new ROSLaser2D(penv));
            break;
        case PT_ProblemInstance:
            if( interfacename == "rossensorpublisher" )
                return InterfaceBasePtr(new ROSSensorPublisher(penv));
            break;
        default:
            break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Sensor].push_back("ROSLaser2D");
    info.interfacenames[PT_ProblemInstance].push_back("ROSSensorPublisher");
}

void DestroyPlugin()
{
    s_listRegisteredReaders.clear();
}
