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
#include "actionlib_controller.h"
#include <rave/plugin.h>

static std::list< boost::shared_ptr<void> >* s_listRegisteredReaders = NULL; ///< have to make it a pointer in order to prevent static object destruction from taking precedence
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( !s_listRegisteredReaders ) {
        s_listRegisteredReaders = new list< boost::shared_ptr<void> >();
        //s_listRegisteredReaders.push_back(ROSMocapSystem::RegisterXMLReader(penv));
    }
    switch(type) {
    case PT_Controller:
        if( interfacename == "rosactionlib")
            return InterfaceBasePtr(new ROSActionLibController(penv,sinput));
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_Controller].push_back("rosactionlib");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    delete s_listRegisteredReaders;
    s_listRegisteredReaders = NULL;
}
