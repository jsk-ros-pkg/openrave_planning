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
#include <openrave/openrave.h>
#include <openrave/plugin.h>

using namespace OpenRAVE;

ModuleBasePtr CreateROSServer(EnvironmentBasePtr penv, std::istream& sinput);

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_Module:
        if( interfacename == "rosserver" ) {
            return InterfaceBasePtr(CreateROSServer(penv,sinput));
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_Module].push_back("ROSServer");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
