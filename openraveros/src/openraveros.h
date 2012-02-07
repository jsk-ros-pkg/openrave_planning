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
#ifndef OPENRAVE_ROS_H
#define OPENRAVE_ROS_H

#include <openrave/openrave.h>
#include <openrave/utils.h>

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_DEPRECATE

#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define RAVE_REGISTER_BOOST
#else

#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

#endif

#include <fstream>
#include <iostream>
#include <sstream>

#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x)/(sizeof( (x)[0] )))
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#ifndef _WIN32
#define strnicmp strncasecmp
#define stricmp strcasecmp
#endif

#include <ros/node_handle.h>
#include <ros/master.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/static_assert.hpp>
#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>

// services
#include <openraveros/body_destroy.h>
#include <openraveros/body_enable.h>
#include <openraveros/body_getaabb.h>
#include <openraveros/body_getaabbs.h>
#include <openraveros/body_getdof.h>
#include <openraveros/body_getjointvalues.h>
#include <openraveros/body_setjointvalues.h>
#include <openraveros/body_settransform.h>
#include <openraveros/env_checkcollision.h>
#include <openraveros/env_closefigures.h>
#include <openraveros/env_createbody.h>
#include <openraveros/env_createplanner.h>
#include <openraveros/env_createmodule.h>
#include <openraveros/env_createrobot.h>
#include <openraveros/env_destroymodule.h>
#include <openraveros/env_getbodies.h>
#include <openraveros/env_getbody.h>
#include <openraveros/env_getrobots.h>
#include <openraveros/env_loadplugin.h>
#include <openraveros/env_loadscene.h>
#include <openraveros/env_plot.h>
#include <openraveros/env_raycollision.h>
#include <openraveros/env_set.h>
#include <openraveros/env_triangulate.h>
#include <openraveros/env_wait.h>
#include <openraveros/planner_init.h>
#include <openraveros/planner_plan.h>
#include <openraveros/module_sendcommand.h>
#include <openraveros/robot_controllersend.h>
#include <openraveros/robot_controllerset.h>
#include <openraveros/robot_getactivevalues.h>
#include <openraveros/robot_sensorgetdata.h>
#include <openraveros/robot_sensorsend.h>
#include <openraveros/robot_setactivedofs.h>
#include <openraveros/robot_setactivevalues.h>
#include <openraveros/robot_starttrajectory.h>

using namespace OpenRAVE;
using namespace std;

#endif
