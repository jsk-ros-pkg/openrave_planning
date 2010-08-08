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
#ifndef OPENRAVE_ROS_H
#define OPENRAVE_ROS_H

#include <cstdio>
#include <cmath>
#include <cstdlib>

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_DEPRECATE

#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define RAVE_REGISTER_BOOST
#else

#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
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

#include <openrave-core.h>
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
#include <openraveros/env_createproblem.h>
#include <openraveros/env_createrobot.h>
#include <openraveros/env_destroyproblem.h>
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
#include <openraveros/problem_sendcommand.h>
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
using namespace openraveros;

#endif
