// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef ROS_PLUGINS_H
#define ROS_PLUGINS_H

#include <rave/rave.h>
using namespace OpenRAVE;

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST
#else

#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#include <stdint.h>
#include <fstream>
#include <iostream>

#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/format.hpp>

using namespace std;

#include <sys/timeb.h>    // ftime(), struct timeb

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline uint32_t timeGetTime()
{
#ifdef _WIN32
    _timeb t;
    _ftime(&t);
#else
    timeb t;
    ftime(&t);
#endif

    return (uint32_t)(t.time*1000+t.millitm);
}

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

inline uint64_t GetMicroTime()
{
#ifdef _WIN32
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    return (uint64_t)t.tv_sec*1000000+t.tv_usec;
#endif
}

#include <ros/node_handle.h>
#include <ros/master.h>
#include <ros/time.h>
using namespace ros;

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
inline Transform GetTransform(const geometry_msgs::Pose& pose)
{
    return Transform(Vector(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z), Vector(pose.position.x, pose.position.y, pose.position.z));
}
inline Transform GetTransform(const btTransform& bt)
{
    btQuaternion q = bt.getRotation();
    btVector3 o = bt.getOrigin();
    return Transform(Vector(q.w(),q.x(),q.y(),q.z()),Vector(o.x(),o.y(),o.z()));
}
inline geometry_msgs::Pose GetPose(const Transform& t) {
    geometry_msgs::Pose p;
    p.orientation.x = t.rot.y; p.orientation.y = t.rot.z; p.orientation.z = t.rot.w; p.orientation.w = t.rot.x;
    p.position.x = t.trans.x; p.position.y = t.trans.y; p.position.z = t.trans.z;
    return p;
}

#include <ros/package.h>

inline string resolveName(const std::string& url)
{
    if (url.find("package://") != 0)
        return url;

    std::string mod_url = url;
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
        throw openrave_exception(str(boost::format("Could not parse %s into file:// format")%url));
    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package);

    if (package_path.empty())
        throw openrave_exception("Package [" + package + "] does not exist");

    return package_path + mod_url;
}
    
#endif
