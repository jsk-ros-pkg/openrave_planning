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
#ifndef ROS_SENSORSYSTEM_SYSTEM
#define ROS_SENSORSYSTEM_SYSTEM

#include "plugindefs.h"

// used to update objects through a mocap system
template <typename T>
class ROSSensorSystem : public SimpleSensorSystem
{
public:
 ROSSensorSystem(const std::string& xmlid, EnvironmentBasePtr penv) : SimpleSensorSystem(xmlid,penv), _bDestroyThread(true)
    {
    }
    virtual ~ROSSensorSystem() {
        Destroy();
    }

    virtual bool Init(istream& sinput)
    {
        string cmd;
        _listtopics.clear();
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
            else break;

            if( !sinput ) {
                RAVELOG_ERRORA("failed\n");
                return false;
            }
        }

        startsubscriptions();
        return !!_ros;
    }

    virtual bool startsubscriptions(int queuesize=10)
    {
        Destroy();

        int argc=0;
        ros::init(argc,NULL,"openrave", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        if( ros::master::check() ) {
            _ros.reset(new ros::NodeHandle());
            FOREACH(ittopic,_listtopics)
                _listsubtopics.push_back(_ros->subscribe(*ittopic,queuesize,&ROSSensorSystem::newdatacb,this));

            _bDestroyThread = false;
            _threadros = boost::thread(boost::bind(&ROSSensorSystem::_threadrosfn, this));
            return true;
        }

        return false;
    }

    virtual void Destroy()
    {
        _bDestroyThread = true;
        FOREACH(itsub,_listsubtopics)
            itsub->shutdown();
        _listsubtopics.clear();
        _ros.reset();
        _threadros.join();
    }

protected:
    inline boost::shared_ptr<SensorSystemBase> shared_system() { return boost::static_pointer_cast<SensorSystemBase>(shared_from_this()); }
    inline boost::shared_ptr<SensorSystemBase const> shared_system_const() const { return boost::static_pointer_cast<SensorSystemBase const>(shared_from_this()); }

    virtual void _threadrosfn()
    {
        while(!_bDestroyThread) {
            ros::spinOnce();
            usleep(1000); // query every 1ms
        }
    }

    virtual void newdatacb(const boost::shared_ptr<T const>& topicmsg)
    {
    }

    boost::shared_ptr<ros::NodeHandle> _ros;
    boost::thread _threadros;
    list<ros::Subscriber> _listsubtopics;
    list<string> _listtopics;
    bool _bDestroyThread;
};

#endif
