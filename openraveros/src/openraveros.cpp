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
#include "openraveros.h"
#include "session.h"
#include <signal.h>

void sigint_handler(int);

boost::shared_ptr<SessionServer> s_sessionserver;

void printhelp()
{
    printf("openraveros [--list] [--debuglevel [level]]\n"
            "  Starts the OpenRAVE ROS server\n"
            "--list             List all the loadable interfaces (ie, collision checkers).\n"
            "-d, --debuglevel [level]    Set a debug level, higher numbers are more verbose (default is 3)\n");
}

void printinterfaces(EnvironmentBasePtr penv)
{
    std::map<InterfaceType, std::vector<std::string> > interfacenames;
    penv->GetLoadedInterfaces(interfacenames);

    stringstream ss;
            
    ss << endl << "Loadable interfaces: " << endl;
    FOREACH(itinterface,interfacenames) {
        ss << RaveGetInterfaceName(itinterface->first) << "(" << itinterface->second.size() << "):" << endl;
        FOREACH(it, itinterface->second)
            ss << " " << *it << endl;
        ss << endl;
    }
    ROS_INFO("%s",ss.str().c_str());
}

int main(int argc, char ** argv)
{
    signal(SIGINT,sigint_handler); // control C

    ros::init(argc,argv,"openraveserver", ros::init_options::NoSigintHandler);    
    if( !ros::master::check() )
        return 1;
    
    s_sessionserver.reset(new SessionServer());
    if( !s_sessionserver->Init() )
        return 2;

    // parse the command line options
    bool bExit = false;
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-?") == 0 || strcmp(argv[i], "/?") == 0 || strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
            printhelp();
            bExit = true;
            break;
        }
        else if( strcmp(argv[i], "--debuglevel") == 0 || strcmp(argv[i], "-d") == 0 ) {
            RaveSetDebugLevel((DebugLevel)atoi(argv[i+1]));
            i += 2;
        }
        else if( strcmp(argv[i], "--list") == 0 ) {
            printinterfaces(s_sessionserver->GetParentEnvironment());
            bExit = true;
            break;
        }
        else
            break;
    }

    if( !bExit )
        ros::spin();
    s_sessionserver.reset();    
    return 0;
}

void sigint_handler(int)
{
    s_sessionserver->shutdown();
    ros::shutdown();
}
