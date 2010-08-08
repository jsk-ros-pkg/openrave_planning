#!/usr/bin/env python
# Copyright (C) 2009 Rosen Diankov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PKG = 'robot_openrave_control'
import roslib; roslib.load_manifest(PKG)

import sys, os
import rospy
from robot_openrave_control.srv import *
from robot_openrave_control.msg import *

from string import atoi,atof
from optparse import OptionParser

from IPython.Shell import IPShellEmbed

class ControllerClient():
    """Controls robots connected using the robot_openrave_control set of services using python"""
    def __init__(self,session='controller_session'):
        sessionpath=os.path.split(session)[0]
        rospy.init_node('robot_openrave_client', anonymous=True)
        self.controller_session_srv = rospy.ServiceProxy(session, controller_session)
        req = controller_sessionRequest(0,controller_sessionRequest.Access_ForceControl)
        res = self.controller_session_srv(req)
        self.sessionid = res.sessionid
        self.connection_header = {rospy.resolve_name(session):str(self.sessionid)}

        self.query_srv = rospy.ServiceProxy(sessionpath+'/Query', Query,persistent=True,headers=self.connection_header)
        self.wait_srv = rospy.ServiceProxy(sessionpath+'/Wait', Wait,persistent=True,headers=self.connection_header)
        self.cancel_srv = rospy.ServiceProxy(sessionpath+'/Cancel', Cancel,persistent=True,headers=self.connection_header)
        self.brake_srv = rospy.ServiceProxy(sessionpath+'/Brake', Brake,persistent=True,headers=self.connection_header)
        self.StartTrajectory_srv = rospy.ServiceProxy(sessionpath+'/StartTrajectory', StartTrajectory,persistent=True,headers=self.connection_header)
        self.StartVelocity_srv = rospy.ServiceProxy(sessionpath+'/StartVelocity', StartVelocity,persistent=True,headers=self.connection_header)
        self.StartTorque_srv = rospy.ServiceProxy(sessionpath+'/StartTorque', StartTorque,persistent=True,headers=self.connection_header)
        self.StartCustomString_srv = rospy.ServiceProxy(sessionpath+'/StartCustomString', StartCustomString,persistent=True,headers=self.connection_header)

    def settorque(self,torques):
        req = StartTorqueRequest()
        req.torques = torques
        res = self.StartTorque_srv(req)

    def setposition(self,positions):
        req = StartTrajectoryRequest()
        req.traj.points.append(JointTrajPoint(positions=positions,time=0))
        req.interpolation = StartTrajectoryRequest.Interp_Linear
        return self.StartTrajectory_srv(req)

    def setvelocity(self,velocities):
        req = StartVelocityRequest()
        req.velocities = velocities
        return self.StartVelocity_srv(req)

    def settrajectoryfromfile(self,trajfile):
        return self.settrajectoryfromdata(open(trajfile, "r").read())

    def settrajectoryfromdata(self,trajdata):
        req = StartTrajectoryRequest()
        req.hastiming = 1
        req.interpolation = StartTrajectoryRequest.Interp_Linear

        tokens = trajdata.split()
        numpoints = atoi(tokens.pop(0))
        numdof = atoi(tokens.pop(0))
        options = atoi(tokens.pop(0))

        if options != 4 or options != 36:
            raise ValueError('trajectory file options need to be 20 (include timestamps)')

        for i in range(numpoints):
            time = atof(tokens.pop(0))
            positions = [atof(tokens.pop(0)) for i in range(numdof)]
            if options == 36:
                torques = [atof(tokens.pop(0)) for i in range(numdof)]
            else:
                torques = []
            req.traj.points.append(JointTrajPoint(positions=positions,time=time,torques=torques))

        return self.StartTrajectory_srv(req)
    def setcustom(self,custom):
        return self.StartCustomString_srv(StartCustomStringRequest(input=custom))
        
    def cancel(self,commandid=0):
        req = CancelRequest()
        req.commandid = commandid
        return self.cancel_srv(req)

if __name__ == '__main__':
    try:
        parser = OptionParser(usage='usage: %prog [options] [solve joint indices]',
                              description="""Python Client for openrave controller""")
        parser.add_option('--trajfile', action='store', type='string', dest='trajfile',default=None,
                          help='trajectory file to execute')
        parser.add_option('--sessionname', action='store', type='string', dest='sessionname',default='controller_session',
                          help='ros scope of controller session')
        parser.add_option('--ipython', action='store_true', dest='ipython',default=False,
                          help='go into ipython (can use class through self)')
        (options, args) = parser.parse_args()
        self = ControllerClient(session=options.sessionname)
        if options.trajfile is not None:
            self.settrajectoryfromfile(options.trajfile)
            sys.exit(0)
        while True:
            cmd = raw_input('Enter command (q-quit,c-custom,i-ipython): ')
            if cmd == 'q':
                break
            elif cmd == 'i':
                ipshell = IPShellEmbed(argv='',banner = 'Dropping into IPython',exit_msg = 'Leaving Interpreter, back to program.')
                ipshell(local_ns=locals())
            elif cmd == 'c':
                cmd=raw_input('enter command: ')
                if len(cmd) > 0:
                    print self.setcustom(cmd)
            #rospy.spinOnce()
    except KeyboardInterrupt, e:
        pass
