#!/usr/bin/env python
#
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
from __future__ import with_statement # for python 2.5

PKG = 'openrave_database' # ROS package name for dependencies
import roslib; roslib.load_manifest(PKG)

from numpy import *
import numpy
import sys, time, threading
import heapq # for nth smallest element
from optparse import OptionParser
from openravepy import *
from openravepy.databases import grasping

from openrave_database.srv import PickledService, PickledServiceRequest, PickledServiceResponse
import rospy, roslaunch
import roslaunch_caller

class EvaluationServerThread(threading.Thread):
    def __init__(self, service, finishcb):
        threading.Thread.__init__(self)
        self.service = service
        self.finishcb = finishcb
        self.ok = True
        self.starteval = threading.Condition(threading.Lock())
        self.req = None
        self.inds = None

    def run(self):
        with self.starteval:
            while self.ok:
                self.starteval.wait()
                if not self.ok:
                    break
                if self.req is None:
                    print('dummy command?')
                    continue
                res = self.service(self.req)
                if res is not None and self.ok:
                    self.finishcb(self.inds,res)
                self.inds = None
                self.req = None

class GraspingModelROS(grasping.GraspingModel):
    def __init__(self,robot,target,servicenames=None,server=False):
        grasping.GraspingModel.__init__(self,robot=robot,target=target)
        self.allthreads = []
        self.evallock = threading.Lock()
        self.server=server
        if not server and servicenames is not None:
            self.setservices(servicenames)

    def __del__(self):
        self.shutdownservices()

    def save(self):
        if not self.server:
            grasping.GraspingModel.save(self)

    def shutdownservices(self):
        if len(self.allthreads) > 0:
            print 'shutting down services'
            for t in self.allthreads:
                t.ok = False
#                 with t.starteval:
#                     t.starteval.notifyAll()
            print 'services have shutdown'

    def setservices(self,servicenames):
        self.shutdownservices()
        print 'waiting for services: ',servicenames
        for servicename in servicenames:
            rospy.wait_for_service(servicename)
        print 'starting service threads...'
        self.allthreads = [EvaluationServerThread(rospy.ServiceProxy(name, PickledService,persistent=True),
                                                  lambda inds,res: self.processResult(inds,pickle.loads(res.output))) for name in servicenames]
        for t in self.allthreads:
            t.start()

    def sendRequest(self,busythreads,input):
        service = None
        while service == None:
            for t in busythreads:
                if t.req is None:
                    service = t
                    break
            if service is None:
                time.sleep(0.01)
        with service.starteval:
            service.req = PickledServiceRequest(input = input)
            service.starteval.notifyAll()
    def processResult(self,inds,result):
        with self.evallock:
            grasp = result[0]
            if grasp is not None and grasp[self.graspindices.get('forceclosure')] > self.forceclosurethreshold:
                print 'found good grasp',len(self.grasps)
                self.grasps.append(grasp)
    def waitForThreads(self,busythreads):
        # wait for all threads to finish
        print 'waiting for all threads to finish'
        for t in busythreads:
            repeat = True
            while(repeat):
                with t.starteval:
                    if t.req is None:
                        repeat = False

    def generate(self,preshapes,standoffs,rolls,approachrays, graspingnoise=None,forceclosurethreshold=1e-9,updateenv=None,manipulatordirections=None,translationstepmult=None,finestep=None):
        if self.server:
            rospy.init_node('ComputeGrasping',anonymous=True)
            s = rospy.Service('ComputeGraspingService', PickledService, self.ComputeGraspingService)
            rospy.spin()
            return
        
        if approachrays is None:
            approachrays = self.computeBoxApproachRays(delta=0.02,normalanglerange=0)
        if preshapes is None:
            # should disable everything but the robot
            with self.target:
                self.target.Enable(False)
                # do not fill with plannername
                taskmanip = interfaces.TaskManipulation(self.robot)
                final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
            preshapes = array([final])
        if rolls is None:
            rolls = arange(0,2*pi,pi/2)
        if standoffs is None:
            standoffs = array([0,0.025])
        if graspingnoise is None:
            graspingnoise = 0.0
        if manipulatordirections is None:
            manipulatordirections = array([self.manip.GetDirection()])
        time.sleep(0.1) # sleep or otherwise viewer might not load well
        N = approachrays.shape[0]
        with self.env:
            Ttarget = self.target.GetTransform()

        # transform each ray into the global coordinate system in order to plot it
        gapproachrays = c_[dot(approachrays[:,0:3],transpose(Ttarget[0:3,0:3]))+tile(Ttarget[0:3,3],(N,1)),dot(approachrays[:,3:6],transpose(Ttarget[0:3,0:3]))]
        totalgrasps = N*len(preshapes)*len(rolls)*len(standoffs)*len(manipulatordirections)
        counter = 0
        self.grasps = []
        self.forceclosurethreshold = forceclosurethreshold

        busythreads = self.allthreads[:]        
        # reset the evaluated threads
        for t in self.allthreads:
            t.starteval.acquire()
            t.starteval.release()
        for approachray in approachrays:
            for roll in rolls:
                for preshape in preshapes:
                    for standoff in standoffs:
                        for manipulatordirection in manipulatordirections:
                            print 'grasp %d/%d'%(counter,totalgrasps),'preshape:',preshape
                            counter += 1
                            grasp = zeros(self.totaldof)
                            grasp[self.graspindices.get('igrasppos')] = approachray[0:3]
                            grasp[self.graspindices.get('igraspdir')] = -approachray[3:6]
                            grasp[self.graspindices.get('igrasproll')] = roll
                            grasp[self.graspindices.get('igraspstandoff')] = standoff
                            grasp[self.graspindices.get('igrasppreshape')] = preshape
                            grasp[self.graspindices.get('imanipulatordirection')] = manipulatordirection
                            self.sendRequest(busythreads,pickle.dumps((grasp,graspingnoise,forceclosurethreshold)))
        self.waitForThreads(busythreads)
        self.grasps = array(self.grasps)

    def ComputeGraspingService(self,req):
        grasp,graspingnoise,forceclosurethreshold = pickle.loads(req.input)
        try:
            contacts,finalconfig,mindist,volume = self.testGrasp(grasp=grasp,graspingnoise=graspingnoise,translate=True,forceclosure=True,forceclosurethreshold=forceclosurethreshold)
            Tlocalgrasp = eye(4)
            with self.env:
                self.robot.SetTransform(finalconfig[1])
                Tgrasp = self.manip.GetEndEffectorTransform()
                Tlocalgrasp = dot(linalg.inv(self.target.GetTransform()),Tgrasp)
                # find a non-colliding transform
                self.setPreshape(grasp)
                dir = self.getGlobalApproachDir(grasp)
                Tgrasp_nocol = array(Tgrasp)
                while self.manip.CheckEndEffectorCollision(Tgrasp_nocol):
                    Tgrasp_nocol[0:3,3] -= dir*0.001 # 1mm good enough?
                Tlocalgrasp_nocol = dot(linalg.inv(self.target.GetTransform()),Tgrasp_nocol)
                self.robot.SetJointValues(finalconfig[0])
            grasp[self.graspindices.get('igrasptrans')] = reshape(transpose(Tlocalgrasp[0:3,0:4]),12)
            grasp[self.graspindices.get('grasptrans_nocol')] = reshape(transpose(Tlocalgrasp_nocol[0:3,0:4]),12)
            grasp[self.graspindices.get('forceclosure')] = mindist
        except planning_error, e:
            print 'Grasp Failed: '
            grasp = None
        return PickledServiceResponse(output=pickle.dumps((grasp,)))

    @staticmethod
    def LaunchNodes(options,serviceaddrs=[('localhost','')],rosnamespace=None):
        nodes = """<machine name="localhost" address="localhost" default="true"/>\n"""
        args = ""
        if options.robot is not None:
            args += '--robot=%s '%options.robot
        if options.manipname is not None:
            args += '--manipname=%s '%options.manipname
        if options.target is not None:
            args += '--target=%s '%options.target
        controlargs = args
        for i,serviceaddr in enumerate(serviceaddrs):
            nodes += """<machine name="m%d" address="%s" default="false" %s/>\n"""%(i,serviceaddr[0],serviceaddr[1])
            nodes += """<node machine="m%d" name="g%d" pkg="%s" type="grasping_ros.py" args="--startserver %s" output="log" cwd="node">\n  <remap from="ComputeGraspingService" to="g%d"/>\n</node>"""%(i,i,PKG,args,i)
            controlargs += '--service=g%d '%i
        if options.boxdelta is not None:
            controlargs += '--boxdelta=%f '%options.boxdelta
        if options.spheredelta is not None:
            controlargs += '--spheredelta=%f '%options.spheredelta
        if options.normalanglerange is not None:
            controlargs += '--normalanglerange=%f '%options.normalanglerange
        if options.directiondelta is not None:
            controlargs += '--directiondelta=%f '%options.directiondelta
        if options.friction is not None:
            controlargs += '--friction=%f '%options.friction
        if options.graspingnoise is not None:
            controlargs += '--graspingnoise=%f '%options.graspingnoise
        if options.standoffs is not None:
            for standoff in options.standoffs:
                controlargs += '--standoff=%s '%standoff
        if options.rolls is not None:
            for roll in options.rolls:
                controlargs += '--roll=%s '%roll
        if options.preshapes is not None:
            for preshape in options.preshapes:
                controlargs += '--preshape=\'%s\' '%preshape
        if options.manipulatordirections is not None:
            for manipulatordirection in options.manipulatordirections:
                controlargs += '--manipulatordirection=\'%s\' '%manipulatordirection
        if options.avoidlinks is not None:
            for avoidlink in options.avoidlinks:
                controlargs += '--avoidlink=%s '%avoidlink
        nodes += """<node machine="localhost" name="grasping" pkg="%s" type="grasping_ros.py" args="%s" output="screen" cwd="node"/>\n"""%(PKG,controlargs)
        xml_text = """<launch>\n"""
        if rosnamespace is not None and len(rosnamespace) > 0:
            xml_text += """<group ns="%s">\n%s</group>"""%(rosnamespace,nodes)
        else:
            xml_text += nodes
        xml_text += "\n</launch>\n"
        roslaunch.pmon._shutting_down = False # roslaunch registers its own signal handlers and shuts down automatically on sigints
        launchscript = roslaunch_caller.ScriptRoslaunch(xml_text)
        launchscript.start()
        try:
            controlname = [name for name in launchscript.pm.get_active_names() if name.startswith('grasping')]
            while True:
                controlproc = launchscript.pm.get_process(controlname[0])
                if controlproc is None or not controlproc.is_alive():
                    break
                time.sleep(1)
            print 'grasping finished'
        finally:
            launchscript.shutdown()
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = grasping.GraspingModel.CreateOptionParser()
        (options, args) = parser.parse_args()
        env = Environment()
        try:
            target = None
            with env:
                target = env.ReadKinBodyXMLFile(options.target)
                target.SetTransform(eye(4))
                env.AddKinBody(target)
            if Model is None:
                NewModel = lambda robot: GraspingModel(robot=robot,target=target)
            else:
                NewModel = lambda robot: Model(robot,target=target)
            OpenRAVEModel.RunFromParser(env=env,Model=NewModel,parser=parser)
        finally:
            env.Destroy()

if __name__=='__main__':
    parser = grasping.GraspingModel.CreateOptionParser()
    parser.add_option('--service', action='append', type='string', dest='servicenames',default=[],
                      help='The services used to evaluate grasping')
    parser.add_option('--startserver', action='store_true', dest='server',default=False,
                      help='If set, will start a service on the ROS network offering to evaluate grasping')
    parser.add_option('--launchservice', action='append', dest='launchservices',default=[],
                      help="""If specified, will roslaunch the services and setup the correct bindings for parallel processing (recommended). Usage: "python grasping_ros.py --launchservice='4*localhost' --robot=robots/barrettsegway.robot.xml --manipname=arm" """)
    (options, args) = parser.parse_args()
    options.useviewer = False
    if len(options.launchservices) > 0:
        serviceaddrs = []
        for launchservice in options.launchservices:
            launchservice = launchservice.strip()
            pos = launchservice.find(' ')
            if pos >= 0:
                addr = launchservice[0:pos]
                args = launchservice[pos+1:]
            else:
                addr = launchservice
                args = ''
            posnum = addr.find('*')
            if posnum >= 0:
                numprocesses=int(addr[0:posnum])
                addr = addr[posnum+1:].strip()
                for i in range(numprocesses):
                    serviceaddrs.append([addr,args])
            else:
                serviceaddrs.append([addr,args])
        GraspingModelROS.LaunchNodes(options,serviceaddrs=serviceaddrs,rosnamespace='grasping')
    else:
        servicenames = options.servicenames[:]
        GraspingModelROS.RunFromParser(parser=parser,Model=lambda robot,target: GraspingModelROS(robot=robot,target=target,servicenames=servicenames,server=options.server))
    sys.exit(0)

def test():
    import grasping_ros
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = grasping_ros.GraspingModelROS(robot=robot,servicenames=['ComputeGraspingService'])
    maxradius=None
    translationonly=False
    xyzdelta=0.04
    quatdelta=0.5
    self.generate(maxradius=maxradius,translationonly=translationonly,xyzdelta=xyzdelta,quatdelta=quatdelta)

def test_launch():
    import grasping_ros
    serviceaddrs=[('localhost','')]
    rosnamespace='grasping'
    robotfilename='robots/barrettsegway.robot.xml'
    options = dict()
    grasping_ros.GraspingModelROS.LaunchNodes(options,serviceaddrs=[('localhost','')],rosnamespace='grasping')
