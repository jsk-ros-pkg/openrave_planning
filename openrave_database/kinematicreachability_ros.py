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
from openravepy.databases import kinematicreachability, convexdecomposition

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

class ReachabilityModelROS(kinematicreachability.ReachabilityModel):
    def __init__(self,robot,servicenames=None,server=False):
        kinematicreachability.ReachabilityModel.__init__(self,robot=robot)
        self.allthreads = []
        self.evallock = threading.Lock()
        self.server=server
        if not server and servicenames is not None:
            self.setservices(servicenames)

    def __del__(self):
        self.shutdownservices()

    def save(self):
        if not self.server:
            kinematicreachability.ReachabilityModel.save(self)

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

    def processResult(self,inds,result):
        with self.evallock:
            for i,ind in enumerate(inds):
                self.reachability3d[ind] = result[0][i][0]
                self.reachabilitydensity3d[ind] = result[0][i][1]
            self.reachabilitystats += result[1]

    def generate(self,maxradius=None,translationonly=False,xyzdelta=0.04,quatdelta=0.5,usefreespace=True,useconvex=False):
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.usefreespace = usefreespace
        with self.env:
            self.robot.SetTransform(eye(4)) # have to transform robot to identify
            if useconvex:
                self.cdmodel = convexdecomposition.ConvexDecompositionModel(self.robot)
                if not self.cdmodel.load():
                    self.cdmodel.autogenerate()
        if self.server:
            rospy.init_node('ComputeReachability',anonymous=True)
            s = rospy.Service('ComputeReachabilityService', PickledService, self.ComputeReachabilityService)
            rospy.spin()
            return
        if xyzdelta is None:
            xyzdelta=0.04
        if quatdelta is None:
            quatdelta=0.5        
        with self.robot:
            Tbase = self.manip.GetBase().GetTransform()
            Tbaseinv = linalg.inv(Tbase)

            starttime = time.time()
            armjoints = self.getOrderedArmJoints()
            baseanchor = transformPoints(Tbaseinv,[armjoints[0].GetAnchor()])
            eetrans = self.manip.GetEndEffectorTransform()[0:3,3]
            armlength = 0
            for j in armjoints[::-1]:
                armlength += sqrt(sum((eetrans-j.GetAnchor())**2))
                eetrans = j.GetAnchor()    
            if maxradius is None:
                maxradius = armlength+xyzdelta

            allpoints,insideinds,shape,self.pointscale = self.UniformlySampleSpace(maxradius,delta=xyzdelta)
            # select the best sphere level matching quatdelta;
            # level=0, quatdist = 0.5160220
            # level=1: quatdist = 0.2523583
            # level=2: quatdist = 0.120735
            qarray = SpaceSampler().sampleSO3(quatdelta=quatdelta)
            rotations = [eye(3)] if translationonly else rotationMatrixFromQArray(qarray)
            self.xyzdelta = xyzdelta
            self.quatdelta = 0
            if not translationonly:
                # for rotations, get the average distance to the nearest rotation
                neighdists = []
                for q in qarray:
                    neighdists.append(heapq.nsmallest(2,quatArrayTDist(q,qarray))[1])
                self.quatdelta = mean(neighdists)
            print 'radius: %f, xyzsamples: %d, quatdelta: %f, rot samples: %d'%(maxradius,len(insideinds),self.quatdelta,len(rotations))

        busythreads = self.allthreads[:]
        
        # reset the evaluated threads
        for t in self.allthreads:
            t.starteval.acquire()
            t.starteval.release()

        self.reachabilitydensity3d = zeros(prod(shape))
        self.reachability3d = zeros(prod(shape))
        self.reachabilitystats = []
        num = len(insideinds)/100
        for i in range(num):
            print '%d/%d'%(i,num)
            translations = allpoints[insideinds[i::num]]
            translations += tile(baseanchor,(len(translations),1))
            service = None
            while service == None:
                for t in busythreads:
                    if t.req is None:
                        service = t
                        break
                if service is None:
                    time.sleep(0.01)
            with service.starteval:
                service.req = PickledServiceRequest(input = pickle.dumps((translations,rotations)))
                service.inds = insideinds[i::num]
                service.starteval.notifyAll()

        # wait for all threads to finish
        print 'waiting for all threads to finish'
        for t in busythreads:
            repeat = True
            while(repeat):
                with t.starteval:
                    if t.req is None:
                        repeat = False

        self.reachability3d = reshape(self.reachability3d,shape)
        self.reachabilitydensity3d = reshape(self.reachabilitydensity3d,shape)
        self.reachabilitystats = array(self.reachabilitystats)
        print 'finished, total time: %f'%(time.time()-starttime)

    def ComputeReachabilityService(self,req):
        translations,rotations = pickle.loads(req.input)
        reachabilitystats = []
        density = []
        T = eye(4)
        with self.robot:
            Tbase = self.manip.GetBase().GetTransform()
            for translation in translations:
                numvalid = 0
                numrotvalid = 0
                T[0:3,3] = translation
                for rotation in rotations:
                    T[0:3,0:3] = rotation
                    if self.usefreespace:
                        solutions = self.manip.FindIKSolutions(dot(Tbase,T),False) # do not want to include the environment
                        if solutions is not None:
                            reachabilitystats.append(r_[poseFromMatrix(T),len(solutions)])
                            numvalid += len(solutions)
                            numrotvalid += 1
                    else:
                        solution = self.manip.FindIKSolution(dot(Tbase,T),False)
                        if solution is not None:
                            reachabilitystats.append(r_[poseFromMatrix(T),1])
                            numvalid += 1
                            numrotvalid += 1
                density.append((numrotvalid/float(len(rotations)),numvalid/float(len(rotations))))
        return PickledServiceResponse(output=pickle.dumps((density,reachabilitystats)))

    @staticmethod
    def LaunchNodes(serviceaddrs=[('localhost','')],rosnamespace=None,robotfilename=None,xyzdelta=None,quatdelta=None,manipname=None,useconvex=None,usefreespace=None):
        nodes = """<machine name="localhost" address="localhost" default="true"/>\n"""
        args = ""
        if robotfilename is not None:
            args += '--robot=%s '%robotfilename
        if manipname is not None:
            args += '--manipname=%s '%manipname
        if not usefreespace is None and not usefreespace:
            args += '--ignorefreespace '
        if not useconvex is None and useconvex:
            args += '--useconvex '
        controlargs = args
        for i,serviceaddr in enumerate(serviceaddrs):
            nodes += """<machine name="m%d" address="%s" default="false" %s/>\n"""%(i,serviceaddr[0],serviceaddr[1])
            nodes += """<node machine="m%d" name="kr%d" pkg="%s" type="kinematicreachability_ros.py" args="--startserver %s" output="log" cwd="node">\n  <remap from="ComputeReachabilityService" to="kr%d"/>\n</node>"""%(i,i,PKG,args,i)
            controlargs += '--service=kr%d '%i
        if xyzdelta is not None:
            controlargs += '--xyzdelta=%f '%xyzdelta
        if quatdelta is not None:
            controlargs += '--quatdelta=%f '%quatdelta
        nodes += """<node machine="localhost" name="kinematicreachability" pkg="%s" type="kinematicreachability_ros.py" args="%s" output="screen" cwd="node"/>\n"""%(PKG,controlargs)
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
            controlname = [name for name in launchscript.pm.get_active_names() if name.startswith('kinematicreachability')]
            while True:
                controlproc = launchscript.pm.get_process(controlname[0])
                if controlproc is None or not controlproc.is_alive():
                    break
                time.sleep(1)
            print 'kinematic reachability finished'
        finally:
            launchscript.shutdown()

if __name__=='__main__':
    parser = kinematicreachability.ReachabilityModel.CreateOptionParser()
    parser.add_option('--service', action='append', type='string', dest='servicenames',default=[],
                      help='The services used to evaluate kinematics')
    parser.add_option('--startserver', action='store_true', dest='server',default=False,
                      help='If set, will start a service on the ROS network offering to evaluate kinematics equations')
    parser.add_option('--launchservice', action='append', dest='launchservices',default=[],
                      help="""If specified, will roslaunch the services and setup the correct bindings for parallel processing (recommended). Usage: "python kinematicreachability_ros.py --launchservice='4*localhost' --robot=robots/barrettsegway.robot.xml --manipname=arm" """)
    (options, args) = parser.parse_args()
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
        ReachabilityModelROS.LaunchNodes(serviceaddrs=serviceaddrs,rosnamespace='kinematicreachability',robotfilename=options.robot,xyzdelta=options.xyzdelta,quatdelta=options.quatdelta,manipname=options.manipname,useconvex=options.useconvex,usefreespace=options.usefreespace)
    else:
        servicenames = options.servicenames[:]
        kinematicreachability.ReachabilityModel.RunFromParser(parser=parser,Model=lambda robot: ReachabilityModelROS(robot=robot,servicenames=servicenames,server=options.server))
    sys.exit(0)

def test():
    import kinematicreachability_ros
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = kinematicreachability_ros.ReachabilityModelROS(robot=robot,servicenames=['ComputeReachabilityService'])
    maxradius=None
    translationonly=False
    xyzdelta=0.04
    quatdelta=0.5
    self.generate(maxradius=maxradius,translationonly=translationonly,xyzdelta=xyzdelta,quatdelta=quatdelta)

def test_launch():
    import kinematicreachability_ros
    serviceaddrs=[('localhost','')]
    rosnamespace='kinematicreachability'
    robotfilename='robots/barrettsegway.robot.xml'
    xyzdelta=0.04
    quatdelta=0.5
    kinematicreachability_ros.ReachabilityModelROS.LaunchNodes(serviceaddrs=[('localhost','')],rosnamespace='kinematicreachability',robotfilename='robots/barrettsegway.robot.xml',xyzdelta=0.04,quatdelta=0.5)
