#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2010 Rosen Diankov (rosen.diankov@gmail.com)
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
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'
import roslib; roslib.load_manifest('orrosplanning')
import rospy

from optparse import OptionParser
from openravepy import *
from numpy import *
import numpy,time,threading
from itertools import izip
import tf

import sensor_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import object_manipulation_msgs.srv
import object_manipulation_msgs.msg
from IPython.Shell import IPShellEmbed

if __name__ == "__main__":
    parser = OptionParser(description='openrave planning example')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='robots/pr2-beta-static.robot.xml',
                      help='scene to load (default=%default)')
    parser.add_option('--collision_map',action="store",type='string',dest='collision_map',default='/collision_map/collision_map',
                      help='The collision map topic (maping_msgs/CollisionMap), by (default=%default)')
    parser.add_option('--ipython', '-i',action="store_true",dest='ipython',default=False,
                      help='if true will drop into the ipython interpreter rather than spin')
    (options, args) = parser.parse_args()
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=False)
    print 'initializing, please wait for ready signal...'

    try:
        rospy.init_node('graspplanning_openrave',disable_signals=False)
        with env:
            env.Load(options.scene)
            robot = env.GetRobots()[0]

            # set robot weights/resolutions (without this planning will be slow)
#             lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
#             if not lmodel.load():
#                 lmodel.autogenerate()
#             lmodel.setRobotWeights()
#             lmodel.setRobotResolutions()

            # create ground right under the robot
            ab=robot.ComputeAABB()
            ground=RaveCreateKinBody(env,'')
            ground.SetName('map')
            ground.InitFromBoxes(array([r_[ab.pos()-array([0,0,ab.extents()[2]+0.002]),2.0,2.0,0.001]]),True)
            env.AddKinBody(ground,False)
            baseframe = robot.GetLinks()[0].GetName()
            collisionmap = RaveCreateSensorSystem(env,'CollisionMap bodyoffset %s topic %s'%(robot.GetName(),options.collision_map))
            basemanip = interfaces.BaseManipulation(robot)
            grasper = interfaces.Grasper(robot)
        
        # have to do this manually because running linkstatistics when viewer is enabled segfaults things
        if options._viewer is None:
            env.SetViewer('qtcoin')
        elif len(options._viewer) > 0:
            env.SetViewer(options._viewer)

        listener = tf.TransformListener()
        values = robot.GetDOFValues()
        valueslock = threading.Lock()
        def UpdateRobotJoints(msg):
            with valueslock:
                with env:
                    for name,pos in izip(msg.name,msg.position):
                        j = robot.GetJoint(name)
                        if j is not None:
                            values[j.GetDOFIndex()] = pos
                    robot.SetDOFValues(values)

        def trimeshFromPointCloud(pointcloud):
            points = zeros((len(pointcloud.points),3),double)
            for i,p in enumerate(pointcloud.points):
                points[i,0] = p.x
                points[i,1] = p.y
                points[i,2] = p.z
            cindices = [c.values for c in pointcloud.channels if c.name == 'indices']
            if len(cindices) > 0:
                vertices = points
                indices = reshape(array(cindices[0],int),(len(cindices[0])/3,3))
            else:
                # compute the convex hull triangle mesh
                meanpoint = mean(points,1)
                planes,faces,triangles = grasper.ConvexHull(points,returntriangles=True)
                usedindices = zeros(len(points),int)
                usedindices[triangles.flatten()] = 1
                pointindices = flatnonzero(usedindices)
                pointindicesinv = zeros(len(usedindices))
                pointindicesinv[pointindices] = range(len(pointindices))
                vertices = points[pointindices]
                indices = reshape(pointindicesinv[triangles.flatten()],triangles.shape)
            return TriMesh(vertices=vertices,indices=indices)
        def CreateTarget(graspableobject):
            target = RaveCreateKinBody(env,'')
            Ttarget = eye(4)
            if graspableobject.type == object_manipulation_msgs.msg.GraspableObject.POINT_CLUSTER:
                target.InitFromTrimesh(trimeshFromPointCloud(graspableobject.cluster),True)
                (trans,rot) = listener.lookupTransform(baseframe, graspableobject.cluster.header.frame_id, rospy.Time(0))
                Ttarget = matrixFromQuat([rot[3],rot[0],rot[1],rot[2]])
                Ttarget[0:3,3] = trans
            else:
                raise ValueError('do not support graspable objects of type %s'%str(graspableobject.type))
            
            target.SetName('graspableobject')
            env.AddKinBody(target,True)
            target.SetTransform(Ttarget)
            return target

        def GraspPlanning(req):
            with valueslock:
                with env:
                    # update the robot
                    (robot_trans,robot_rot) = listener.lookupTransform(baseframe, robot.GetLinks()[0].GetName(), rospy.Time(0))
                    Trobot = matrixFromQuat([robot_rot[3],robot_rot[0],robot_rot[1],robot_rot[2]])
                    Trobot[0:3,3] = robot_trans
                    robot.SetTransform(Trobot)
                    # set the manipulator
                    if len(req.arm_name) > 0:
                        manip = robot.GetManipulator(req.arm_name)
                        if manip is None:
                            rospy.logerr('failed to find manipulator %s'%req.arm_name)
                            return None
                    else:
                        manips = [manip for manip in robot.GetManipulators() if manip.GetIkSolver() is not None and len(manip.GetArmIndices()) >= 6]
                        if len(manips) == 0:
                            rospy.logerr('failed to find manipulator end effector %s'%req.hand_frame_id)
                            return None
                        manip = manips[0]
                    robot.SetActiveManipulator(manip)

                    # create the target
                    target = env.GetKinBody(req.collision_object_name)
                    removetarget=False
                    if target is None:
                        target = CreateTarget(req.target)
                        removetarget = True
                    try:
                        res = object_manipulation_msgs.srv.GraspPlanningResponse()
                        # start planning
                        fastgrasping = examples.fastgrasping.FastGrasping(robot,target)
                        grasp,jointvalues = fastgrasping.computeGrasp(updateenv=False)
                        if grasp is not None:
                            res.error_code.value = object_manipulation_msgs.msg.GraspPlanningErrorCode.SUCCESS
                            rosgrasp = object_manipulation_msgs.msg.Grasp()
                            rosgrasp.pre_grasp_posture.header.stamp = rospy.Time.now()
                            rosgrasp.pre_grasp_posture.header.frame_id = baseframe
                            rosgrasp.pre_grasp_posture.name = [robot.GetJointFromDOFIndex(index).GetName() for index in fastgrasping.gmodel.manip.GetGripperIndices()]
                            rosgrasp.pre_grasp_posture.position = fastgrasping.gmodel.getPreshape(grasp)
                            # also include the arm positions
                            rosgrasp.grasp_posture.header = rosgrasp.pre_grasp_posture.header
                            rosgrasp.grasp_posture.name = rosgrasp.pre_grasp_posture.name + [robot.GetJointFromDOFIndex(index).GetName() for index in fastgrasping.gmodel.manip.GetArmIndices()]
                            rosgrasp.grasp_posture.position = jointvalues[r_[fastgrasping.gmodel.manip.GetGripperIndices(),fastgrasping.gmodel.manip.GetArmIndices()]]
                            T = fastgrasping.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
                            q = quatFromRotationMatrix(T[0:3,0:3])
                            rosgrasp.grasp_pose.position = geometry_msgs.msg.Point(T[0,3],T[1,3],T[2,3])
                            rosgrasp.grasp_pose.orientation = geometry_msgs.msg.Quaternion(q[1],q[2],q[3],q[0])
                            res.grasps.append(rosgrasp)
                        else:
                            res.error_code.value = object_manipulation_msgs.msg.GraspPlanningErrorCode.OTHER_ERROR
                        return res
                    finally:
                        env.Remove(target)

        sub = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, UpdateRobotJoints,queue_size=1)
        s = rospy.Service('GraspPlanning', object_manipulation_msgs.srv.GraspPlanning, GraspPlanning)
        print 'openrave %s service ready'%s.resolved_name

        if options.ipython:
            ipshell = IPShellEmbed(argv='',banner = 'Dropping into IPython',exit_msg = 'Leaving Interpreter, back to program.')
            ipshell(local_ns=locals())
        else:
            rospy.spin()
    finally:
        RaveDestroy()
