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
import os

from optparse import OptionParser
from openravepy import *
from openravepy.misc import OpenRAVEGlobalArguments
from numpy import *
import numpy,time,threading
from itertools import izip
import tf

import orrosplanning.srv
from orrosplanning.srv import IKRequest
import sensor_msgs.msg
import arm_navigation_msgs.msg
from arm_navigation_msgs.msg import ArmNavigationErrorCodes
import geometry_msgs.msg
import kinematics_msgs.srv

if __name__ == "__main__":
    parser = OptionParser(description='openrave planning example')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='robots/pr2-beta-static.zae',
                      help='scene to load (default=%default)')
    parser.add_option('--ipython', '-i',action="store_true",dest='ipython',default=False,
                      help='if true will drop into the ipython interpreter rather than spin')
    parser.add_option('--collision_map',action="store",type='string',dest='collision_map',default='/collision_map/collision_map',
                      help='The collision map topic (maping_msgs/CollisionMap), by (default=%default)')
    parser.add_option('--mapframe',action="store",type='string',dest='mapframe',default=None,
                      help='The frame of the map used to position the robot. If --mapframe="" is specified, then nothing will be transformed with tf')
    (options, args) = parser.parse_args()
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    RaveLoadPlugin(os.path.join(roslib.packages.get_pkg_dir('orrosplanning'),'lib','liborrosplanning.so'))
    env.LoadProblem(RaveCreateModule(env,"textserver"),"")
    # load the orrosplanning plugin
    
    print 'initializing, please wait for ready signal...'

    try:
        rospy.init_node('ik_openrave',disable_signals=False)
        with env:
            env.Load(options.scene)
            robot = env.GetRobots()[0]
            # create ground right under the robot
            ab=robot.ComputeAABB()
            ground=RaveCreateKinBody(env,'')
            ground.SetName('map')
            ground.InitFromBoxes(array([r_[ab.pos()-array([0,0,ab.extents()[2]+0.002]),2.0,2.0,0.001]]),True)
            env.AddKinBody(ground,False)
            if options.mapframe is None:
                options.mapframe = robot.GetLinks()[0].GetName()
                rospy.loginfo('setting map frame to %s'%options.mapframe)
            collisionmap = RaveCreateSensorSystem(env,'CollisionMap bodyoffset %s topic %s'%(robot.GetName(),options.collision_map))
        
        valueslock = threading.Lock()
        listener = tf.TransformListener()
        values = robot.GetDOFValues()
        def UpdateRobotJoints(msg):
            with valueslock:
                with env:
                    for name,pos in izip(msg.name,msg.position):
                        j = robot.GetJoint(name)
                        if j is not None:
                            values[j.GetDOFIndex()] = pos
                    robot.SetDOFValues(values)

        def IKFn(req):
            global options
            with valueslock:
                with env:
                    if len(options.mapframe) > 0:
                        (robot_trans,robot_rot) = listener.lookupTransform(options.mapframe, robot.GetLinks()[0].GetName(), rospy.Time(0))
                        Trobot = matrixFromQuat([robot_rot[3],robot_rot[0],robot_rot[1],robot_rot[2]])
                        Trobot[0:3,3] = robot_trans
                        robot.SetTransform(Trobot)
                        goal = listener.transformPose(options.mapframe, req.pose_stamped)
                    else:
                        goal = req.pose_stamped
                    o = goal.pose.orientation
                    p = goal.pose.position
                    Thandgoal = matrixFromQuat([o.w,o.x,o.y,o.z])
                    Thandgoal[0:3,3] = [p.x,p.y,p.z]

                    if len(req.joint_state.name) > 0:
                        dofindices = [robot.GetJoint(name).GetDOFIndex() for name in req.joint_state.name]
                        robot.SetDOFValues(req.joint_state.position,dofindices)

                    res = orrosplanning.srv.IKResponse()

                    # resolve the ik type
                    iktype = None
                    if len(req.iktype) == 0:
                        iktype = IkParameterization.Type.Transform6D
                    else:
                        for value,type in IkParameterization.Type.values.iteritems():
                            if type.name.lower() == req.iktype.lower():
                                iktype = type
                                break
                        if iktype is None:
                            rospy.logerror('failed to find iktype %s'%(str(req.iktype)))
                            return None

                    ikp = IkParameterization()
                    if iktype == IkParameterization.Type.Direction3D:
                        ikp.SetDirection(Thandgoal[0:3,2])
                    elif iktype == IkParameterization.Type.Lookat3D:
                        ikp.SetLookat(Thandgoal[0:3,3])
                    elif iktype == IkParameterization.Type.Ray4D:
                        ikp.SetRay(Ray(Thandgoal[0:3,3],Thandgoal[0:3,2]))
                    elif iktype == IkParameterization.Type.Rotation3D:
                        ikp.SetRotation(quatFromRotationMatrix(Thandgoal[0:3,0:3]))
                    elif iktype == IkParameterization.Type.Transform6D:
                        ikp.SetTransform(Thandgoal)
                    elif iktype == IkParameterization.Type.Translation3D:
                        ikp.SetTranslation(Thandgoal[0:3,3])
                    elif iktype == IkParameterization.Type.TranslationDirection5D:
                        ikp.SetTranslationDirection5D(Ray(Thandgoal[0:3,3],Thandgoal[0:3,2]))

                    if len(req.manip_name) > 0:
                        manip = robot.GetManipulator(req.manip_name)
                        if manip is None:
                            rospy.logerror('failed to find manipulator %s'%req.manip_name)
                            return None
                    else:
                        manips = [manip for manip in robot.GetManipulators() if manip.GetEndEffector().GetName()==req.hand_frame_id]
                        if len(manips) == 0:
                            rospy.logerror('failed to find manipulator end effector %s'%req.hand_frame_id)
                            return None
                        manip = manips[0]

                    if manip.CheckIndependentCollision():
                        res.error_code.val = ArmNavigationErrorCodes.START_STATE_IN_COLLISION
                        return

                    robot.SetActiveManipulator(manip)
                    robot.SetActiveDOFs(manip.GetArmIndices())
                    
                    if manip.GetIkSolver() is None or not manip.GetIkSolver().Supports(IkParameterization.Type.TranslationDirection5D):
                        rospy.loginfo('generating ik for %s'%str(manip))
                        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=iktype)
                        if not ikmodel.load():
                            ikmodel.autogenerate()

                    if req.filteroptions & IKRequest.IGNORE_ENVIRONMENT_COLLISIONS:
                        filteroptions = 0
                    else:
                        filteroptions = IkFilterOptions.CheckEnvCollisions
                    if req.filteroptions & IKRequest.IGNORE_SELF_COLLISIONS:
                        filteroptions |= IkFilterOptions.IgnoreSelfCollisions
                    if req.filteroptions & IKRequest.IGNORE_JOINT_LIMITS:
                        filteroptions |= IkFilterOptions.IgnoreJointLimits

                    if req.filteroptions & IKRequest.RETURN_ALL_SOLUTIONS:
                        solutions = manip.FindIKSolutions(ikp,filteroptions)
                    elif req.filteroptions & IKRequest.RETURN_CLOSEST_SOLUTION:
                        solutions = manip.FindIKSolutions(ikp,filteroptions)
                        if solutions is not None and len(solutions) > 0:
                            # find closest configuration space distance with respect to source solution
                            weights = robot.GetActiveDOFWeights()
                            dists = []
                            for sol in solutions:
                                diff = robot.SubtractActiveDOFValues(sol,robot.GetActiveDOFValues())
                                dists.append(dot(weights,diff**2))
                            index = argmin(dists)
                            solutions = [solutions[index]]
                    else:
                        solution = manip.FindIKSolution(ikp,filteroptions)
                        if solution is not None:
                            solutions = [solution]
                        else:
                            solutions = []

                    if len(solutions) == 0:
                        res.error_code.val = ArmNavigationErrorCodes.NO_IK_SOLUTION
                    else:
                        res.error_code.val = ArmNavigationErrorCodes.SUCCESS
                        res.solutions.header.stamp = rospy.Time.now()
                        res.solutions.joint_names = [j.GetName() for j in robot.GetJoints(manip.GetArmIndices())]
                        for s in solutions:
                            res.solutions.points.append(trajectory_msgs.msg.JointTrajectoryPoint(positions=s))
                    return res

        def GetPositionIKFn(reqall):
            global options
            req=reqall.ik_request
            with valueslock:
                with env:
                    if len(options.mapframe) > 0:
                        (robot_trans,robot_rot) = listener.lookupTransform(options.mapframe, robot.GetLinks()[0].GetName(), rospy.Time(0))
                        Trobot = matrixFromQuat([robot_rot[3],robot_rot[0],robot_rot[1],robot_rot[2]])
                        Trobot[0:3,3] = robot_trans
                        robot.SetTransform(Trobot)
                        goal = listener.transformPose(options.mapframe, req.pose_stamped)
                    else:
                        goal = req.pose_stamped
                    o = goal.pose.orientation
                    p = goal.pose.position
                    Thandgoal = matrixFromQuat([o.w,o.x,o.y,o.z])
                    Thandgoal[0:3,3] = [p.x,p.y,p.z]

                    if len(req.robot_state.joint_state.name) > 0:
                        dofindices = [robot.GetJoint(name).GetDOFIndex() for name in req.robot_state.joint_state.name]
                        robot.SetDOFValues(req.robot_state.joint_state.position,dofindices)
                    #if len(req.robot_state.multi_dof_joint_state) > 0:
                    #    rospy.logwarn('ik_openrave.py does not support multi_dof_joint_state yet')

                    manips = [manip for manip in robot.GetManipulators() if manip.GetEndEffector().GetName()==req.ik_link_name]
                    if len(manips) == 0:
                        rospy.logerr('failed to find manipulator end effector %s'%req.ik_link_name)
                        return None
                    manip = manips[0]
                    rospy.logdebug('ik_openrave.py choosing %s manipulator'%manip.GetName())
                    robot.SetActiveManipulator(manip)

                    if manip.GetIkSolver() is None:
                        rospy.loginfo('generating ik for %s'%str(manip))
                        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
                        if not ikmodel.load():
                            ikmodel.autogenerate()


                    res = kinematics_msgs.srv.GetPositionIKResponse()
                    if manip.CheckIndependentCollision():
                        res.error_code.val = ArmNavigationErrorCodes.START_STATE_IN_COLLISION
                        return

                    filteroptions = IkFilterOptions.CheckEnvCollisions
                    solution = manip.FindIKSolution(Thandgoal,filteroptions)
                    if solution is None:
                        res.error_code.val = ArmNavigationErrorCodes.NO_IK_SOLUTION
                    else:
                        res.solution.joint_state.header.stamp = rospy.Time.now()
                        res.solution.joint_state.name = [j.GetName() for j in robot.GetJoints(manip.GetArmIndices())]
                        res.solution.joint_state.position = list(solution)
                    return res

        sub = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, UpdateRobotJoints,queue_size=1)
        s1 = rospy.Service('IK', orrosplanning.srv.IK, IKFn)
        s2 = rospy.Service('GetPositionIK', kinematics_msgs.srv.GetPositionIK, GetPositionIKFn)
        rospy.loginfo('openrave services ready: %s, %s'%(s1.resolved_name,s2.resolved_name))

        if options.ipython:
            from IPython.Shell import IPShellEmbed
            ipshell = IPShellEmbed(argv='',banner = 'Dropping into IPython',exit_msg = 'Leaving Interpreter, back to program.')
            ipshell(local_ns=locals())
        else:
            rospy.spin()
    finally:
        RaveDestroy()
