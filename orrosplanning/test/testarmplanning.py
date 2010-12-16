#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__license__ = 'Apache License, Version 2.0'

import roslib; roslib.load_manifest('orrosplanning')
import rospy, time
import orrosplanning.srv
import geometry_msgs.msg
from numpy import *

if __name__=='__main__':
    rospy.init_node('armplanning_test')
    rospy.wait_for_service('MoveToHandPosition')
    MoveToHandPositionFn = rospy.ServiceProxy('MoveToHandPosition',orrosplanning.srv.MoveToHandPosition)
    req = orrosplanning.srv.MoveToHandPositionRequest()
#     req.hand_frame_id = 'l_gripper_palm_link'
#     req.hand_goal.pose.position = geometry_msgs.msg.Point(0.6,0.189,0.4765)
#     req.hand_goal.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
#     req.hand_goal.header.frame_id = 'base_footprint'
#     req.manip_name = 'leftarm'
#     res=MoveToHandPositionFn(req)
#     print res

    req.hand_frame_id = 'r_gripper_palm_link'
    req.hand_goal.pose.position = geometry_msgs.msg.Point(0.6,-0.189,0.8)
    req.hand_goal.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
    req.hand_goal.header.frame_id = 'base_footprint'
    req.manip_name = 'rightarm'
    res=MoveToHandPositionFn(req)
    print res
