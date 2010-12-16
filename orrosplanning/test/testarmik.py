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
    rospy.init_node('armik_test')
    rospy.wait_for_service('IK')
    IKFn = rospy.ServiceProxy('IK',orrosplanning.srv.IK)
    req = orrosplanning.srv.IKRequest()
    req.pose_stamped.pose.position = geometry_msgs.msg.Point(0.6,0.189,0.7)
    req.pose_stamped.pose.orientation = geometry_msgs.msg.Quaternion(0,0.707,0,0.707)
    req.pose_stamped.header.frame_id = 'base_footprint'
    req.manip_name = 'leftarm'
    res=IKFn(req)
    print res
