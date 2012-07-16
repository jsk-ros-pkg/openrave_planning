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
    rospy.wait_for_service('MoveManipulator')
    MoveManipulatorFn = rospy.ServiceProxy('MoveManipulator',orrosplanning.srv.MoveManipulator)
    req = orrosplanning.srv.MoveManipulatorRequest()
    req.manip_goal = 0.5*ones(7)
    req.manip_name = 'rightarm'
    res=MoveManipulatorFn(req)
    print res
