#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('openraveros_tutorials') 
import rospy

from openraveros.srv import env_loadscene

if __name__ == "__main__":
    rospy.wait_for_service('openrave/env_loadscene')
    env_loadscene = rospy.ServiceProxy('openrave/env_loadscene', env_loadscene)
    env_loadscene(filename='data/lab1.env.xml',resetscene=1)
    
