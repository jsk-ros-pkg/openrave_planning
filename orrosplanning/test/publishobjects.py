#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__license__ = 'Apache License, Version 2.0'

PKG = 'orrosplanning' # this package name
import roslib; roslib.load_manifest(PKG) 
import rospy, time
import posedetection_msgs.msg
from numpy import *

if __name__=='__main__':
    rospy.init_node('ObjectPublisher')
    pub_objdet = rospy.Publisher('/myobjects', posedetection_msgs.msg.ObjectDetection)
    starttime = time.time()
    while not rospy.is_shutdown():
        angle = 0.5*(time.time()-starttime)
        pose = posedetection_msgs.msg.Object6DPose()
        pose.type = 'package://openrave/share/openrave-0.3/data/mug1.kinbody.xml'
        pose.pose.orientation.w = 1
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.position.x = 0.5*cos(angle)
        pose.pose.position.y = 0.5*sin(angle)
        pose.pose.position.z = 0.5
        objdetmsg = posedetection_msgs.msg.ObjectDetection()
        objdetmsg.objects=[pose]
        objdetmsg.header.frame_id = 'map'
        objdetmsg.header.stamp = rospy.get_rostime()
        pub_objdet.publish(objdetmsg)
        
        # change the frame, this requires tf frames to be published
        objdetmsg.header.frame_id = 'l_forearm_cam_optical_frame'
        pub_objdet.publish(objdetmsg)
        time.sleep(0.05)

