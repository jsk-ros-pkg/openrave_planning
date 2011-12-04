#!/usr/bin/python
# -*- coding: utf-8 -*-
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
import roslib; roslib.load_manifest('orrosplanning')
import rospy
from openravepy import *
import sensor_msgs.msg
import geometry_msgs.msg
import tf
import tf.msg
import numpy

class ColladaJointPublisher():
    def __init__(self):
        description = rospy.get_param("robot_description")
        self.env=Environment()
        self.robot=self.env.ReadRobotXMLData(description)
        self.env.AddRobot(self.robot)
        self.env.SetViewer('qtcoin')
        # can use to set random values
        #self.robot.SetJointValues([1,1,1,1],[0,1,2,3])
        self.pub = rospy.Publisher('joint_states', sensor_msgs.msg.JointState)
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)

    def loop(self):
        hz = rospy.get_param("rate", 100) # 10hz
        r = rospy.Rate(hz) 

        # Publish Joint States
        while not rospy.is_shutdown():
            msg = sensor_msgs.msg.JointState()
            msg.header.stamp = rospy.Time.now()
            for j in self.robot.GetJoints():
                msg.name.append(j.GetName())
                msg.position.append(j.GetValues()[0])
            self.pub.publish(msg)
            transforms = []
            for link in self.robot.GetLinks():
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = msg.header.stamp
                t.child_frame_id = link.GetName()
                T = link.GetTransform()
                if len(link.GetParentLinks()) == 0:
                    t.header.frame_id = '/map'
                    Tparent = numpy.eye(4)
                else:
                    t.header.frame_id = link.GetParentLinks()[0].GetName()
                    Tparent = link.GetParentLinks()[0].GetTransform()
                T = numpy.dot(numpy.linalg.inv(Tparent),T)
                q = quatFromRotationMatrix(T[0:3,0:3])
                t.transform.translation.x = T[0,3]
                t.transform.translation.y = T[1,3]
                t.transform.translation.z = T[2,3]
                t.transform.rotation.x = q[1]
                t.transform.rotation.y = q[2]
                t.transform.rotation.z = q[3]
                t.transform.rotation.w = q[0]

                transforms.append(t)
            self.pub_tf.publish(tf.msg.tfMessage(transforms))
            r.sleep()

if __name__ == '__main__':
    try:
        RaveInitialize()
        rospy.init_node('collada_joint_publisher')
        jsp = ColladaJointPublisher()
        jsp.loop()
    except rospy.ROSInterruptException:
        pass
