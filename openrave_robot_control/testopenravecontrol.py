#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
PKG = 'openrave_robot_control'
import roslib; roslib.load_manifest(PKG)

from openravepy import *
from numpy import *
import time
import sys

env = Environment() # create openrave environment
env.SetViewer('qtcoin')
env.Load(sys.argv[1])
with env:
    robot = env.GetRobots()[0] # get the first robot
    manip = robot.GetManipulators()[0]
    jointnames = ' '.join(robot.GetJoints()[j].GetName() for j in manip.GetArmJoints())
    robot.SetController(RaveCreateController(env,'ROSOpenRAVE trajectoryservice /controller_session '+jointnames),range(robot.GetDOF()),0)
    lower,upper = robot.GetJointLimits()

# sending velocity command?
#robot.GetController().SendCommand("setvelocity 4 .01")

while True:
    with robot: # save the robot state and get random joint values that are collision free
        while True:
            values = lower + random.rand(len(lower))*(upper-lower)
            robot.SetJointValues(values)
            if not robot.CheckSelfCollision() and not env.CheckCollision(robot):
                break
    print 'setting: ',values
    robot.GetController().SetDesired(values)
    robot.WaitForController(0)
    time.sleep(1.0)

RaveDestroy()
