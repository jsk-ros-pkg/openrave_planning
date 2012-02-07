#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('openraveros_tutorials')
import rospy
import os
from optparse import OptionParser
from openravepy import *
from openravepy.misc import OpenRAVEGlobalArguments

if __name__ == "__main__":
    parser = OptionParser(description='openrave planning example')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, args) = parser.parse_args()
    try:
        env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=False)
        env.SetViewer('qtcoin')
        RaveLoadPlugin(os.path.join(roslib.packages.get_pkg_dir('openraveros'),'lib','openraveros'))
        namespace = 'openrave'
        if env.AddModule(RaveCreateModule(env,'rosserver'),namespace) != 0:
            raise ValueError('failed to create openraveros server')
        
        raw_input('press any key to exit')
    finally:
        RaveDestroy()
