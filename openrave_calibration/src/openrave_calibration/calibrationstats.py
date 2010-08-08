#!/usr/bin/env python
# Copyright (C) 2010 Rosen Diankov (rosen.diankov@gmail.com)
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
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import roslib; roslib.load_manifest('openrave_calibration')
import time
import numpy # nice to be able to explicitly call some functions
from numpy import *
from optparse import OptionParser
from openravepy import *

import calibraterobotcamera

try:
    from itertools import izip, combinations
except ImportError:
    def combinations(items,n):
        if n == 0: yield[]
        else:
            for  i in xrange(len(items)):
                for cc in combinations(items[i+1:],n-1):
                    yield [items[i]]+cc

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib.patches import Arrow, Polygon
    from matplotlib.collections import PatchCollection, PolyCollection, LineCollection
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
except ImportError:
    print 'failed to import matplotlib.pyplot'

def compareIntrinsicStatistics(self,observations):
    KKall = []
    KKall_fixed = []
    for obs in combinations(observations,5):
        try:
            KKorig,kcorig,Tcamera,info = self.calibrateFromObservations(obs,False,fixprincipalpoint=False)
            KKall.append([KKorig[0,0],KKorig[1,1],KKorig[0,2],KKorig[1,2],kcorig[0],kcorig[1],kcorig[2],kcorig[3],info['intrinsiccondition'],info['error_grad'][0]])
        except self.CalibrationError,e:
            print e
            pass

        try:
            KKorig,kcorig,Tcamera,info = self.calibrateFromObservations(obs,False,fixprincipalpoint=True)
            KKall_fixed.append([KKorig[0,0],KKorig[1,1],KKorig[0,2],KKorig[1,2],kcorig[0],kcorig[1],kcorig[2],kcorig[3],info['intrinsiccondition'],info['error_grad'][0]])
        except self.CalibrationError,e:
            print e
            pass

    KKall = array(KKall)
    KKall_fixed = array(KKall_fixed)
    print 'with principal:'
    print array([mean(KKall,0),std(KKall,0)])
    print 'fixed:'
    print array([mean(KKall_fixed,0),std(KKall_fixed,0)])
    return KKall,KKall_fixed

def compareIntrinsicGradients(self,observations):
    numimages = 5
    KKall = []
    KKall_fixed = []
    for obs in combinations(observations,numimages):
        try:
            KKorig,kcorig,Tcamera,info = self.calibrateFromObservations(obs,False,fixprincipalpoint=False)
            KKall.append([KKorig[0,0],info['error_grad'][0],mean(info['error_intrinsic']),info['intrinsiccondition'][0],info['intrinsiccondition'][-2],info['intrinsiccondition'][-1]])
        except self.CalibrationError,e:
            print e
            pass
        
        try:
            KKorig,kcorig,Tcamera,info = self.calibrateFromObservations(obs,False,fixprincipalpoint=True)
            KKall_fixed.append([KKorig[0,0],info['error_grad'][0],mean(info['error_intrinsic']),info['intrinsiccondition'][0],info['intrinsiccondition'][-2],info['intrinsiccondition'][-1]])
        except self.CalibrationError,e:
            print e
            pass

    KKall = array(KKall)
    KKall_fixed = array(KKall_fixed)
    fig = plt.figure()
    KK = KKall_fixed
    fig.clf()
#     ax = fig.add_subplot (1,2,1)
#     ax.set_title('f_x intrinsic calibration %d images'%numimages)
#     ax.set_ylabel('projection error')
#     #ax.set_xlabel('normalized gradient')
#     ax.plot(KK[:,4],KK[:,2],'o',markersize=2)
#     ax = fig.add_subplot (1,2,2)
    ax = fig.add_subplot (1,1,1)
    ax.set_title('f_x intrinsic calibration %d images'%numimages)
    ax.set_ylabel('f_x')
    ax.set_xlabel('intrinsic error')
    #ax.set_xlabel('normalized gradient')
    ax.plot(KK[:,4]/KK[:,5],KK[:,0],'o',markersize=2)
    ax.set_ylim(1800,2200)#mean(KK[:,0])-100,mean(KK[:,0])+100)
    fig.show()

    fig.clf()
    ax = fig.add_subplot (1,1,1)
    ax.set_title('f_x intrinsic calibration %d images'%numimages)
    ax.set_ylabel('f_x')
    ax.set_xlabel('second derivative of error')
    #ax.set_xlabel('normalized gradient')
    ax.plot(KK[:,1]KK[:,0],'o',markersize=2)
    ax.set_ylim(1800,2200)
    fig.show()

    return KKall,KKall_fixed
