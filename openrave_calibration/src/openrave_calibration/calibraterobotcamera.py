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
import time, threading, pickle
import numpy # nice to be able to explicitly call some functions
from numpy import *
from optparse import OptionParser
from openravepy import *
from openravepy.interfaces import BaseManipulation

import rospy
from roslib import rostime
import message_filters
import std_msgs.msg
import sensor_msgs.msg

import cv
from cv_bridge import CvBridge, CvBridgeError

from scipy.optimize import leastsq
#from scipy.optimize.slsqp import approx_jacobian
from itertools import izip

class CalibrateRobotCamera(metaclass.AutoReloader):
    class CalibrationError(Exception):
        def __init__(self,value,action=None):
            self.value=value
            self.action=action
        def __str__(self):
            return repr(self.value)+'; action='+str(self.action)

    def __init__(self,image_raw='image_raw',pattern=None,KK=None,kc=None,timeout=4.0):
        self.image_raw=image_raw
        self.bridge = CvBridge()
        self.pattern=pattern
        px=self.pattern['corners_x']*self.pattern['spacing_x']
        py=self.pattern['corners_y']*self.pattern['spacing_y']
        self.pattern['type'] = """<KinBody name="calibration">
  <Body name="calibration">
    <Geom type="box">
      <extents>%f %f 0.001</extents>
      <translation>%f %f 0</translation>
      <diffusecolor>0 0.5 0</diffusecolor>
    </Geom>
    <Geom type="box">
      <extents>%f %f 0.002</extents>
      <translation>%f %f 0</translation>
      <diffusecolor>0 1 0</diffusecolor>
    </Geom>
  </Body>
</KinBody>
"""%(px*0.5+2.0*self.pattern['spacing_x'],py*0.5+2.0*self.pattern['spacing_y'],px*0.5,py*0.5,px*0.5,py*0.5,px*0.5,py*0.5)
        self.obstaclexml = """<KinBody name="obstacle">
  <Body name="obstacle">
    <Geom type="box">
      <extents>%f %f 0.001</extents>
      <translation>0 0 0.001</translation>
      <diffusecolor>1 0 0</diffusecolor>
    </Geom>
  </Body>
</KinBody>"""%(px+0.1,py+0.1)
        self.timeout=timeout
        self.cvKK = None if KK is None else cv.fromarray(KK)
        self.cvkc = None if kc is None else cv.fromarray(kc)

    def detect(self, cvimage):
        corners_x = self.pattern['corners_x']
        corners_y = self.pattern['corners_y']
        found, corners = cv.FindChessboardCorners(cvimage, (corners_x, corners_y), cv.CV_CALIB_CB_ADAPTIVE_THRESH)
        if found:
            board_corners = (corners[0], corners[corners_x  - 1], corners[(corners_y - 1) * corners_x], corners[len(corners) - 1])
            #find the perimeter of the checkerboard
            perimeter = 0.0
            for i in range(len(board_corners)):
                next = (i + 1) % 4
            xdiff = board_corners[i][0] - board_corners[next][0]
            ydiff = board_corners[i][1] - board_corners[next][1]
            perimeter += math.sqrt(xdiff * xdiff + ydiff * ydiff)
            #estimate the square size in pixels
            square_size = perimeter / ((corners_x - 1 + corners_y - 1) * 2)
            radius = int(square_size * 0.5 + 0.5)
            corners = array(cv.FindCornerSubPix(cvimage, corners, (radius, radius), (-1, -1), (cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 30, 0.01)),float64)
            return corners
        else:
            return None

    def getObjectPoints(self):
        object_points=zeros((self.pattern['corners_x']*self.pattern['corners_y'],3),float64)
        X,Y = meshgrid(arange(self.pattern['corners_x'])*self.pattern['spacing_x'],arange(self.pattern['corners_y'])*self.pattern['spacing_y'])
        object_points[:,0] = X.flatten()
        object_points[:,1] = Y.flatten()
        return object_points

    def calibrateIntrinsicCamera(self,all_object_points,all_corners,imagesize,usenonlinoptim=True,fixprincipalpoint=False,computegradients=False):
        pointCounts = vstack([len(object_points) for object_points in all_object_points])
        cvKK = cv.CreateMat(3,3,cv.CV_64F)
        cvkc = cv.CreateMat(5,1,cv.CV_64F)
        cvrvecs = cv.CreateMat(len(pointCounts),3,cv.CV_64F)
        cvtvecs = cv.CreateMat(len(pointCounts),3,cv.CV_64F)
        flags = cv.CV_CALIB_FIX_PRINCIPAL_POINT if fixprincipalpoint else 0
        cv.CalibrateCamera2(cv.fromarray(vstack(all_object_points)),cv.fromarray(vstack(all_corners)),cv.fromarray(pointCounts), (1024,768),cvKK,cvkc,cvrvecs,cvtvecs,flags)
        rvecs = array(cvrvecs)
        tvecs = array(cvtvecs)
        KK = array(cvKK)
        kc = array(cvkc)
        Ts = []
        for i in range(len(pointCounts)):
            T = matrixFromAxisAngle(rvecs[i])
            T[0:3,3] = tvecs[i]
            Ts.append(T)
        error = None
        if usenonlinoptim:
            # for some reason, the opencv solver is not doing as good of a job as it can... (it also uses levmarq)
            x0 = r_[KK[0,0],KK[1,1]]
            if not fixprincipalpoint:
                x0 = r_[x0,KK[0,2],KK[1,2]]
            x0 = r_[x0,kc[:,0]]
            for i in range(len(pointCounts)):
                x0 = r_[x0,rvecs[i],tvecs[i]]
            N = pointCounts[0]
            cv_image_points = cv.CreateMat(N,2,cv.CV_64F)
            cv_object_points = [cv.fromarray(x) for x in all_object_points]
            def errorfn(x):
                xoff = 2
                cvKK[0,0] = x[0]; cvKK[1,1] = x[1];
                if not fixprincipalpoint:
                    cvKK[0,2] = x[2]; cvKK[1,2] = x[3]
                    xoff += 2
                for i in range(5):
                    cvkc[i,0] = x[xoff+i]
                xoff += 5
                e = zeros(len(all_object_points)*N*2)
                off = 0
                for i in range(len(all_object_points)):
                    for j in range(3):
                        cvrvecs[0,j] = x[xoff+6*i+j]
                        cvtvecs[0,j] = x[xoff+6*i+3+j]
                    cv.ProjectPoints2(cv_object_points[i],cvrvecs[0],cvtvecs[0],cvKK,cvkc,cv_image_points)
                    image_points = array(cv_image_points)
                    e[off:(off+len(image_points))] = all_corners[i][:,0]-image_points[:,0]
                    off += len(image_points)
                    e[off:(off+len(image_points))] = all_corners[i][:,1]-image_points[:,1]
                    off += len(image_points)
                #print 'rms: ',sqrt(sum(e**2))
                return e

            x, success = leastsq(errorfn,x0,maxfev=100000,epsfcn=1e-7)
            if not success:
                raise CalibrationError('failed to converge to answer')
            e = errorfn(x)
            abse = sqrt(sum(e**2))
            e2=reshape(e,[len(all_object_points),2*N])**2
            error=mean(sqrt(e2[:,0:N]+e2[:,N:]),1)
            KK[0,0] = x[0]; KK[1,1] = x[1];
            xoff = 2
            if not fixprincipalpoint:
                KK[0,2] = x[2]; KK[1,2] = x[3]
                xoff += 2
            for i in range(5):
                kc[i,0] = x[xoff+i]
            if computegradients:
                deltas = r_[0.01*ones(2 if fixprincipalpoint else 4),0.0001*ones(5)]
                grad = []
                normalization = 1.0/(len(all_object_points)*N*2)
                for i,delta in enumerate(deltas):
                    x[i] += delta
                    e_p = errorfn(x)
                    abse_p = sqrt(sum(e_p**2))
                    x[i] -= 2*delta
                    e_n = errorfn(x)
                    abse_n = sqrt(sum(e_n**2))
                    x[i] += delta
                    grad.append(normalization*(abse_p+abse_n-2*abse)/(delta**2))#sum((e_p-e)**2) + sum((e-e_n)**2))/(2.0*delta))
                return KK,kc,Ts,error,array(grad)
        return KK,kc,Ts,error

    def validateCalibrationData(self,all_object_points,all_corners):
        """Builds up linear equations using method of Zhang 1999 "Flexible Camera Calibration By Viewing a Plane From Unknown Orientations". The condition number of the linear equations can tell us how good the data is. The object data all needs to lie on the Z=0 plane (actually any plane would probably do?)
        """
        cvH = cv.fromarray(eye(3))
        B = zeros((6,1))
        B[1,0] = 1 # skewness is 0 constraint
        # for every observation, solve for the homography and build up a matrix to solve for the intrinsicparameters
        for corners,object_points in izip(all_corners,all_object_points):
            if not all(object_points[:,2]==0):
                raise ValueError('The object data all needs to lie on the Z=0 plane.')
            
            cv.FindHomography(cv.fromarray(object_points[:,0:2]),cv.fromarray(corners),cvH,0)
            H = array(cvH)
            v = []
            for i,j in [(0,1),(0,0),(1,1)]:
                v.append([H[0,i]*H[0,j],H[0,i]*H[1,j]+H[1,i]*H[0,j],H[1,i]*H[1,j],H[2,i]*H[0,j]+H[0,i]*H[2,j],H[2,i]*H[1,j] + H[1,i]*H[2,j],H[2,i]*H[2,j]])
            B = c_[B,array(v[0]),array(v[1])-array(v[2])]
        U,s,Vh = linalg.svd(dot(B,transpose(B)))
        b = U[:,-1]
        A = array([[b[0],b[1],b[3]],[b[1],b[2],b[4]],[b[3],b[4],b[5]]])
        # A has to be positive definite
        Aeigs = linalg.eigvals(A)
        if not all(sign(Aeigs)>0) and not all(sign(Aeigs)<0):
            return None
        #print 'eigenvalues: ',sqrt(s)
        print 'condition is: ',sqrt(s[-2]/s[-1])
        if False:
            # compute the intrinsic parameters K = [alpha c u0; 0 beta v0; 0 0 1]
            b = U[:,-1]
            v0 = (b[1]*b[3]-b[0]*b[4])/(b[0]*b[2]-b[1]*b[1])
            lm = b[5]-(b[3]*b[3]+v0*(b[1]*b[3]-b[0]*b[4]))/b[0]
            alpha = sqrt(lm/b[0])
            beta = sqrt(lm*b[0]/(b[0]*b[2]-b[1]*b[1]))
            c = -b[1]*alpha*alpha*beta/lm
            u0 = c*v0/alpha - b[3]*alpha*alpha/lm
            print alpha,beta,u0,v0
        return sqrt(s)/len(all_object_points)

    def waitForRobot(self,robot):
        # wait for robot to stop moving
        lastvalues=robot.GetJointValues()
        print 'waiting for robot to stop...'
        while True:
            time.sleep(0.5) # sleep some time for better values difference
            newvalues=robot.GetJointValues()
            if sqrt(sum((lastvalues-newvalues)**2)) < 0.001:
                break
            lastvalues=newvalues
        time.sleep(0.5) # stabilize camera images?
        
    def waitForPattern(self,timeout=None,waitforkey=False,robot=None):
        # wait for robot to stop moving
        if robot:
            self.waitForRobot(robot)
        if timeout is None:
            timeout = self.timeout
        timestart=rospy.get_rostime()
        donecond = threading.Condition()
        data = [None]
        def detcb(imagemsg):
            if imagemsg.header.stamp > timestart:
                cvimage = self.bridge.imgmsg_to_cv(imagemsg,'mono8')
                corners = self.detect(cvimage)
                if corners is not None:
                    with donecond:
                        # get the pose with respect to the attached sensor tf frame
                        if self.cvKK is None or self.cvkc is None:
                            KK,kc,Ts,error = self.calibrateIntrinsicCamera([self.getObjectPoints()],[corners],(cvimage.width,cvimage.height),usenonlinoptim=False)
                            T = Ts[0]
                        else:
                            cvrvec = cv.CreateMat(1,3,cv.CV_64F)
                            cvtvec = cv.CreateMat(1,3,cv.CV_64F)
                            cv.FindExtrinsicCameraParams2(cv.fromarray(self.getObjectPoints()),cv.fromarray(corners),self.cvKK,self.cvkc,cvrvec,cvtvec)
                            T = matrixFromAxisAngle(array(cvrvec)[0])
                            T[0:3,3] = array(cvtvec)[0]
                        data[0] = {'T':T, 'corners_raw':corners, 'image_raw':array(cvimage)}
                        if 'type' in self.pattern:
                            data[0]['type'] = self.pattern['type']
                        donecond.notifyAll()
        image_sub = rospy.Subscriber(self.image_raw, sensor_msgs.msg.Image,detcb,queue_size=4)
        try:
            with donecond:
                donecond.wait(timeout)
                if data[0] is not None:
                    print 'found pattern'
        finally:
            image_sub.unregister()
        if waitforkey:
            cmd=raw_input('press any key to continue (q-quit)... ')
            if cmd == 'q':
                raise KeyboardInterrupt()
        return data[0]

    def waitForObjectDetectionMessage(self,timeout=None,waitforkey=False,robot=None):
        # wait for robot to stop moving
        if robot:
            self.waitForRobot(robot)
        if timeout is None:
            timeout = self.timeout
        timestart=rospy.get_rostime()
        donecond = threading.Condition()
        data = [None]
        def detcb(imagemsg, detmsg):
            if len(detmsg.objects) > 0 and detmsg.header.stamp > timestart:
                with donecond:
                    # get the pose with respect to the attached sensor tf frame
                    q = detmsg.objects[0].pose.orientation
                    t = detmsg.objects[0].pose.position
                    cvimage = self.bridge.imgmsg_to_cv(imagemsg)
                    data[0] = {'T':matrixFromPose([q.w,q.x,q.y,q.z,t.x,t.y,t.z]), 'type':detmsg.objects[0].type, 'image':array(cvimage)}
                    donecond.notifyAll()
        image_sub = message_filters.Subscriber(self.image_raw, sensor_msgs.msg.Image)
        det_sub = message_filters.Subscriber(self.objectdetection, posedetection_msgs.msg.ObjectDetection)
        ts = message_filters.TimeSynchronizer([image_sub, det_sub], 10)
        ts.registerCallback(detcb)
        try:
            with donecond:
                donecond.wait(timeout)
        finally:
            del ts # explicitly delete
        if waitforkey:
            cmd=raw_input('press any key to continue (q-quit)... ')
            if cmd == 'q':
                raise KeyboardInterrupt()
        return data[0]

    @staticmethod
    def moveRobot(robot,values):
        basemanip = BaseManipulation(robot)
        with robot:
            robot.SetActiveDOFs(arange(robot.GetDOF()))
            basemanip.MoveActiveJoints(values)
        while not robot.GetController().IsDone():
            time.sleep(0.01)

    def gatherCalibrationData(self,robot,sensorname,waitforkey=False,forcecalibintrinsic=False,calibextrinsic=True,maxconedist=0.1,maxconeangle=0.6,sensorrobot=None,**kwargs):
        waitcond=lambda: self.waitForPattern(robot=robot,waitforkey=waitforkey)
        origvalues=None
        env=robot.GetEnv()
        obstacle = None
        observations = []
        try:
            if forcecalibintrinsic or (self.cvKK is None or self.cvkc is None):
                origvalues=robot.GetJointValues()
                averagedist=0.03
                angledelta=0.3
                while True:
                    observations += examples.calibrationviews.CalibrationViews(robot=robot,sensorrobot=sensorrobot,sensorname=sensorname).computeAndMoveToObservations(waitcond=waitcond,usevisibility=False,angledelta=angledelta,maxconedist=maxconedist,maxconeangle=maxconeangle,averagedist=averagedist,**kwargs)
                    pickle.dump(observations,open('observations_initial.pp','w'))
                    try:
                        KKorig,kcorig,Tcamera,info = self.calibrateFromObservations(observations,False,fixprincipalpoint=True)
                        break
                    except self.CalibrationError,e:
                        print 'cannot compute stable KK, attempting more views',e
                        self.moveRobot(robot,origvalues)
                        averagedist = 0.02
                        angledelta=0.2
                print 'num observations is %d: '%len(observations),KKorig,kcorig
                self.cvKK = cv.fromarray(KKorig)
                self.cvkc = cv.fromarray(kcorig)
            if not calibextrinsic:
                return KKorig,kcorig,None,{'observations':observations}
            if origvalues is not None:
                print 'moving robot back to original values'
                self.moveRobot(robot,origvalues)

            # add an obstacle around the pattern
            if self.obstaclexml is not None:
                data=waitcond()
                obstacle = env.ReadKinBodyXMLData(self.obstaclexml)
                env.AddKinBody(obstacle,True)
                obstacle.SetTransform(dot(robot.GetSensor(sensorname).GetTransform(),data['T']))
            newobservations,target = examples.calibrationviews.CalibrationViews.gatherCalibrationData(robot=robot,sensorrobot=sensorrobot,sensorname=sensorname,waitcond=waitcond,**kwargs)
            observations += newobservations
            KKorig,kcorig,Tcamera,info=self.calibrateFromObservations(observations,Tcameraestimate=array(robot.GetSensor(sensorname).GetRelativeTransform(),float64))
            info['observations'] = observations
            return KKorig,kcorig,Tcamera,info
        finally:
            if obstacle is not None:
                env.RemoveKinBody(obstacle)

    def calibrateFromObservations(self,observations,calibextrinsic=True,pruneoutliers=True,full_output=True,fixprincipalpoint=False,Tcameraestimate=None):
        while True:
            if len(observations) < 3:
                raise self.CalibrationError('very little observations: %d!'%len(observations),action=1)
            object_points=self.getObjectPoints()
            all_object_points = []
            all_corners = []
            imagesize = None
            for o in observations:
                all_object_points.append(object_points)
                all_corners.append(o['corners_raw'])
                if imagesize:
                    assert imagesize[0]==o['image_raw'].shape[1] and imagesize[1]==o['image_raw'].shape[0]
                else:
                    imagesize = (o['image_raw'].shape[1],o['image_raw'].shape[0])
            intrinsiccondition = self.validateCalibrationData(all_object_points,all_corners)
            if intrinsiccondition is None:
                raise self.CalibrationError('bad condition number, %d observations'%(len(observations)),action=1)
            KKorig,kcorig,Traws,error_intrinsic,error_grad = self.calibrateIntrinsicCamera(all_object_points,all_corners,imagesize,fixprincipalpoint=fixprincipalpoint,computegradients=True)
            cvKKorig = cv.fromarray(KKorig)
            cvkcorig = cv.fromarray(kcorig)
            thresh=median(error_intrinsic)+0.5
            if any(error_intrinsic>thresh):
                # compute again using a pruned set of observations
                print 'pruning observations (thresh=%f) because intrinsic error is: '%thresh,error_intrinsic
                observations = [o for i,o in enumerate(observations) if error_intrinsic[i] <= thresh]
            else:
                if mean(error_intrinsic) > 0.6:
                    raise self.CalibrationError('intrinsic error is huge (%s)! giving up, KK=(%s)'%(str(error_intrinsic),str(KKorig)))
                break
        if not calibextrinsic:
            return KKorig,kcorig,None,{'error_intrinsic':error_intrinsic,'intrinsiccondition':intrinsiccondition,'error_grad':error_grad}
        if Tcameraestimate is None:
            raise TypeError('Tcameraestimate needs to be initialized to a 4x4 matrix')
        # unwarp all images and re-detect the checkerboard patterns
        cvKK = cv.CreateMat(3,3,cv.CV_64F)
        cv.GetOptimalNewCameraMatrix(cvKKorig,cvkcorig,imagesize,0,cvKK)
        cvUndistortionMapX = cv.CreateMat(imagesize[1],imagesize[0],cv.CV_32F)
        cvUndistortionMapY = cv.CreateMat(imagesize[1],imagesize[0],cv.CV_32F)
        cv.InitUndistortRectifyMap(cvKKorig,cvkcorig,cv.fromarray(eye(3)),cvKK,cvUndistortionMapX, cvUndistortionMapY)
        KK = array(cvKK)
        KKinv = linalg.inv(KK)
        if full_output:
            print 'KKorig: ',KKorig,'  KK:',KK
        cvimage = None
        cvkczero = cv.fromarray(zeros(kcorig.shape))
        cvrvec = cv.CreateMat(1,3,cv.CV_64F)
        cvtvec = cv.CreateMat(1,3,cv.CV_64F)
        all_optimization_data = []
        Tworldpatternestimates = []
        for i,o in enumerate(observations):
            cvimage_dist = cv.fromarray(o['image_raw'])
            if not cvimage:
                cvimage = cv.CloneMat(cvimage_dist)
            cv.Remap(cvimage_dist,cvimage,cvUndistortionMapX,cvUndistortionMapY,cv.CV_INTER_LINEAR+cv.CV_WARP_FILL_OUTLIERS)
            corners = self.detect(cvimage)
            if corners is not None:
                cv.FindExtrinsicCameraParams2(cv.fromarray(object_points),cv.fromarray(corners),cvKK,cvkczero,cvrvec,cvtvec)
                T = matrixFromAxisAngle(array(cvrvec)[0])
                T[0:3,3] = array(cvtvec)[0]
                Tworldpatternestimates.append(dot(dot(o['Tlink'],Tcameraestimate),T))
                all_optimization_data.append((transformPoints(KKinv,corners),linalg.inv(o['Tlink'])))
            else:
                print 'could not detect original pattern ',i

        # have the following equation: Tlink * Tcamera * Tdetectedpattern * corners3d = Tworldpattern * corners3d
        # need to solve for Tcamera and Tworldpattern
        # instead of using Tdetectedpattern, use projected difference:
        # corners - proj( inv(Tcamera) * inv(Tlink) * Tworldpattern *corners3d)
        corners3d = self.getObjectPoints()
        quatWorldPatternEstimates = array([quatFromRotationMatrix(T[0:3,0:3]) for T in Tworldpatternestimates])
        quatWorldPatternInitial,success = leastsq(lambda q: quatArrayTDist(q/sqrt(sum(q**2)),quatWorldPatternEstimates), quatWorldPatternEstimates[0],maxfev=100000,epsfcn=1e-6)
        
        rWorldPatternInitial = zeros(6,float64)
        rWorldPatternInitial[0:3] = axisAngleFromQuat(quatWorldPatternInitial)
        rWorldPatternInitial[3:6] = mean(array([T[0:3,3] for T in Tworldpatternestimates]),0)
        Tcameraestimateinv = linalg.inv(Tcameraestimate)
        rCameraInitial = zeros(6,float64)
        rCameraInitial[0:3] = axisAngleFromRotationMatrix(Tcameraestimateinv[0:3,0:3])
        rCameraInitial[3:6] = Tcameraestimateinv[0:3,3]
        def errorfn(x,optimization_data):
            Tworldpattern = matrixFromAxisAngle(x[0:3])
            Tworldpattern[0:3,3] = x[3:6]
            Tcamerainv = matrixFromAxisAngle(x[6:9])
            Tcamerainv[0:3,3] = x[9:12]
            err = zeros(len(optimization_data)*len(corners3d)*2)
            off = 0
            for measuredcameracorners,Tlinkinv in optimization_data:
                cameracorners3d = transformPoints(dot(dot(Tcamerainv,Tlinkinv),Tworldpattern),corners3d)
                iz=1.0/cameracorners3d[:,2]
                err[off:(off+len(corners3d))] = measuredcameracorners[:,0]-cameracorners3d[:,0]*iz
                off += len(corners3d)
                err[off:(off+len(corners3d))] = measuredcameracorners[:,1]-cameracorners3d[:,1]*iz
                off += len(corners3d)
            if full_output:
                print 'rms: ',sqrt(sum(err**2))
            return err

        optimization_data=all_optimization_data
        xorig, cov_x, infodict, mesg, iter = leastsq(lambda x: errorfn(x,all_optimization_data),r_[rWorldPatternInitial,rCameraInitial],maxfev=100000,epsfcn=1e-6,full_output=1)
        if pruneoutliers:
            # prune the images with the most error
            errors=reshape(errorfn(xorig,all_optimization_data)**2,(len(all_optimization_data),len(corners3d)*2))
            errorreprojection=mean(sqrt(KK[0,0]**2*errors[:,0:len(corners3d)]+KK[1,1]**2*errors[:,len(corners3d):]),1)
            #thresh=mean(errorreprojection)+std(errorreprojection)
            thresh=median(errorreprojection)+20#0.5*std(errorreprojection)
            print 'thresh:',thresh,'errors:',errorreprojection
            optimization_data = [all_optimization_data[i] for i in flatnonzero(errorreprojection<=thresh)]
            x, cov_x, infodict, mesg, iter = leastsq(lambda x: errorfn(x,optimization_data),xorig,maxfev=100000,epsfcn=1e-6,full_output=1)
        else:
            x=xorig
        Tcamerainv = matrixFromAxisAngle(x[6:9])
        Tcamerainv[0:3,3] = x[9:12]
        Tcamera = linalg.inv(Tcamerainv)
        Tworldpattern = matrixFromAxisAngle(x[0:3])
        Tworldpattern[0:3,3] = x[3:6]
        points3d = self.Compute3DObservationPoints(Tcamerainv,optimization_data)

        if full_output:
            errors=reshape(errorfn(x,optimization_data)**2,(len(optimization_data),len(corners3d)*2))
            error_reprojection=sqrt(KK[0,0]**2*errors[:,0:len(corners3d)]+KK[1,1]**2*errors[:,len(corners3d):]).flatten()
            print 'final reprojection error (pixels): ',mean(error_reprojection),std(error_reprojection)
            error_3d = sqrt(sum( (transformPoints(Tworldpattern,corners3d)-points3d)**2, 1))
            print '3d error: ',mean(error_3d),std(error_3d)
            return KKorig,kcorig,Tcamera,{'error_reprojection':error_reprojection,'error_3d':error_3d,'error_intrinsic':error_intrinsic,'intrinsiccondition':intrinsiccondition,'error_grad':error_grad,'KK':array(cvKK)}
        return KKorig,kcorig,Tcamera,None

    @staticmethod
    def Compute3DObservationPoints(Tcamerainv,optimization_data):
        Ps = [dot(Tcamerainv,Tlinkinv) for measuredcameracorners,Tlinkinv in optimization_data]
        points = zeros((len(optimization_data[0][0]),3))
        for i in range(len(points)):
            points[i] = CalibrateRobotCamera.Compute3DPoint(Ps,[measuredcameracorners[i] for measuredcameracorners,Tlinkinv in optimization_data])
        return points

    @staticmethod
    def Compute3DPoint(Ps,imagepoints):
        """Computes the 3D point from image correspondences"""
        A = zeros((2*len(imagepoints),3))
        b = zeros(2*len(imagepoints))
        for i in range(len(Ps)):
            A[2*i] = Ps[i][0,0:3]-imagepoints[i][0]*Ps[i][2,0:3]
            A[2*i+1] = Ps[i][1,0:3]-imagepoints[i][1]*Ps[i][2,0:3]
            b[2*i] = imagepoints[i][0]*Ps[i][2,3]-Ps[i][0,3]
            b[2*i+1] = imagepoints[i][1]*Ps[i][2,3]-Ps[i][1,3]
        x,residues,rank,s = linalg.lstsq(A,b)
        return x

    @staticmethod
    def symbolicderivatives():
        """symbolic derivatives using sympy, for now too complicated to use... quaternion representation is simpler though..."""
        axis0 = [Symbol('rx0'),Symbol('ry0'),Symbol('rz0'),Symbol('a0')]
        axis1 = [Symbol('rx1'),Symbol('ry1'),Symbol('rz1'),Symbol('a1')]
        Ts = [eye(4),eye(4),eye(4)]
        Ts[0][0:3,0:3]=ikfast.IKFastSolver.rodrigues(Matrix(3,1,axis0[0:3]),axis0[3])
        for i in range(3):
            for j in range(3):
                Ts[1][i,j] = Symbol('r%d%d'%(i,j))
        Ts[2][0:3,0:3]=ikfast.IKFastSolver.rodrigues(Matrix(3,1,axis1[0:3]),axis1[3])
        quats = [[Symbol('q%dx'%i),Symbol('q%dy'%i), Symbol('q%dz'%i), Symbol('q%dw'%i)] for i in range(3)]
        translations = [Matrix(3,1,[Symbol('t%dx'%i),Symbol('t%dy'%i), Symbol('t%dz'%i)]) for i in range(3)]        
        X = Matrix(3,1,[Symbol('x'),Symbol('y'),Symbol('z')])
        Xq = X
        for i in range(3):
            q = quats[i]
            xx = q[1] * q[1]
            xy = q[1] * q[2]
            xz = q[1] * q[3]
            xw = q[1] * q[0]
            yy = q[2] * q[2]
            yz = q[2] * q[3]
            yw = q[2] * q[0]
            zz = q[3] * q[3]
            zw = q[3] * q[0]
            Xq = translations[i]+Matrix(3,1,[(0.5-yy-zz)*Xq[0]+(xy-zw)*Xq[1]+(xz+yw)*Xq[2],(xy+zw)*Xq[0]+(0.5-xx-zz)*Xq[1]+(yz-xw)*Xq[2],(xz-yw)*Xq[0]+(yz+xw)*Xq[1]+(0.5-xx-yy)*Xq[2]])
        T = Ts[0]*Ts[1]*Ts[2]
        Xfinal=T[0:3,0:3]*X+T[0:3,3]
        cornerx=Xfinal[0]/Xfinal[2]
        cornery=Xfinal[1]/Xfinal[2]
        [diff(cornerx,axis0[i]) for i in range(i)] + [diff(cornerx,axis1[i]) for i in range(i)]

if __name__ == "__main__":
    parser = OptionParser(description='Views a calibration pattern from multiple locations.')
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/pa10calib.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--sensorname',action="store",type='string',dest='sensorname',default='wristcam',
                      help='Name of the sensor whose views to generate (default=%default)')
    (options, args) = parser.parse_args()

    rospy.init_node('calibraterobotcamera')#,disable_signals=False)
    env = Environment()
    try:
        env.SetViewer('qtcoin')
        env.Load(options.scene)
        env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        self = CalibrateRobotCamera()
        self.gatherCalibrationData(env.GetRobots()[0],sensorname=options.sensorname)
    finally:
        env.Destroy()

def test():
    import calibraterobotcamera
    rospy.init_node('calibraterobotcamera',disable_signals=False)
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('scenes/pa10lab.env.xml')
    robot = env.GetRobots()[0]
    sensorname='wristcam'
    self = calibraterobotcamera.CalibrateRobotCamera()
    self.gatherCalibrationData(robot,sensorname=sensorname)
