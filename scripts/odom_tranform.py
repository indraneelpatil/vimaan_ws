#!/usr/bin/env python
#Author: Lentin Joseph, Qbotics Labs
#Email: qboticslabs@gmail.com


import rospy
import sys
import tf
from tf import TransformListener, TransformerROS,transformations, TransformBroadcaster

import tf2_ros
from geometry_msgs.msg import PoseStamped,TransformStamped,Pose,Point,Quaternion, PoseWithCovarianceStamped,TwistWithCovariance
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float32

from vimaan_aruco_navigation.msg import ArucoBoardData
import roslaunch
import nodelet
from nodelet.srv import NodeletUnload
import os
import rospkg


import numpy as np

from realsense2_camera.srv import ResetCamera
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from multiprocessing import Process
#import multiprocessing
import time
import signal
import psutil


resonance_count=0
rviz_debugging_flag=True                 # Rviz marker visualization for jevois and realsense messages
jv_filter_flag=True                 # Yaw outlier filter on incoming jevois readings

twist=TwistWithCovariance()
first_aruco=0                       #To trigger the first static transform world -> base_link_origin
count=0
covariance=[]
uuid=str(0)



###############################################################################################3
###                                     rviz marker declarations
#################################################################################################
marker_array=MarkerArray()
marker = Marker()

marker.header.frame_id = "world"             # Frame for jevois data
marker.type = marker.ARROW
marker.action = marker.ADD
marker.color.a = 1.0
marker.scale.x = 0.3
marker.scale.y = 0.09
marker.scale.z = 0.05
marker.id=1

marker_array_rs=MarkerArray()
marker_rs = Marker()

marker_rs.header.frame_id = "base_link_origin"
marker_rs.type = marker.ARROW
marker_rs.action = marker.ADD
marker_rs.color.a = 1.0
marker_rs.scale.x = 0.3
marker_rs.scale.y = 0.09
marker_rs.scale.z = 0.05
marker_rs.id=1

marker_array_fuse=MarkerArray()
marker_fuse = Marker()

marker_fuse.header.frame_id = "world"
marker_fuse.type = marker.ARROW
marker_fuse.action = marker.ADD
marker_fuse.color.a = 0.1
marker_fuse.scale.x = 0.3
marker_fuse.scale.y = 0.09
marker_fuse.scale.z = 0.05
marker_fuse.id=1
########################################################################################################3#######33

# Rviz markers for debugging and visualization
pub_mark=rospy.Publisher('/position_jv',MarkerArray,queue_size=10)
pub_rs=rospy.Publisher('/position_rs',MarkerArray,queue_size=10)
pub_fuse=rospy.Publisher('/position_fuse',MarkerArray,queue_size=10)


# ROS Publishers for robot_localization package in the required message type
pub_odometry=rospy.Publisher('/tracker_odom',Odometry,queue_size=10)
pub_jevois=rospy.Publisher('jv_pose_new',PoseWithCovarianceStamped,queue_size=10)

# Yaw angle publishers
pub_jv_yaw=rospy.Publisher('/jv_yaw',Float32,queue_size=10)
pub_rs_yaw=rospy.Publisher('/rs_yaw',Float32,queue_size=10)


# JV : Yellow


# Global params
it, ix, iy, iz, iRoll, iPitch, iYaw, iSrs, nEventCols = range(9)
iPos = [ix, iy, iz]
iAngle = [iRoll, iPitch, iYaw]
iCoords = iPos + iAngle
srsDict = {'RealSense':0, 'Jevois':1}



def static_transform_world_base_link_origin(msg):
	print "Static Transform from world to base_link_origin published"

	broadcaster = tf2_ros.StaticTransformBroadcaster()
	static_transformStamped = TransformStamped()
  
	static_transformStamped.header.stamp = rospy.Time.now()
	static_transformStamped.header.frame_id = "world"
	static_transformStamped.child_frame_id = "base_link_origin"
  
	static_transformStamped.transform.translation.x = msg.pose.translation.x
	static_transformStamped.transform.translation.y = msg.pose.translation.y
	static_transformStamped.transform.translation.z = msg.pose.translation.z

	static_transformStamped.transform.rotation.x = msg.pose.rotation.x
	static_transformStamped.transform.rotation.y = msg.pose.rotation.y
	static_transformStamped.transform.rotation.z = msg.pose.rotation.z
	static_transformStamped.transform.rotation.w = msg.pose.rotation.w
 
	broadcaster.sendTransform(static_transformStamped)


# ##  RS fusion with JV class (by Nikolay, with Sumil rotation)
# In[2]:
def is_any_none(arr):
    # Check if None, or Iterable (Array/Lis/Tuple) with at least single None in it
    if arr is None:
        return True
    
    for elem in arr:
        if elem is None:
            return True
        
    return False

def time_stamp(stamp, sec2nsec=1e-9):
    # Calculate time in seconds
    return stamp.secs + stamp.nsecs*sec2nsec
    
def plot_res(eventsRS, eventsJV, fixedPosRS, settingTxt):
    # Presnt RS, JV and Fixed/Fused data
    if isinstance(eventsRS, list):
        eventsRS = np.array(eventsRS)
    if isinstance(eventsJV, list):
        eventsJV = np.array(eventsJV)
    if isinstance(fixedPosRS, list):
        fixedPosRS = np.array(fixedPosRS)
    
    
    # Present the outcome
    nPosCols = len(iCoords)
    iTime = 0
    fig, ax = plt.subplots(figsize=(20, 10), nrows=nPosCols, ncols=1)
    coordNames = ['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'Srs']
    posLgnd = ['RealSense', 'Jevois', 'fixedRS']
    for iAx, iCoord in enumerate(iCoords):
        axis = ax[iAx]
        axis.plot(eventsRS[:, iTime], eventsRS[:, iCoord],
                  color='green', marker='s', linestyle='None', linewidth=2, markersize=6)    # 'ro',
        axis.plot(eventsJV[:, iTime], eventsJV[:, iCoord],
                  color='blue', marker='*', linestyle='None', linewidth=2, markersize=4)    #, 'b.',
        axis.plot(fixedPosRS[:, iTime], fixedPosRS[:, iCoord],
                  color='red', marker='.', linestyle='None', linewidth=2, markersize=4)    #, , 'g*')

        axis.set_title(coordNames[iCoord]+settingTxt)
        axis.legend(posLgnd)
        axis.set_xlabel(coordNames[iTime])
        if iCoord in iAngle:
            y_lbl='Angle'
        elif iCoord in iPos:
            y_lbl='Pos'
        else:
            y_lbl=''
        y_lbl += '- ' + coordNames[iCoord]
        axis.set_ylabel(y_lbl)
    # fig.set_size_inches(18.5, 10.5)
    plt.show()
    print(settingTxt+' done.\n')
    
def euler_to_quaternion(yaw, pitch, roll, isDeg=False):
    if isDeg:
        # Convert Degrees inputs to Radians
        yaw, pitch, roll = np.radians(yaw), np.radians(pitch), np.radians(roll)
    cos_r, sin_r = np.cos(roll/2), np.sin(roll/2)
    cos_p, sin_p = np.cos(pitch/2), np.sin(pitch/2)
    cos_y, sin_y = np.cos(yaw/2), np.sin(yaw/2)

    qx = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
    qy = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
    qz = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
    qw = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
    return (qx, qy, qz, qw)
    
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    # X = math.degrees(math.atan2(t0,t1)) # math.atan2(t0, t1)
    X = (math.atan2(t0,t1)) # math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    #Y = math.degrees(math.asin(t2))
    Y=(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    #Z = math.degrees(math.atan2(t3,t4)) # math.atan2(t3, t4)
    Z = (math.atan2(t3,t4)) # math.atan2(t3, t4)

    return (X,Y,Z)

def transfrom(x,y,z,u,v,w,deltax,deltay,deltaz,du,dv,dw, isDeg=False):
    if isDeg:
        # Convert Degrees inputs to Radians
        u,v,w = np.radians(u), np.radians(v), np.radians(w)
        du,dv,dw = np.radians(du), np.radians(dv), np.radians(dw)
        
    R = np.array([[np.cos(v)*np.cos(w),
                   np.sin(u)*np.sin(v)*np.cos(w)-np.cos(u)*np.sin(w),
                   np.sin(u)*np.sin(w)+np.cos(u)*np.sin(v)*np.cos(w)],
                  [np.cos(v)*np.sin(w),
                   np.cos(u)*np.cos(w)+np.sin(u)*np.sin(v)*np.sin(w),
                   np.cos(u)*np.sin(v)*np.sin(w)-np.sin(u)*np.cos(w)],
                  [-np.sin(v),
                   np.sin(u)*np.cos(v),
                   np.cos(u)*np.cos(v)]])
    Rd = np.array([[np.cos(dv)*np.cos(dw),
                    np.sin(du)*np.sin(dv)*np.cos(dw)-np.cos(du)*np.sin(dw),
                    np.sin(du)*np.sin(dw)+np.cos(du)*np.sin(dv)*np.cos(dw)],
                   [np.cos(dv)*np.sin(dw),
                    np.cos(du)*np.cos(dw)+np.sin(du)*np.sin(dv)*np.sin(dw),
                    np.cos(du)*np.sin(dv)*np.sin(dw)-np.sin(du)*np.cos(dw)],
                   [-np.sin(dv),
                    np.sin(du)*np.cos(dv),
                    np.cos(du)*np.cos(dv)]])
    RH = np.array([[np.cos(dv)*np.cos(dw),
                    np.sin(du)*np.sin(dv)*np.cos(dw)-np.cos(du)*np.sin(dw),
                    np.sin(du)*np.sin(dw)+np.cos(du)*np.sin(dv)*np.cos(dw),deltax],
                   [np.cos(dv)*np.sin(dw),
                    np.cos(du)*np.cos(dw)+np.sin(du)*np.sin(dv)*np.sin(dw),
                    np.cos(du)*np.sin(dv)*np.sin(dw)-np.sin(du)*np.cos(dw),deltay],
                   [-np.sin(dv),
                    np.sin(du)*np.cos(dv),
                    np.cos(du)*np.cos(dv),deltaz],
                   [0,0,0,1]])
    
    T= np.array([x,y,z,1])
    Tprime=np.dot(RH,T)
    X=Tprime[0]
    Y=Tprime[1]
    Z=Tprime[2]
    Rprime=np.dot(R,Rd)
    W=math.atan2(Rprime[1,0],Rprime[0,0]) # math.atan2/9Rprime[1,0],Rprime[0,0])
    V=math.asin(-Rprime[2,0])
    U=math.atan2(Rprime[2,1],Rprime[2,2])   # math.atan2(Rprime[2,1]/Rprime[2,2]) 
    
    return  (X,Y,Z,U,V,W)

def transfrom2(x,y,z,u,v,w,du,dv,dw,error_x,error_y,error_z,isDeg=False):
    if isDeg:
        # Convert Degrees inputs to Radians
        u,v,w = np.radians(u), np.radians(v), np.radians(w)
        #du,dv,dw = np.radians(du), np.radians(dv), np.radians(dw)
    
    RS = np.array([[np.cos(v)*np.cos(w),
                   np.sin(u)*np.sin(v)*np.cos(w)-np.cos(u)*np.sin(w),
                   np.sin(u)*np.sin(w)+np.cos(u)*np.sin(v)*np.cos(w)],
                  [np.cos(v)*np.sin(w),
                   np.cos(u)*np.cos(w)+np.sin(u)*np.sin(v)*np.sin(w),
                   np.cos(u)*np.sin(v)*np.sin(w)-np.sin(u)*np.cos(w)],
                  [-np.sin(v),
                   np.sin(u)*np.cos(v),
                   np.cos(u)*np.cos(v)]])
    Corr = np.array([[np.cos(dv)*np.cos(dw),
                    np.sin(du)*np.sin(dv)*np.cos(dw)-np.cos(du)*np.sin(dw),
                    np.sin(du)*np.sin(dw)+np.cos(du)*np.sin(dv)*np.cos(dw)],
                   [np.cos(dv)*np.sin(dw),
                    np.cos(du)*np.cos(dw)+np.sin(du)*np.sin(dv)*np.sin(dw),
                    np.cos(du)*np.sin(dv)*np.sin(dw)-np.sin(du)*np.cos(dw)],
                   [-np.sin(dv),
                    np.sin(du)*np.cos(dv),
                    np.cos(du)*np.cos(dv)]])

    Rprime=np.dot(Corr,RS)
    T= np.array([x,y,z])
    Tprime=np.dot(Corr,T)
    
    X=Tprime[0]+error_x
    Y=Tprime[1]+error_y
    Z=Tprime[2]+error_z
    
    W=math.atan2(Rprime[1,0],Rprime[0,0])
    V=-math.asin(Rprime[2,0])
    U=math.atan2(Rprime[2,1],Rprime[2,2])
    #print(X,Y,Z)
    return  (X,Y,Z,U,V,W)


def rot_to_7_coords(rotation):
    xRot, yRot, zRot, wRot = rotation.x, rotation.y, rotation.z, rotation.w
    roll, pitch, yaw = quaternion_to_euler(xRot, yRot, zRot, wRot)
    return (xRot, yRot, zRot, wRot, roll, pitch, yaw)

def msg_from_topic(position, rotation, time, srs):
    x, y, z = position.x, position.y, position.z
    xRot, yRot, zRot, wRot, roll, pitch, yaw = rot_to_7_coords(rotation)
    msg = [time, x, y, z, roll, pitch, yaw, srs]
    return msg 

def getoffsets(posJV,posRS):
        dx=posJV[ix]#-rs_x[-1]
        dy=posJV[iy]#-rs_y[-1]
        dz=posJV[iz]#-rs_z[-1]
        du=0#posJV[iRoll]#-rs_roll[-1]
        dv=0#posJV[iPitch]#-rs_pitch[-1]
        dw=posJV[iYaw]#-rs_yaw[-1]
        rs_x_ctr,rs_y_ctr,rs_z_ctr = posRS[ix],posRS[iy],posRS[iz]
        u=0#posRS[iRoll]
        v=0#posRS[iPitch]
        w=posRS[iYaw]
    	RS = np.array([[np.cos(v)*np.cos(w),
                   np.sin(u)*np.sin(v)*np.cos(w)-np.cos(u)*np.sin(w),
                   np.sin(u)*np.sin(w)+np.cos(u)*np.sin(v)*np.cos(w)],
                  [np.cos(v)*np.sin(w),
                   np.cos(u)*np.cos(w)+np.sin(u)*np.sin(v)*np.sin(w),
                   np.cos(u)*np.sin(v)*np.sin(w)-np.sin(u)*np.cos(w)],
                  [-np.sin(v),
                   np.sin(u)*np.cos(v),
                   np.cos(u)*np.cos(v)]])
    	JV = np.array([[np.cos(dv)*np.cos(dw),
                    np.sin(du)*np.sin(dv)*np.cos(dw)-np.cos(du)*np.sin(dw),
                    np.sin(du)*np.sin(dw)+np.cos(du)*np.sin(dv)*np.cos(dw)],
                   [np.cos(dv)*np.sin(dw),
                    np.cos(du)*np.cos(dw)+np.sin(du)*np.sin(dv)*np.sin(dw),
                    np.cos(du)*np.sin(dv)*np.sin(dw)-np.sin(du)*np.cos(dw)],
                   [-np.sin(dv),
                    np.sin(du)*np.cos(dv),
                    np.cos(du)*np.cos(dv)]])
        Corr=np.dot(JV,np.linalg.inv(RS))
	#rscorrmat=np.dot(Corr,RS)
        rscorr=np.dot(Corr,np.array([rs_x_ctr,rs_y_ctr,rs_z_ctr]))
        error_x=posJV[ix]-rscorr[0]
        error_y=posJV[iy]-rscorr[1]
        error_z=posJV[iz]-rscorr[2]        
        error_W=math.atan2(Corr[1,0],Corr[0,0])
        error_V=-math.asin(Corr[2,0])
        error_U=math.atan2(Corr[2,1],Corr[2,2])
        #print(error_x,error_y,error_z,error_W,error_V,error_U)
        out = [error_x, error_y, error_z, error_U,error_V,error_W]
        return out    


# In[3]:

def is_any_none(arr):
    # Check if None, or Iterable (Array/Lis/Tuple) with at least single None in it
    if arr is None:
        return True
    
    for elem in arr:
        if elem is None:
            return True
        
    return False



class MergeJV2RS:
    ## Constructor
    def __init__(self, nDataCols=nEventCols):
       # Set user nipu params
        # Enale Nikolay's Fusion
        self.isApplyFusion = True
        # Enables Sumil's compensation for Drone angular and position movemenemts
        self.isFixAngles = True
        
        # Set alg. Parameters
        self.nMaxFuseTable = 0 # 5
        self.nMaxTransTable = 1 # 5
        self.clock2sec = 1e-9
        self.maxErrAge = 15/self.clock2sec    # value in seconds, converted to clocks
        self.isUpdErrOnJV = True
        self.factorJV = 0.1
        self.minConf = 0.65
        self.prevFix = 0
        self.isArucoSeen = False
        

        # Set init values
        dummyEvent = np.array([None]*nDataCols)     # np.zeros(shape=nDataCols)
        self.currRS  = dummyEvent.copy()
        self.currRawRS= dummyEvent.copy()
        self.offsetRS= dummyEvent.copy()
        self.prevJV  = dummyEvent.copy()
        self.fixedRS = None # dummyEvent.copy()
        self.errZeroTpl = (np.zeros(shape=nDataCols, dtype=np.float), 0.0)      # default 0 zero pos err and conf
        self.noErr = (self.errZeroTpl, 0.0)
        self.errTransData, self.errFuseData = self.noErr, self.noErr
        
        self.errFuseTable = None
        self.errTransTable = None

    def setCurrRS(self, event):
        # Set RS data from event if it is RS
        eventSrs = int(event[iSrs])
        if eventSrs == srsDict['RealSense']:
            eventRotRS = event[:].copy() # Use the arriving data
            # RealSense data arrived
            
            if self.isFixAngles:
                self.currRawRS = eventRotRS.copy() # The raw arriving RS event

                if self.isArucoSeen:
                    xRS, yRS, zRS = event[ix], event[iy], event[iz]
                    rollRS, pitchRS, yawRS = event[iRoll], event[iPitch], event[iYaw]
                    # c_ofst = self.offsetRS
                    uErrTrans, errTransConf = self.findErr(event=event, isTrans=True) # get the error
                    
                    if not is_any_none(uErrTrans) and errTransConf > 0: #errConf > self.minConf:
                        dx, dy, dz = uErrTrans[ix],    uErrTrans[iy],     uErrTrans[iz]
                        du, dv, dw = uErrTrans[iRoll], uErrTrans[iPitch], uErrTrans[iYaw]
			
                        ## Calculate the corrected transfomred RS
                        eventRotRS[ix], eventRotRS[iy], eventRotRS[iz],eventRotRS[iRoll], eventRotRS[iPitch], eventRotRS[iYaw] = transfrom2(xRS, yRS, zRS, 0, 0, yawRS, du, dv, dw, dx, dy, dz)

			#print(eventRotRS)

            self.currRS = eventRotRS
    
    def setCurrJV(self, event):
        # Return JV data from event if it is JV, or previous value otherwise
        eventSrs = int(event[iSrs])
        isJevois = int(event[iSrs]) == srsDict['Jevois']
        if isJevois:
            # Jevois data arrived
            self.setFirstArucoPos(isJevois=isJevois)                        
            # Save the rotated data
            self.currJV = event[:].copy() # Use the arriving data
            
            if self.isFixAngles:
                # Find the coordinates offset or RS and JV 
                self.findJVOffset2RawRS(event)
            
    def getLastRawRS(self):
        return self.currRawRS.copy()
        
    def getLastRS(self):
        # Return last RS
        return self.currRS.copy()
    
    def getLastJV(self):
        # Return last RS
        return self.currJV.copy()

    def promoteErrTable(self, newErr=None, errTable=None, nMaxTable=0):
        # Remove old/obsolete entries from table, then add new error
        if nMaxTable == 0:
            return errTable # Do nothing
        if errTable is not None:
            # pdb.set_trace()
            errAge = self.now - errTable[:, it]
            isBadEntry = errAge > self.maxErrAge
            if isBadEntry.any():
                # Remove old entries
                errTable = errTable[np.logical_not(isBadEntry), :]

            # Remove obsolete entires from table
            nSamples = errTable.shape[0]
            nObsoleteSamples = nSamples - nMaxTable + 1
            if nObsoleteSamples > 0:
                isStaying = np.ones(shape=nSamples, dtype=np.bool)
                isStaying[:nObsoleteSamples] = False
                # Remove first nObsoleteSamples entries
                errTable = errTable[isStaying, :]

            if errTable.size == 0:
                errTable = None
        # Add new entries to the error table
        if not is_any_none(newErr):
            # Add new entries
            newErr = newErr.reshape(1, -1)
            if errTable is None:
                errTable = newErr
            else:
                errTable = np.append(errTable, newErr, axis=0) # add error resulting from current JeVois
        return errTable

    def estErrFromTable(self, errTable=None, nMaxTable=0):
        # Calculate the error from the table with confidence
        if errTable is None or nMaxTable == 0 or errTable.size == 0:
            return self.noErr # Table is empty
        uErr = errTable.mean(axis=0) # the average of the error Table

        # Propose an estimation of the errConf, based on Table size, values uniformity (STD) and Table entries age
        stdErr = errTable.std(axis=0)
        errAge = self.now - uErr[it]
        # Error confidence decreases with age, and with std
        # Error is defined by postion error only, not conisdering angle error!
        posSTD2Mean = stdErr[iPos].mean()
        stdConf = 1/(posSTD2Mean + 1)               # TODO Scaling is needed- see relevant STD values
        ageConf = 1/(errAge*self.clock2sec + 1)     # TODO Scaling is needed- see relevant errAge values

        # Larger number of error measurement improves confidence- confidence increases with nTableErr
        nTableErr = errTable.shape[0]
        tableSizeConf = 1 if nTableErr >= nMaxTable else  nTableErr/nMaxTable

        errConf = stdConf*ageConf*tableSizeConf

        erplotrConf = min([max([0.0, errConf]), 1.0])       # Limit so errConf would be [0, 1]
        return (uErr, errConf)

    
    
    def findJVOffset2RawRS(self, event):
        # Update error for every 'Jevois' input, otherwise save as is        
        isJevois = int(event[iSrs]) == srsDict['Jevois']
        if isJevois:
            eventRS = self.getLastRawRS()#[iCoords]
            if not is_any_none(eventRS): # not None in posRS:
                eventJV = self.getLastJV()#[iCoords]              
                self.offsetRS = event.copy()
                self.offsetRS[iCoords] = getoffsets(eventJV, eventRS)

        
    def findErr(self, event, isTrans=True):
        # Update error for every 'Jevois' input, otherwise save as is
        isJevois = int(event[iSrs]) == srsDict['Jevois']
        if isJevois:
            posRS = self.getLastRS()[iCoords] 
            if not is_any_none(posRS): # not None in posRS:
                if isTrans:
                    errRS = self.offsetRS # Offset in this case

                    # Modify the table
                    self.errTransTable = self.promoteErrTable(errRS, self.errTransTable, self.nMaxTransTable)
                    self.errTransData = self.estErrFromTable(self.errTransTable, self.nMaxTransTable)   # Store the current err data
                else:
                    posJV = self.getLastJV()[iCoords]
                    errRS = event.copy()
                    errRS[iCoords] = posRS- posJV
                
                    # Modify the table
                    self.errFuseTable = self.promoteErrTable(errRS, self.errFuseTable, self.nMaxFuseTable)
                    self.errFuseData = self.estErrFromTable(self.errFuseTable, self.nMaxFuseTable)   # Store the current err data

        return self.errTransData if isTrans else self.errFuseData
    

    def setFirstArucoPos(self, isJevois):
        if self.isArucoSeen or not isJevois:
            # Wait for first Aruco, before and after that do nothing!
            return
        
        # Else: The first Aruco has arrived!
        self.isArucoSeen = True
        
    def process(self, event, orientation):
        self.now = event[it]
        self.setCurrRS(event)
        self.setCurrJV(event)
        uErrTrans, errTransConf = self.findErr(event=event, isTrans=True)
        
        if not self.isArucoSeen:
            return
        
        uErr, errConf = self.findErr(event=event, isTrans=False)
        
        eventSrs = int(event[iSrs])
        isRS = eventSrs == srsDict['RealSense']
        if isRS:
            if errConf > self.minConf:
                currFix = errConf*uErr[iCoords]
                self.prevFix = currFix
            else:
                currFix = self.prevFix   
            fixedEvent = self.getLastRS().copy()
            if self.isApplyFusion:
                # Add the error found by Jevois scaled by the confidence
		factor=1        
		fixedEvent[iCoords] = fixedEvent[iCoords] - currFix*factor

                ## IIR current fixedPos with previous to have smooth out
                # fixedEvent[iCoords] = errConf*fixedEvent[iCoords] + (1-errConf)*self.fixedRS[iCoords]
            self.fixedRS = fixedEvent
        elif self.fixedRS is None:
            self.fixedRS = event.copy()


# ## Main code part: read bag file and process (Transform and Fuse) all topics messages to return modified position

# In[9]:



class Drone_Odometry:

	def __init__(self,odom_selector):
		rospy.init_node('robot_trans')
		
		self.pub = rospy.Publisher('/mavros/vision_pose/original', PoseStamped, queue_size=10)
		
		
		
		#This number will select which topic will publish to mavros topic
		self.odom_selector = odom_selector
		
		#Subscribing odometry
    		rospy.Subscriber('/camera/odom/sample',Odometry,self.odometryCb)
    		#Subscribing jevois data
    		rospy.Subscriber('/aruco_localization_node/jevois/aruco_board/data',ArucoBoardData,self.jeovisCb)

    		self.rate = rospy.Rate(20.0)

    		self.listener = tf.TransformListener()
		self.r = PoseStamped()
		self.jevois_msg = PoseStamped()
		self.sensor_fused = PoseStamped()
		
		
		self.real_sense_odom_enable = False
		self.jevois_odom_enable = False
		#Sensor fusion object
		self.MergeV2A = MergeJV2RS(nDataCols=nEventCols) # Init the Merging class
		self.fixedPosRS_last = []
		self.fixedPosRS = []
				
		#Sensor fusion
		if(odom_selector == 3):
			self.run_sensor_fusion()
		
		
		
	#Odometry callback
	def odometryCb(self,msg):

	  
	    self.real_sense_odom_enable = True
	    global twist,covariance
	    
	    br = TransformBroadcaster() 

	    translation = msg.pose.pose.position
	    rotation = msg.pose.pose.orientation
	    covariance=msg.pose.covariance
	    twist=msg.twist
 	  
	    mat = []
	    try:
	    	a = 1
	    	
	    	br.sendTransform((translation.x, translation.y, translation.z), (rotation.x,rotation.y,rotation.z,rotation.w), rospy.Time.now(), 't_265','odom_origin')
	    	
	    	
	    	
	    	self.trans_base_link()
	    	
	    except:
	    	print "Exception in sending transform"
	    	pass



	#Just a callback of jeovis messages
	def jeovisCb(self,msg):
		global first_aruco
		pose = msg.pose
		
                first_aruco=first_aruco+1
		if first_aruco==2:
			static_transform_world_base_link_origin(msg)
			first_aruco=first_aruco+1
		self.trans_jevois = pose.translation
		self.rot_jevois = pose.rotation
			
		
		self.process_jeovis()
		
	#Converting jeovis message data to mavros topic 	
	def process_jeovis(self):
            global marker,marker_array,pub_mark,pub_jevois,jv_filter_flag,pub_jv_yaw,rviz_debugging_flag
	    jv_msg=PoseWithCovarianceStamped()
            self.jevois_msg.header.stamp = rospy.Time.now()
            self.jevois_msg.header.frame_id = "base_link"

            self.jevois_msg.pose.position.x = self.trans_jevois.x
            self.jevois_msg.pose.position.y = self.trans_jevois.y
            self.jevois_msg.pose.position.z = self.trans_jevois.z

            self.jevois_msg.pose.orientation.x = self.rot_jevois.x
            self.jevois_msg.pose.orientation.y = self.rot_jevois.y
            self.jevois_msg.pose.orientation.z = self.rot_jevois.z
            self.jevois_msg.pose.orientation.w = self.rot_jevois.w
	    X,Y,Z=quaternion_to_euler(self.rot_jevois.x, self.rot_jevois.y, self.rot_jevois.z, self.rot_jevois.w)
       	    

            jv_msg.pose.pose.orientation.x = self.rot_jevois.x
            jv_msg.pose.pose.orientation.y = self.rot_jevois.y
            jv_msg.pose.pose.orientation.z = self.rot_jevois.z
            jv_msg.pose.pose.orientation.w = self.rot_jevois.w
            marker.pose.orientation.x= self.rot_jevois.x
            marker.pose.orientation.y= self.rot_jevois.y
            marker.pose.orientation.z= self.rot_jevois.z
            marker.pose.orientation.w= self.rot_jevois.w

 	    #print X,Y,Z
            jv_msg.header.stamp = rospy.Time.now()
            jv_msg.header.frame_id = "world" 
            jv_msg.pose.pose.position.x = self.trans_jevois.x
            jv_msg.pose.pose.position.y = self.trans_jevois.y
            jv_msg.pose.pose.position.z = self.trans_jevois.z

            jv_msg.pose.covariance=[1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1]
            #jv_msg.pose.covariance=[10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10]
            #jv_msg.pose.covariance=[0.0033, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00086, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0033, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0035, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00044, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.87]	
            #If robot_localization fusion is stored in global variable calculate Euclidean distance from jevois reading
            yaw_msg=Float32()
            yaw_msg.data=Z

            print Z
	    if jv_filter_flag==True:
            	if((Z>-0.2 and Z<0.2) or (abs(Z)>3.05 and abs(Z)<3.2)):
            			pub_jevois.publish(jv_msg)
                        	pub_jv_yaw.publish(yaw_msg)
            	else:
            		print "Outlier Yaw"
            else:
            	pub_jevois.publish(jv_msg)
                pub_jv_yaw.publish(yaw_msg)


            if rviz_debugging_flag==True:		
    	    	marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.id=marker.id+1
                marker.pose.position.x = self.trans_jevois.x
                marker.pose.position.y = self.trans_jevois.y
                marker.pose.position.z = self.trans_jevois.z

                
                marker_array.markers.append(marker)
                if jv_filter_flag==True:
                	if((Z>-0.2 and Z<0.2) or (abs(Z)>3.05 and abs(Z)<3.2)):
                			pub_mark.publish(marker_array)
                else:
                	pub_mark.publish(marker_array)
            	
            self.jevois_odom_enable = True
            if( self.odom_selector == 2):
                self.pub.publish(self.jevois_msg)
                self.rate.sleep()

			
		#except:
		#	pass
			
			
	def send_sensor_fusion_data(self):
	
		
	
		try:

			self.sensor_fused.header.stamp = rospy.Time.now()
			self.sensor_fused.header.frame_id = "base_link"
		

			self.sensor_fused.pose.position.x = self.fixedPosRS_last[1]
			self.sensor_fused.pose.position.y = self.fixedPosRS_last[2]
			self.sensor_fused.pose.position.z = self.fixedPosRS_last[3]
			
			#print((self.fixedPosRS_last[1]),(self.fixedPosRS_last[2]),(self.fixedPosRS_last[3]),(self.fixedPosRS_last[4]), (self.fixedPosRS_last[5]),(self.fixedPosRS_last[6]))		
			quaternion = tf.transformations.quaternion_from_euler((self.fixedPosRS_last[4]), 
									      (self.fixedPosRS_last[5]), 
									      (self.fixedPosRS_last[6]))

			

			self.sensor_fused.pose.orientation.x = quaternion[0]
			self.sensor_fused.pose.orientation.y = quaternion[1]
			self.sensor_fused.pose.orientation.z = quaternion[2]
			self.sensor_fused.pose.orientation.w = quaternion[3]
			global marker_fuse,marker_array_fuse,pub_fuse,count,rviz_debugging_flag

            		if rviz_debugging_flag==True:
                            marker_fuse.color.r = 0.0
                            marker_fuse.color.g = 1.0
                            marker_fuse.color.b = 0.0
                            marker_fuse.id=marker_fuse.id+1
                            marker_fuse.pose.position.x = self.fixedPosRS_last[1]
                            marker_fuse.pose.position.y = self.fixedPosRS_last[2]
                            marker_fuse.pose.position.z = self.fixedPosRS_last[3]

                            marker_fuse.pose.orientation.x= quaternion[0]
                            marker_fuse.pose.orientation.y= quaternion[1]
                            marker_fuse.pose.orientation.z= quaternion[2]
                            marker_fuse.pose.orientation.w= quaternion[3]
                       
    			
    			    marker_array_fuse.markers.append(marker_fuse)
                            pub_fuse.publish(marker_array_fuse)
			#count=0

			if( self.odom_selector == 3):
				self.pub.publish(self.sensor_fused)	
				self.rate.sleep()

			
		except:
			pass			
			
	
		
	#Transform from base_link_origin to base_liknk	
	def trans_base_link(self):
		trans = []
    		rot = []
	
		try:
			(trans,rot) = self.listener.lookupTransform('/base_link_origin', '/base_link', rospy.Time(0))
            		(trans_world,rot_world) = self.listener.lookupTransform('/world', '/base_link', rospy.Time(0))
	
		
			self.real_sense_odom_enable = True

			self.r.header.stamp = rospy.Time.now()
			self.r.header.frame_id = "base_link"
		
			self.r.pose.position.x = trans[0]
			self.r.pose.position.y = trans[1]
			self.r.pose.position.z = trans[2]

			self.r.pose.orientation.x = rot[0]
			self.r.pose.orientation.y = rot[1]
			self.r.pose.orientation.z = rot[2]
			self.r.pose.orientation.w = rot[3]
            		X,Y,Z=quaternion_to_euler(rot_world[0], rot_world[1], rot_world[2], rot_world[3])
			global marker_rs,marker_array_rs,pub_rs,count,pub_odometry,twist,covariance,pub_rs_yaw,rviz_debugging_flag,resonance_count
            
                        odom=Odometry()
			
			odom.header.stamp = rospy.Time.now()
            		odom.header.frame_id = "base_link_origin"
			odom.pose.pose.position.x = trans[0]
           		odom.pose.pose.position.y = trans[1]
            		odom.pose.pose.position.z = trans[2]

            		odom.pose.pose.orientation.x = rot[0]
            		odom.pose.pose.orientation.y = rot[1]
            		odom.pose.pose.orientation.z = rot[2]
            		odom.pose.pose.orientation.w = rot[3]
            		odom.pose.covariance=[1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1]
            		#odom.pose.covariance=[10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10]
            		#odom.pose.covariance=covariance
            		odom.twist=twist
            		#odom.twist.covariance=covariance
            		odom.twist.covariance=[1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1]
            		#odom.twist.covariance=[10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10]
            		#print "Tracker Odom callback"
                    	if ((abs(odom.twist.twist.linear.x)>0.8 or abs(odom.twist.twist.linear.y)>0.8 or abs(odom.twist.twist.linear.z)>0.8) and resonance_count==0):
                		print "Resonance Detected in t265 performance"
                        	print "*****************************************************************************************"
                        	print "**************************************************************************************"
                        	resonance_count=resonance_count+1
                            	# Restart the realsense sensor
                            	rospy.wait_for_service('/camera/realsense2_camera/reset_camera')
                            	try:
                                	reset_t265 = rospy.ServiceProxy('/camera/realsense2_camera/reset_camera', ResetCamera)
                                	resp1 = reset_t265()
                                	return resp1.reset_feedback
                            	except rospy.ServiceException, e:
                                	print "Service call failed: %s"%e

                        

			else:
                		pub_odometry.publish(odom)

                        	yaw_msg=Float32()
                        	yaw_msg.data=Z
                        	pub_rs_yaw.publish(yaw_msg)

            			if rviz_debugging_flag==True:
                			marker_rs.color.r = 1.0
                            		marker_rs.color.g = 1.0
                            		marker_rs.color.b = 1.0
                            		marker_rs.id=marker_rs.id+1
                            		marker_rs.pose.position.x = trans[0]
                            		marker_rs.pose.position.y = trans[1]
                            		marker_rs.pose.position.z = trans[2]

                            		marker_rs.pose.orientation.x= rot[0]
                            		marker_rs.pose.orientation.y= rot[1]
                            		marker_rs.pose.orientation.z= rot[2]
                            		marker_rs.pose.orientation.w= rot[3]
                			count=count+1
                			if(count==20):
                				count=0
                				marker_array_rs.markers.append(marker_rs)
                            			pub_rs.publish(marker_array_rs)


            

		    
		    	if(self.odom_selector == 1):  #If the option is sensor fusion, dont publish the topic
				self.pub.publish(self.r)	
		  

		    	
            	
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

			pass
			
	def time_stemp(self,stamp, sec2nsec=1e-9):
    	# Calculate time in seconds
    		return stamp.secs + stamp.nsecs*sec2nsec
	
			
	def sensor_fusion(self):
		#rospy.loginfo("In sensor fusion")
		self.fixedPosRS_last = []

		if(self.real_sense_odom_enable):
		
			#try:
		
				time_stmp = self.time_stemp(self.r.header.stamp, sec2nsec=1e-9)
			
				self.fixedPosRS_last.append(time_stmp)
				self.fixedPosRS_last.append(self.r.pose.position.x)
				self.fixedPosRS_last.append(self.r.pose.position.y)
				self.fixedPosRS_last.append(self.r.pose.position.z)
			
			
			
				quaternion = (self.r.pose.orientation.x,
	    				      self.r.pose.orientation.y,
	    				      self.r.pose.orientation.z,
	    				      self.r.pose.orientation.w)
			
				euler = tf.transformations.euler_from_quaternion(quaternion)

				self.fixedPosRS_last.append(euler[0])
				self.fixedPosRS_last.append(euler[1])
				self.fixedPosRS_last.append(euler[2])


				#src append 0 for real sense input
				self.fixedPosRS_last.append(srsDict['RealSense'])
			
							
				
				self.MergeV2A.process(np.array(self.fixedPosRS_last), self.r.pose.orientation)
				self.fixedPosRS_last = self.MergeV2A.fixedRS
				
				if not self.fixedPosRS_last is None:
			    		self.fixedPosRS.append(self.fixedPosRS_last) 
			    		
						

		if(self.jevois_odom_enable):
			self.fixedPosRS_last = []
			try:
		
				time_stmp = self.time_stemp(self.jevois_msg.header.stamp, sec2nsec=1e-9)
			
				self.fixedPosRS_last.append(time_stmp)
				self.fixedPosRS_last.append(self.jevois_msg.pose.position.x)
				self.fixedPosRS_last.append(self.jevois_msg.pose.position.y)
				self.fixedPosRS_last.append(self.jevois_msg.pose.position.z)
			
			
			
				quaternion = (self.jevois_msg.pose.orientation.x,
	    				      self.jevois_msg.pose.orientation.y,
	    				      self.jevois_msg.pose.orientation.z,
	    				      self.jevois_msg.pose.orientation.w)
			
				euler = tf.transformations.euler_from_quaternion(quaternion)

				self.fixedPosRS_last.append(euler[0])
				self.fixedPosRS_last.append(euler[1])
				self.fixedPosRS_last.append(euler[2])

				#src append for jevois
				self.fixedPosRS_last.append(srsDict['Jevois'])
			
				
				self.MergeV2A.process(np.array(self.fixedPosRS_last), self.r.pose.orientation)
				self.fixedPosRS_last = self.MergeV2A.fixedRS
				
				if not self.fixedPosRS_last is None:
			    		self.fixedPosRS.append(self.fixedPosRS_last) 
			    		

				self.jevois_odom_enable = False    
				

								
				
			except:
				print("Exception in jeovis")
				pass			
		

	
		#Send to mavros topic
		self.send_sensor_fusion_data()	
		
	#Sensor fusion main loop	

	def run_sensor_fusion(self):
		while not rospy.is_shutdown():
			self.sensor_fusion()

            	

if __name__ == '__main__':
    
    	odom_source = {'RealSense_transform':1, 'Jevois':2,'Fusion':3}
    
    
    	try:
		odom_selector = odom_source[str(sys.argv[1])]

	except:
		#default to real sense transformed
		odom_selector = 1		


			
	print odom_selector
	obj = Drone_Odometry(odom_selector)
	
	rospy.spin()
	


	
