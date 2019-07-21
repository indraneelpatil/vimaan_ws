#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import math
import tf
from tf import TransformListener, TransformerROS,transformations, TransformBroadcaster
import tf2_ros

counter=0
counter_jv=0
sequence_counter=0
last_estimate=[0,0,0]
current_estimate=[0,0,0]
estimate=[0,0,0]
dist=100

jv_current=[0,0,0]
jv_last=[0,0,0]

x_buffer=0
y_buffer=0
z_buffer=0

yaw_buffer=0   # in radians
initial_transform=0
rot=[0,0,0,0]
euler_list=[0,0,0]
trans=[0,0,0]

pub=rospy.Publisher('/odometry/smoothened',Odometry,queue_size=10)
#pub2=rospy.Publisher('/mavros/vision_pose/pose',PoseStamped,queue_size=10)



def callback(data):
    global pub,counter,current_estimate,last_estimate,dist,sequence_counter,x_buffer,y_buffer,z_buffer,estimate,yaw_buffer,initial_transform,rot,euler_list,trans
    current_estimate[0]=data.pose.pose.position.x
    current_estimate[1]=data.pose.pose.position.y
    current_estimate[2]=data.pose.pose.position.z
    #print last_estimate[0]
    if counter==0:
    	#Publish as it is for the first callback function
    	#print "HI"
    	sequence_counter=sequence_counter+1
    	data.header.seq=sequence_counter
    	data.header.stamp=rospy.Time.now()
    	pub.publish(data)
    	counter=counter+1
    	#last_estimate=current_estimate
    else:
    	#print "Hello"
    	a = np.array((current_estimate[0],current_estimate[1],current_estimate[2]))
    	b = np.array((last_estimate[0],last_estimate[1],last_estimate[2]))
    	dist = np.linalg.norm(a-b)
    	#print dist
    	if dist>0.15 and dist<2:
    	    print "Big Jump in Position Estimate Found",current_estimate[0],last_estimate[0]
            #Add the jump to the buffer
            x_buffer=x_buffer+(current_estimate[0]-last_estimate[0])
            y_buffer=y_buffer+(current_estimate[1]-last_estimate[1])
            z_buffer=z_buffer+(current_estimate[2]-last_estimate[2])

            #Subtract the buffer from current estimate and send it to ROS
            data.pose.pose.position.x=current_estimate[0]-x_buffer
            data.pose.pose.position.y=current_estimate[1]-y_buffer
            data.pose.pose.position.z=current_estimate[2]-z_buffer

            sequence_counter=sequence_counter+1
            data.header.stamp=rospy.Time.now()
            data.header.seq=sequence_counter
            pub.publish(data)

            #print jv_current[0],jv_current[1],jv_current[2]
            #print jv_last[0],jv_last[1],jv_last[2]

            ##################### Calculate Yaw offset since last Jevois Reading in the X-Y plane
            ########### Only calculate offset my jevois readings are atleast 1 apart
            c=np.array((jv_current[0],jv_current[1]))
            d=np.array((jv_last[0],jv_last[1]))
            e=np.array((last_estimate[0],last_estimate[1]))
            if np.linalg.norm(c-d)>1:
                

                c_theta=np.dot(e-d,c-d)/(np.linalg.norm(e-d)*np.linalg.norm(c-d))
                yaw_offset=(math.acos(c_theta))   #math.degrees for degrees
                print "Offset",math.degrees(yaw_offset)
                ##############################Look up current existing transform

                listener = tf.TransformListener()
                try:
                    if initial_transform==0:
                        (trans,rot) = listener.lookupTransform('world', 'base_link_origin', rospy.Time(0))
                        print "Initial static transform found"
                        initial_transform=initial_transform+1
                        #print trans[0],trans[1],trans[2]
                        #print rot[0],rot[1],rot[2],rot[3]
                        euler= tf.transformations.euler_from_quaternion(rot)
                        euler_list=[euler[0],euler[1],euler[2]]
                    print "Old Yaw",math.degrees(euler_list[2])
                    ############################## Change euler angle based on position of estimate before jump
                    if last_estimate[1]<jv_current[1]:
                        euler_list[2]=euler_list[2]+yaw_offset
                        print "New Yaw",math.degrees(euler_list[2])
                        quaternion=tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1],euler_list[2])
                        yaw_buffer=yaw_buffer+yaw_offset
                    else:
                        euler_list[2]=euler_list[2]-yaw_offset
                        print "New Yaw",math.degrees(euler_list[2])
                        quaternion=tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1], euler_list[2])
                        yaw_buffer=yaw_buffer-yaw_offset

                    ########################## Republish new static transform   
                    print "Static Transform from world to base_link_origin modified"

                    #broadcaster = tf2_ros.StaticTransformBroadcaster()
                    #static_transformStamped = TransformStamped()

                    #static_transformStamped.header.stamp = rospy.Time.now()
                    #static_transformStamped.header.frame_id = "world"
                    #static_transformStamped.child_frame_id = "base_link_origin"

                    #static_transformStamped.transform.translation.x = trans[0]
                    #static_transformStamped.transform.translation.y = trans[1]
                    #static_transformStamped.transform.translation.z = trans[2]

                    #static_transformStamped.transform.rotation.x = quaternion[0]
                    #static_transformStamped.transform.rotation.y = quaternion[1]
                    #static_transformStamped.transform.rotation.z = quaternion[2]
                    #static_transformStamped.transform.rotation.w = quaternion[3]

                    #broadcaster.sendTransform(static_transformStamped)
                except Exception as e:
                    rospy.loginfo(str(e))
                
                
            
    	else:
            #Slowly release the buffer
            x_buffer=x_buffer*(1-0.005)
            y_buffer=y_buffer*(1-0.005)
            z_buffer=z_buffer*(1-0.005)

            data.pose.pose.position.x=current_estimate[0]-x_buffer
            data.pose.pose.position.y=current_estimate[1]-y_buffer
            data.pose.pose.position.z=current_estimate[2]-z_buffer

	    sequence_counter=sequence_counter+1
            data.header.seq=sequence_counter
    	    data.header.stamp=rospy.Time.now()
    	    pub.publish(data)

            yaw_buffer=yaw_buffer*(1-0.005)
            quaternion=tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1], euler_list[2]-yaw_buffer)
            
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = TransformStamped()

            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "world"
            static_transformStamped.child_frame_id = "base_link_origin"

            static_transformStamped.transform.translation.x = trans[0]
            static_transformStamped.transform.translation.y = trans[1]
            static_transformStamped.transform.translation.z = trans[2]

            static_transformStamped.transform.rotation.x = quaternion[0]
            static_transformStamped.transform.rotation.y = quaternion[1]
            static_transformStamped.transform.rotation.z = quaternion[2]
            static_transformStamped.transform.rotation.w = quaternion[3]

            broadcaster.sendTransform(static_transformStamped)





    last_estimate[0]=current_estimate[0]
    last_estimate[1]=current_estimate[1]
    last_estimate[2]=current_estimate[2]
    #print current_estimate[0]
    #print last_estimate[0]

def jv_callback(data):
    global counter_jv,jv_current,jv_last
    
    if counter_jv==0:
        jv_current[0]=data.pose.pose.position.x
        jv_current[1]=data.pose.pose.position.y
        jv_current[2]=data.pose.pose.position.z
        counter_jv=counter_jv+1
    else:
        jv_last[0]=jv_current[0]
        jv_last[1]=jv_current[1]
        jv_last[2]=jv_current[2]
        jv_current[0]=data.pose.pose.position.x
        jv_current[1]=data.pose.pose.position.y
        jv_current[2]=data.pose.pose.position.z


    
def listener():

    
    rospy.init_node('smoothen_fusion', anonymous=True)

    rospy.Subscriber("/odometry/filtered", Odometry, callback)
    rospy.Subscriber("jv_pose_new",PoseWithCovarianceStamped,jv_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
