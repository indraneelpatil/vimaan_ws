#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import tf
from tf import TransformListener, TransformerROS,transformations, TransformBroadcaster
from scipy import signal
import tf2_ros

counter=0
sequence_counter=0
last_estimate=[0,0,0,0,0,0,0]
current_estimate=[0,0,0,0,0,0,0]
estimate=[0,0,0]
dist=100

x_buffer=0
y_buffer=0
z_buffer=0
yaw_buffer=0

save_counter=0

yaw_array=[]

pub=rospy.Publisher('/odometry/smoothened',Odometry,queue_size=10)
pub2=rospy.Publisher('/smooth_yaw',Float32,queue_size=10)
#pub2=rospy.Publisher('/mavros/vision_pose/pose',PoseStamped,queue_size=10)



def callback(data):
    global pub,counter,current_estimate,last_estimate,dist,sequence_counter,x_buffer,y_buffer,z_buffer,estimate,yaw_buffer,pub2,save_counter,yaw_array
    current_estimate[0]=data.pose.pose.position.x
    current_estimate[1]=data.pose.pose.position.y
    current_estimate[2]=data.pose.pose.position.z
    current_estimate[3]=data.pose.pose.orientation.x
    current_estimate[4]=data.pose.pose.orientation.y
    current_estimate[5]=data.pose.pose.orientation.z
    current_estimate[6]=data.pose.pose.orientation.w
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
        ## Check for Jumps in Yaw
        quat_current=[current_estimate[3],current_estimate[4],current_estimate[5],current_estimate[6]]
        quat_last=[last_estimate[3],last_estimate[4],last_estimate[5],last_estimate[6]]
        #convert tuples to list
        euler_current = tf.transformations.euler_from_quaternion(quat_current)
        euler_current_list=[euler_current[0],euler_current[1],euler_current[2]]
        euler_last = tf.transformations.euler_from_quaternion(quat_last)
        euler_last_list=[euler_last[0],euler_last[1],euler_last[2]]
        
        yaw_array.append(euler_current_list[2])
        if abs(euler_current_list[2])<2.8:
            
            

            # Try the low pass filter or return as it is
            try:
                nyquist = 25 # 0.5 times the sampling frequency
                cutoff=0.01 # fraction of nyquist frequency, here  it is 5 days
                b, a = signal.butter(5, cutoff, btype='lowpass') #low pass filter

                dUfilt = signal.filtfilt(b, a, yaw_array)
                dUfilt=np.array(dUfilt)
                dUfilt=dUfilt.transpose()
                #print dUfilt.size
                euler_current_list[2]=dUfilt[dUfilt.size-1]
                ###################################################update current_estimate
                quat_update = tf.transformations.quaternion_from_euler(euler_current_list[0], euler_current_list[1], euler_current_list[2])
                current_estimate[3]=quat_update[0]
                current_estimate[4]=quat_update[1]
                current_estimate[5]=quat_update[2]
                current_estimate[6]=quat_update[3]

                data.pose.pose.orientation.x=quat_update[0]
                data.pose.pose.orientation.y=quat_update[1]
                data.pose.pose.orientation.z=quat_update[2]
                data.pose.pose.orientation.w=quat_update[3]


            except Exception as e:
                        rospy.loginfo(str(e))


            msg=Float32()
            msg.data=euler_current_list[2]
            pub2.publish(msg)
        else:
        
            if (abs((euler_current_list[2])-(euler_last_list[2]))>0.04 and abs((euler_current_list[2])-(euler_last_list[2]))<3.14):
                print "Big Jump in Yaw found",euler_current_list[2],euler_last_list[2],(euler_current_list[2]-euler_last_list[2])
                print "Before",yaw_buffer
                yaw_buffer=yaw_buffer+(euler_current_list[2])-(euler_last_list[2])
                print "After",yaw_buffer
                print " "

                euler_current_list[2]= euler_current_list[2]-yaw_buffer
                if euler_current_list[2]>3.14:
                    euler_current_list[2]=euler_current_list[2]-6.28
                elif euler_current_list[2]<-3.14:
                    euler_current_list[2]=euler_current_list[2]+6.28
                else:
                    euler_current_list[2]=euler_current_list[2]
                # Visualise Yaw Angle
                msg=Float32()
                msg.data=euler_current_list[2]
                pub2.publish(msg)
                quat_new = tf.transformations.quaternion_from_euler(euler_current_list[0], euler_current_list[1], euler_current_list[2])
                
                data.pose.pose.orientation.x=quat_new[0]
                data.pose.pose.orientation.y=quat_new[1]
                data.pose.pose.orientation.z=quat_new[2]
                data.pose.pose.orientation.w=quat_new[3]
            
            elif (abs((euler_current_list[2])-(euler_last_list[2]))>3.14):
                print "***Reverse Big Jump in Yaw found",euler_current_list[2],euler_last_list[2],(6.28-abs(euler_current_list[2]-euler_last_list[2]))
                print "Before",yaw_buffer
                yaw_buffer=yaw_buffer-(6.28-abs(euler_current_list[2]-euler_last_list[2]))
            	print "After",yaw_buffer
                print " "

                euler_current_list[2]= euler_current_list[2]-yaw_buffer
                if euler_current_list[2]>3.14:
                    euler_current_list[2]=euler_current_list[2]-6.28
                elif euler_current_list[2]<-3.14:
                    euler_current_list[2]=euler_current_list[2]+6.28
                else:
                    euler_current_list[2]=euler_current_list[2]
                # Visualise Yaw Angle
                msg=Float32()
                msg.data=euler_current_list[2]
                #msg.data=test_low[0]
                pub2.publish(msg)

                quat_new = tf.transformations.quaternion_from_euler(euler_current_list[0], euler_current_list[1], euler_current_list[2])
                
                data.pose.pose.orientation.x=quat_new[0]
                data.pose.pose.orientation.y=quat_new[1]
                data.pose.pose.orientation.z=quat_new[2]
                data.pose.pose.orientation.w=quat_new[3]

            else :
                yaw_buffer=yaw_buffer*(1-0.005)
                euler_current_list[2]= euler_current_list[2]-yaw_buffer
                if euler_current_list[2]>3.14:
                    euler_current_list[2]=euler_current_list[2]-6.28
                elif euler_current_list[2]<-3.14:
                    euler_current_list[2]=euler_current_list[2]+6.28
                else:
                    euler_current_list[2]=euler_current_list[2]
                # Visualise Yaw Angle
                msg=Float32()
                msg.data=euler_current_list[2]
                pub2.publish(msg)

                quat_new = tf.transformations.quaternion_from_euler(euler_current_list[0], euler_current_list[1], euler_current_list[2])
                
                data.pose.pose.orientation.x=quat_new[0]
                data.pose.pose.orientation.y=quat_new[1]
                data.pose.pose.orientation.z=quat_new[2]
                data.pose.pose.orientation.w=quat_new[3]


        


        #print "Hello"
        a = np.array((current_estimate[0],current_estimate[1],current_estimate[2]))
        b = np.array((last_estimate[0],last_estimate[1],last_estimate[2]))
        dist = np.linalg.norm(a-b)
        #print dist
        if dist>0.05 and dist<2:
            print "Big Jump in Position Estimate Found",current_estimate[0],last_estimate[0]
            #Add the jump to the buffer
            x_buffer=x_buffer+(current_estimate[0]-last_estimate[0])
            y_buffer=y_buffer+(current_estimate[1]-last_estimate[1])
            z_buffer=z_buffer+(current_estimate[2]-last_estimate[2])

            #Subtract the buffer from current estimate and sent it to ROS
            data.pose.pose.position.x=current_estimate[0]-x_buffer
            data.pose.pose.position.y=current_estimate[1]-y_buffer
            data.pose.pose.position.z=current_estimate[2]-z_buffer


            sequence_counter=sequence_counter+1
            data.header.stamp=rospy.Time.now()
            data.header.seq=sequence_counter
            pub.publish(data)

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




    last_estimate[0]=current_estimate[0]
    last_estimate[1]=current_estimate[1]
    last_estimate[2]=current_estimate[2]
    last_estimate[3]=current_estimate[3]
    last_estimate[4]=current_estimate[4]
    last_estimate[5]=current_estimate[5]
    last_estimate[6]=current_estimate[6]
    #print current_estimate[0]
    #print last_estimate[0]


    
def listener():

    
    rospy.init_node('smoothen_fusion', anonymous=True)

    rospy.Subscriber("/odometry/filtered", Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

