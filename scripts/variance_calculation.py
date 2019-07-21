#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math

pos_x=[]
pos_y=[]
pos_z=[]
angle_x=[]
angle_y=[]
angle_z=[]
num_readings=0
first_message=0
last_message_time=0
current_message_time=0

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

def calculate_variances():
	global pos_x,pos_y,pos_z,angle_z,angle_y,angle_x
	print len(pos_x)
	print "Variance in X is :",np.var(pos_x)
	print "Variance in Y is :",np.var(pos_y)
	print "Variance in Z is :",np.var(pos_z)
	print "Variance in Yaw is :",np.var(angle_x)
	print "Variance in Pitch is :",np.var(angle_y)
	print "Variance in Roll is :",np.var(angle_z)





def callback(data):
	global pos_x,pos_y,pos_z,angle_z,angle_y,angle_x,num_readings,first_message,last_message_time,current_message_time
	if first_message==0:
		last_message_time=rospy.Time.now()
		first_message=first_message+1
	num_readings=num_readings+1
	print num_readings
	current_message_time=rospy.Time.now()
	if float(current_message_time.secs)-float(last_message_time.secs)>5:
		calculate_variances()
		print "Number of readings taken: ",num_readings
	X,Y,Z=quaternion_to_euler(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
	angle_x.append(X)
	angle_y.append(Y)
	angle_z.append(Z)
	pos_x.append(data.pose.pose.position.x)
	pos_y.append(data.pose.pose.position.y)
	pos_z.append(data.pose.pose.position.z)
	
	#print (float(current_message_time.secs)-float(last_message_time.secs))
	

	last_message_time=current_message_time

    
    
def listener():

    
    rospy.init_node('variance_calculator', anonymous=True)

    rospy.Subscriber("jv_pose_new", PoseWithCovarianceStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()