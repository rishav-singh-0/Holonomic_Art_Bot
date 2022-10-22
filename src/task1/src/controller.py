!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type: Twist

from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

hola_x = 0
hola_y = 0
hola_theta = 0

def odometryCb(msg):

    global hola_x, hola_y, hola_theta
    # Write your code to take the msg and update the three variables
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w

    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y

    

    hola_theta = euler_from_quaternion([x,y,z,w])[2]


	

def main():

    # Initialze Node
    rospy.init_node('controller')
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
	
	# Initialze Publisher and Subscriber

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/odom', Odometry, odometryCb)
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively

    # Declare a Twist message
    vel = Twist()
    
    # Initialise the required variables to 0
	# <This is explained below>
	
    rate = rospy.Rate(100)



	# For maintaining control loop rate.
	

	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller

	#
	# 
	# Control Loop goes here
	#
	#
    while not rospy.is_shutdown():

        vel.linear.x = 0 #vel_x
        vel.linear.y = 0 #vel_y
        vel.angular.z = 0 #vel_z
        print(math.degrees(hola_theta), hola_y, hola_x)

        pub.publish(vel)
        rate.sleep()

	

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
