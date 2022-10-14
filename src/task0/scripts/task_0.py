#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			1254
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_0.py
# Functions:
# 					[ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node


####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist     # to use 'geometry_msgs/Twist' message type from '/cmd_vel' topic
from turtlesim.msg import Pose

import math
##############################################################

###################### Edits #########################
class Controller():
    def __init__(self):
        # Initializing node to communicating with the ROS Master 
        rospy.init_node('node_turtle_revolve')
        
        # declaring that node is publishing to the 'cmd_vel' topic using the message type 'geometry_msgs.Twist'
        self.pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.rate = rospy.Rate(10) 

        self.velocity_msg = Twist()
        self.velocity_msg.linear.x = 0
        self.velocity_msg.angular.z = 0

        self.angle_in_rad = 0

        self.speed = 1
        self.angular_speed = 1

        self.flag = True

        # self.pub_vel.publish(self.velocity_msg)
    
    def pose_callback(self, msg):

        self.angle_in_rad = round(msg.theta,2)

    def distance(self,v,w,angle): return abs((angle)*v/w)


    def follow_circle(self,angle):
        # speed = 1   
        self.velocity_msg.linear.x = self.speed
        self.velocity_msg.angular.z = self.angular_speed
        self.pub_vel.publish(self.velocity_msg)
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        while(current_distance <= self.distance(self.velocity_msg.linear.x,self.velocity_msg.angular.z,angle) + 0.2):
            self.pub_vel.publish(self.velocity_msg)
            self.rate.sleep()   
            t1 = rospy.Time.now().to_sec()
            current_distance = self.speed*(t1-t0)
            # rospy.loginfo("Moving in Circle")
            print("My turtleBot is: Moving in circle!!")
            print(self.angle_in_rad)
            self.velocity_msg.linear.x = self.speed
            self.velocity_msg.angular.z = self.angular_speed
            self.pub_vel.publish(self.velocity_msg)
    
    def rotate(self,sudo_angle):
        t0 = rospy.Time.now().to_sec()
        
        ori_rad = 0
        while(ori_rad <= (math.degrees(sudo_angle))):
            self.rate.sleep()  
            t1 = rospy.Time.now().to_sec()
            print("My turtleBot is: Rotating!")
            print(self.angle_in_rad)
            ori_rad = (180)*((self.angular_speed)*(t1-t0))/(math.pi)
            self.velocity_msg.angular.z = self.angular_speed
            self.pub_vel.publish(self.velocity_msg)
        
        
    def go_straight(self,no_of_times_radius):
        self.speed = 1   
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        while(current_distance <= (no_of_times_radius*(2/2))):
            self.rate.sleep()  
            self.pub_vel.publish(self.velocity_msg)
            t1 = rospy.Time.now().to_sec()
            print("My turtleBot is: Moving Straight!!!")
            print(self.angle_in_rad)
            current_distance = self.speed*(t1-t0)
            
            self.velocity_msg.linear.x = self.speed
            self.velocity_msg.angular.z = 0
            self.pub_vel.publish(self.velocity_msg)

    def reset(self):
        self.velocity_msg.linear.x = 0
        self.velocity_msg.angular.z = 0
        self.pub_vel.publish(self.velocity_msg)


###################### Edits #########################

def callback(data):
	"""
	Purpose:
	---
	This function should be used as a callback. Refer Example #1: Pub-Sub with Custom Message in the Learning Resources Section of the Learning Resources.
    You can write your logic here.
    NOTE: Radius value should be 1. Refer expected output in document and make sure that the turtle traces "same" path.

	Input Arguments:
	---
        `data`  : []
            data received by the call back function

	Returns:
	---
        May vary depending on your logic.

	Example call:
	---
        Depends on the usage of the function.
	"""


def main():

    """
	Purpose:
	---
	This function will be called by the default main function given below.
    You can write your logic here.

	Input Arguments:
	---
        None

	Returns:
	---
        None

	Example call:
	---
        main()
	"""


    controller = Controller()
    try:
        controller = Controller()
        while not rospy.is_shutdown():
            if(controller.flag):


                controller.rate.sleep()       # putting some delay so subscribers work perfectely from start
                controller.follow_circle((math.pi))
                controller.reset()
                controller.rotate((math.pi/2))
                controller.reset()
                controller.go_straight(2)
                controller.reset()
                print("Done!")
                controller.flag = False
            # self.pub_vel.publish(self.velocity_msg)
    except rospy.ROSInterruptException as e:
        print(e)

	

    



################# ADD GLOBAL VARIABLES HERE #################



##############################################################


################# ADD UTILITY FUNCTIONS HERE #################



##############################################################


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")

