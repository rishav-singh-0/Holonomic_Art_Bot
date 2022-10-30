#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

class Controller():
	PI = 3.14

	def __init__(self):
		################## GLOBAL VARIABLES ######################

		self.x_goals = []
		self.y_goals = []
		self.theta_goals = []

		# force vectors initialization
		self.right_wheel = Wrench()
		self.left_wheel = Wrench()
		self.front_wheel = Wrench()

		# position as [x, y, theta]
		self.hola_position = [None, None, None]
		self.goal_position = [None, None, None]

		#################### ROS Node ############################

		rospy.init_node('controller_node')

		signal.signal(signal.SIGINT, self.signal_handler)

		self.right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
		self.front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
		self.left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

		rospy.Subscriber('detected_aruco',Pose2D,self.aruco_feedback_Cb)
		rospy.Subscriber('task2_goals',PoseArray,self.task2_goals_Cb)
		
		self.rate = rospy.Rate(100)

	##################### FUNCTION DEFINITIONS #######################

	def signal_handler(self, sig, frame):
		
		# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
		print('Clean-up !')
		self.cleanup()
		sys.exit(0)

	def cleanup(self):
		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 
		#	-> Not mandatory - but it is recommended to do some cleanup over here,
		#	   to make sure that your logic and the robot model behaves predictably in the next run.

		############################################
		pass
	
	def task2_goals_Cb(self, msg):
		self.x_goals.clear()
		self.y_goals.clear()
		self.theta_goals.clear()

		for waypoint_pose in msg.poses:
			self.x_goals.append(waypoint_pose.position.x)
			self.y_goals.append(waypoint_pose.position.y)

			orientation_q = waypoint_pose.orientation
			orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
			theta_goal = euler_from_quaternion (orientation_list)[2]
			self.theta_goals.append(theta_goal)

	def aruco_feedback_Cb(self, msg):
		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 
		#	-> Receive & store the feedback / coordinates found by aruco detection logic.
		#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

		############################################
		pass

	def inverse_kinematics(self):
		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 
		#	-> Use the target velocity you calculated for the robot in previous task, and
		#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
		#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
		############################################
		pass

	def main(self):

		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 
		#	-> Make use of the logic you have developed in previous task to go-to-goal.
		#	-> Extend your logic to handle the feedback that is in terms of pixels.
		#	-> Tune your controller accordingly.
		# 	-> In this task you have to further implement (Inverse Kinematics!)
		#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
		#      given velocity of the chassis (Vx, Vy, W)
		#	   
			
		while not rospy.is_shutdown():
			
			# Calculate Error from feedback

			# Change the frame by using Rotation Matrix (If you find it required)

			# Calculate the required velocity of bot for the next iteration(s)
			
			# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

			# Apply appropriate force vectors

			# Modify the condition to Switch to Next goal (given position in pixels instead of meters)

			self.rate.sleep()

		############################################

if __name__ == "__main__":
	try:
		control = Controller()
		control.main()
	except rospy.ROSInterruptException:
		pass

