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
import numpy as np

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
from std_srvs.srv import Empty			# for shutdown hook

# publishing to /cmd_vel with msg type: Twist

# from geometry_msgs.msg import Twist	



import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

class Controller():
	PI = 3.14

	def __init__(self):
		################## GLOBAL VARIABLES ######################

		# self.x_goals = [50,350,50,250,250]
		# self.y_goals = [350,50,50,350,50]
		# self.theta_goals = [1.57, -1.57, 3, -3, 0]

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

		self.error_global = [0, 0, 0]
		self.error_local = [0, 0]       # only needs [x, y]

		self.index = 0					# For travercing the setpoints

		# variables for P controller
		self.kp = [0.002, 0.05]

		# Variables for wheel force
		self.front_wheel_force = None
		self.left_wheel_force = None
		self.right_wheel_force = None

		self.w = None
		self.vel_x = None
		self.vel_y = None
		self.vel_z = None

		self.front_w = Wrench()
		self.left_w = Wrench()
		self.right_w = Wrench()

		self.prev = [0,0,0]

		self.tr_mat = np.array([[1, -math.cos(math.radians(60)), -math.cos(math.radians(60))], [-0, math.cos(math.radians(30)), -math.cos(math.radians(30))], [-1, -1, -1]])

		# self.vel = Twist()

		#################### ROS Node ############################

		rospy.init_node('controller_node')

		signal.signal(signal.SIGINT, self.signal_handler)
		self.rate = rospy.Rate(200)

		# self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
		self.front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
		self.left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

		rospy.Subscriber('detected_aruco',Pose2D,self.aruco_feedback_Cb)
		rospy.Subscriber('task2_goals',PoseArray,self.task2_goals_Cb)
		
        #ShutdownHook
		# rospy.wait_for_service('/gazebo/reset_world')
		# self.reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)

	##################### FUNCTION DEFINITIONS #######################



	def signal_handler(self, sig, frame):
		
		# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
		print('\nClean-up !')
		self.cleanup()
		sys.exit(0)

	def cleanup(self):
		force_zero = Wrench()
		force_zero.force.x = force_zero.force.y = force_zero.force.z = 0
		self.right_wheel_pub.publish(force_zero)
		self.front_wheel_pub.publish(force_zero)
		self.left_wheel_pub.publish(force_zero)
		# self.vel.linear.x = 0
		# self.vel.linear.y = 0
		# self.vel.angular.z = 0

		# print(self.vel)
		# self.pub.publish(self.vel)
		# self.reset_world()
	
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
		self.hola_position[0] = msg.x
		self.hola_position[1] = msg.y
		self.hola_position[2] = msg.theta
	
	def is_ready(self):
		condition = self.x_goals == [] or \
					self.x_goals == None or \
					self.hola_position[0] == None
		# print(self.x_goals,self.hola_position)
		return condition




	def threshold_box(self):
		condition = abs(self.error_global[0]) < 1 and abs(self.error_global[1]) < 1 and abs(math.degrees(self.error_global[2])) <= 0.5
		return condition

	def next_goal(self):
		condition = self.threshold_box()
		if(condition):
			rospy.sleep(0.01)
			if(self.index < len(self.x_goals)-1):
				self.index += 1
				# rospy.loginfo(self.index)
				self.goal_position = [
                self.x_goals[self.index], 
                self.y_goals[self.index], 
                self.theta_goals[self.index]
            ]

	def safety_check(self, vel):
		if(vel < -1.5):
			return -1.5
		if(vel > 1.5):
			return 1.5

		return vel

	def inverse_kinematics(self,):
		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 
		#	-> Use the target velocity you calculated for the robot in previous task, and
		#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
		#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
		############################################

		transform_matrix = np.linalg.inv(self.tr_mat)
		local_frame_velocicites = np.array([[self.vel_x], [self.vel_y], [self.vel_z]])

		[self.front_wheel_force, self.left_wheel_force, self.right_wheel_force] = np.dot(transform_matrix,local_frame_velocicites)

		self.front_wheel_force = self.front_wheel_force[0]
		self.left_wheel_force = self.left_wheel_force[0]
		self.right_wheel_force = self.right_wheel_force[0]

		self.front_wheel_force = 1500*self.front_wheel_force + (0.5)*(self.prev[0] - self.front_wheel_force)
		self.left_wheel_force = 1500*self.left_wheel_force + (0.5)*(self.prev[1] - self.left_wheel_force)
		self.right_wheel_force = 1500*self.right_wheel_force + (0.5)*(self.prev[2] - self.right_wheel_force)

		self.prev = [self.front_wheel_force, self.left_wheel_force,self.right_wheel_force]

	def local_frame_controller(self):

		self.error_global[0] = self.goal_position[0] - self.hola_position[0]
		self.error_global[1] = self.goal_position[1] - self.hola_position[1]
		self.error_global[2] = self.goal_position[2] - self.hola_position[2]

		# Calculating error in body frame
		self.w = self.hola_position[2]
		self.error_local[0] = self.error_global[0]*math.cos(self.w) - self.error_global[1]*math.sin(self.w)
		self.error_local[1] = -self.error_global[0]*math.sin(self.w) - self.error_global[1]*math.cos(self.w)


		self.vel_x = self.kp[0] * self.error_local[0] 
		self.vel_y = self.kp[0] * self.error_local[1]
		self.vel_z = self.kp[1] * self.error_global[2]
		
		# print(self.error_global)
		# Safety Check
		# to make sure the velocities are within a range.
		self.vel_x = self.safety_check(self.vel_x)
		self.vel_y = self.safety_check(self.vel_y)


		# self.vel.linear.x = self.vel_x
		# self.vel.linear.y = self.vel_y
		# self.vel.angular.z = self.vel_z

		# # print(self.vel)
		# self.pub.publish(self.vel)

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
			
			if self.is_ready():
				print("Waiting!")
				self.rate.sleep()
				continue
			self.goal_position = [
                self.x_goals[self.index], 
                self.y_goals[self.index], 
                self.theta_goals[self.index]
            ]
			# print(self.goal_position)
			# Calculate Error from feedback

			# Change the frame by using Rotation Matrix (If you find it required)

			# Calculate the required velocity of bot for the next iteration(s)
			
			# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

			# Apply appropriate force vectors

			# Modify the condition to Switch to Next goal (given position in pixels instead of meters)
			self.local_frame_controller()
			self.inverse_kinematics()

			self.front_w.force.x = self.front_wheel_force
			self.right_w.force.x = self.right_wheel_force
			self.left_w.force.x = self.left_wheel_force

			# print(self.front_wheel_force,self.right_wheel_force,self.left_wheel_force)
			self.right_wheel_pub.publish(self.right_w)
			self.front_wheel_pub.publish(self.front_w)
			self.left_wheel_pub.publish(self.left_w)
			
			print(self.index)
			self.rate.sleep()

			self.next_goal()

		############################################

if __name__ == "__main__":
	try:
		control = Controller()
		control.main()
	except rospy.ROSInterruptException:
		pass

