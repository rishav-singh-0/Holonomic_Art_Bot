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

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

class Controller():

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

		self.error_global = [0, 0, 0]
		self.error_local = {'x':0, 'y':0}       # only needs [x, y]

		self.index = 0					# For travercing the setpoints

		# variables for P controller
		self.const_vel = [0.003, 0.06]			# [kp_xy, kp_w]
		self.const_force = [2000, 0.02, 0.0]	# [kp, kd, ki]
		# self.err_prev = { 'front':0, 'left':0, 'right':0 }
		# self.err_sum = [0,0,0]

		# Variables for wheel force
		self.wheel_force = { 'front': None, 'left': None, 'right': None }

		self.vel = {'x':None, 'y':None, 'w':None }

		self.front_w = Wrench()
		self.left_w = Wrench()
		self.right_w = Wrench()

		self.tr_mat = np.array([
			[1, -np.cos(math.radians(60)), -np.cos(math.radians(60))], 
			[-0, np.cos(math.radians(30)), -np.cos(math.radians(30))], 
			[-2, -2, -2]
		])
		self.transform_matrix = np.linalg.inv(self.tr_mat)

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
		rospy.wait_for_service('/gazebo/reset_world')
		self.reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)

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
		self.reset_world()
	
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
		condition = abs(self.error_global[0]) < 5 and \
					abs(self.error_global[1]) < 5 and \
					abs(math.degrees(self.error_global[2])) <= 1
		return condition

	def next_goal(self):
		condition = self.threshold_box()
		if(condition):
			if(self.index < len(self.x_goals)-1):
				self.index += 1
				rospy.sleep(0.1)
				# rospy.loginfo(self.index)
				self.goal_position = [
					self.x_goals[self.index], 
					self.y_goals[self.index], 
					self.theta_goals[self.index]
				]

	def safety_check(self, vel):
		constain = 2
		if(vel < -constain): return -constain
		if(vel > constain): return constain
		return vel

	def inverse_kinematics(self):
		'''
		Calculate the force to wheels needed for gaining required velocity
		'''

		local_frame_velocicites = np.array([[self.vel['x']], [self.vel['y']], [self.vel['w']]])

		force = np.dot(self.transform_matrix,local_frame_velocicites)
		self.wheel_force['front'] = force[0][0]
		self.wheel_force['left'] = force[1][0]
		self.wheel_force['right'] = force[2][0]

		for i in ['front', 'left', 'right']:
			self.wheel_force[i] = self.const_force[0]*self.wheel_force[i]
								# + self.const_force[1]*(self.wheel_force[i] - self.err_prev[i])
								# + self.const_force[2]*self.sum[i]
		# self.err_prev = self.wheel_force
		# self.err_sum = [self.sum[0] + self.wheel_force['front'], self.sum[1] + self.wheel_force['left'], self.sum[2] + self.wheel_force['right']]

	def position_controller(self):
		'''
		Calculate the velocity required to reach desired position goal
		'''

		for i in range(3):
			self.error_global[i] = self.goal_position[i] - self.hola_position[i]

		# Calculating error in body frame
		w = self.hola_position[2]
		self.error_local['x'] = self.error_global[0]*math.cos(w) - self.error_global[1]*math.sin(w)
		self.error_local['y'] = -self.error_global[0]*math.sin(w) - self.error_global[1]*math.cos(w)


		# P controller for velocity
		self.vel['w'] = self.const_vel[1] * self.error_global[2]	# angular velocity
		for i in ['x', 'y']:
			self.vel[i] = self.const_vel[0] * self.error_local[i]

			# Safety Check to ensure the velocities are within a range.
			self.vel[i] = self.safety_check(self.vel[i])
		
	def publish_force(self):
		self.front_w.force.x = self.wheel_force['front']
		self.left_w.force.x = self.wheel_force['left']
		self.right_w.force.x = self.wheel_force['right']

		self.right_wheel_pub.publish(self.right_w)
		self.front_wheel_pub.publish(self.front_w)
		self.left_wheel_pub.publish(self.left_w)

	def main(self):

		while not rospy.is_shutdown():
			
			# checking if the subscibed variables are on position
			if self.is_ready():
				print("Waiting!")
				self.rate.sleep()
				continue
			
			# removing error while going to next test case
			try:
				self.goal_position = [
					self.x_goals[self.index], 
					self.y_goals[self.index], 
					self.theta_goals[self.index]
				]
			except IndexError as e:
				rospy.logerr(e)
				self.rate.sleep()
				continue

			self.position_controller()
			self.inverse_kinematics()

			self.publish_force()
			self.rate.sleep()
			self.next_goal()


if __name__ == "__main__":
	try:
		control = Controller()
		control.main()
	except rospy.ROSInterruptException:
		pass

