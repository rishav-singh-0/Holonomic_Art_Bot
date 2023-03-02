#!/usr/bin/env python3

'''
*****************************************************************************************
Path Planner decides path(list of goals) for bot to follow.
Also performs the task of position controller.
*****************************************************************************************
'''

# Author List:  Rishav Singh
# Filename:		path_planner.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rospy
import numpy as np

from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
from geometry_msgs.msg import Twist         # velocity

import time
import math		# If you find it useful
from math import pi as PI

from tf.transformations import euler_from_quaternion	# Convert angles


class PathPlanner():

    def __init__(self):
        ################## GLOBAL VARIABLES ######################

        self.x_goals = np.array([])
        self.y_goals = np.array([])
        self.theta_goals = np.array([])
        # self.x_goals = np.array([250, 255, 260, 270, 270, 270])
        # self.y_goals = np.array([250, 250, 250, 255, 260, 265])
        # self.theta_goals = np.array([0, 0, 0, 0, 0, 0])
        # self.x_goals = np.array([150, 150, 350, 250])
        # self.y_goals = np.array([300, 150, 150, 250])
        # self.theta_goals = np.array([3*PI/4, -3*PI/4, -PI/4, 0])

        # position as [x, y, theta]
        self.hola_position = [0, 0, 0]              # current position
        self.goal_position = [None, None, None]     # current goals

        self.goal_index = 0					# For travercing the setpoints
        
        self.max_setpoints = 30            # max setpoints to be trasversed by bot

        # variables for P controller
        self.const_vel = [0.0065, 0.50]			# [kp_xy, kp_w]

        self.error_global = [0, 0, 0]
        self.error_local = {'x':0, 'y':0}       # only needs [x, y]
        self.err_sum = [0, 0, 0]

        # Variables for directional speed
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0

        #################### ROS Node ############################

        rospy.init_node('path_planner_node')

        self.rate = rospy.Rate(200)

        self.goal_publisher = rospy.Publisher('path_plan', Twist, queue_size=10)

        rospy.Subscriber('detected_aruco', Pose2D, self.aruco_feedback_Cb)
        
    ##################### FUNCTION DEFINITIONS #######################

    def aruco_feedback_Cb(self, msg):
        self.hola_position[0] = msg.x
        self.hola_position[1] = msg.y
        self.hola_position[2] = msg.theta
        # print(self.hola_position)
    
    def is_ready(self):
        condition = self.x_goals.any() == False or \
                    self.hola_position[0] == None
        # print(self.x_goals,self.hola_position)
        return condition
    
    def threshold_box(self):
        condition = abs(self.error_global[0]) < 4 and \
                    abs(self.error_global[1]) < 4 and \
                    abs(math.degrees(self.error_global[2])) <= 5
        return condition

    def next_goal(self):
        condition = self.threshold_box()
        # print(self.hola_position)
        if(condition):
            if(self.goal_index < len(self.x_goals)-1):
                self.goal_index += 1
                rospy.sleep(0.1)
                rospy.loginfo("Goal reached " + str(self.goal_index)+": "+str(self.goal_position))
                self.goal_position = [
                    self.x_goals[self.goal_index], 
                    self.y_goals[self.goal_index], 
                    self.theta_goals[self.goal_index]
                ]

    def safety_check(self):
        '''
        Limit x, y velocities while maintaining the ratio between them to maintain trajectory
        '''
        max_velocity = 0.8
        # rospy.loginfo(self.vel.linear.x)
        ratio = 1.2
        if(abs(self.vel.linear.x) > max_velocity and abs(self.vel.linear.y) > max_velocity):
            self.vel.linear.x /= ratio
            self.vel.linear.y /= ratio
            self.vel.angular.z /= ratio

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
        self.vel.angular.z = self.const_vel[1] * self.error_global[2]	# angular velocity
        self.vel.linear.x = self.const_vel[0] * self.error_local['x']
        self.vel.linear.y = self.const_vel[0] * self.error_local['y']

        # Safety Check to ensure the velocities are within a range.
        self.safety_check()
        
    def image_mode(self):
        self.x_goals = []
        self.y_goals = []
        self.theta_goals = []

    def function_mode(self):
        
        # take few points from 0 to 2*PI and generate setpoints in x, y and theta arrays
        t = np.linspace(0, 2*PI, num=self.max_setpoints)       
        x = lambda t: 150*math.cos(t) + 250
        y = lambda t: 80*math.sin(2*t) + 250
        theta = lambda t: (PI/4)*math.sin(t) # you may need to add a phase shift
        
        self.x_goals = np.array([x(i) for i in t])
        self.y_goals = np.array([y(i) for i in t])
        self.theta_goals = np.array([theta(i) for i in t])


    def main(self):

        while not rospy.is_shutdown():

            self.rate.sleep()
            
            # checking if the subscibed variables are on position
            if self.is_ready():
                print("Waiting in path planner!")
                continue
            
            # removing error while going to next test case
            try:
                self.goal_position = [
                    self.x_goals[self.goal_index], 
                    self.y_goals[self.goal_index], 
                    self.theta_goals[self.goal_index]
                ]
            except IndexError as e:
                rospy.logerr(e)
                continue

            self.position_controller()
            # print(self.hola_position)

            # print(self.vel)
            self.goal_publisher.publish(self.vel)
            # rospy.loginfo("Goal: "+str([self.vel.linear.x, self.vel.linear.y, self.vel.angular.z]))
            self.next_goal()


if __name__ == "__main__":
    try:
        control = PathPlanner()
        control.function_mode()
        control.main()
    except rospy.ROSInterruptException:
        pass

