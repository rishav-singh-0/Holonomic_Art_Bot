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
import cv2

# from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
from cv_basics.msg import aruco_data
from geometry_msgs.msg import Twist         # velocity
from std_msgs.msg import String             # x and y setpoints/pixel list
from std_msgs.msg import Int32              # taskStatus

import time
import math		# If you find it useful
from math import pi as PI

from tf.transformations import euler_from_quaternion	# Convert angles


class PathPlanner():

    def __init__(self):
        ################## GLOBAL VARIABLES ######################

        self.x_goals = []
        self.y_goals = []
        self.theta_goals = []
        # self.x_goals = np.array([250, 255, 260, 270, 270, 270])
        # self.y_goals = np.array([250, 250, 250, 255, 260, 265])
        # self.theta_goals = np.array([0, 0, 0, 0, 0, 0])
        # self.x_goals = np.array([np.array([150, 150, 350, 250]), np.array([150, 150, 350, 250])])
        # self.y_goals = np.array([np.array([300, 150, 150, 250]), np.array([300, 150, 150, 250])])
        # self.theta_goals = np.array([np.array([3*PI/4, -3*PI/4, -PI/4, 0]), np.array([3*PI/4, -3*PI/4, -PI/4, 0])])

        # position as [x, y, theta]
        self.hola_position = [0, 0, 0]              # current position
        self.goal_position = [None, None, None]     # current goals

        self.contour_index = 0				# For travercing the contours
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
        
        # publish variables
        self.cData = String()
        self.penData = Int32()
        self.penData.data = 0       # brush is not drawing

        #################### ROS Node ############################

        rospy.init_node('path_planner_node')

        self.rate = rospy.Rate(200)

        self.goal_publisher = rospy.Publisher('/path_plan', Twist, queue_size=10)
        
        self.contourPub = rospy.Publisher('/contours', String, queue_size=10)
        self.penPub = rospy.Publisher('/penStatus', Int32, queue_size=10)

        # rospy.Subscriber('/endSignal',Int32, self.end_signal_Cb) #optional
        rospy.Subscriber('/detected_aruco', aruco_data, self.aruco_feedback_Cb)
        
    ##################### FUNCTION DEFINITIONS #######################

    def aruco_feedback_Cb(self, msg):
        self.hola_position[0] = msg.x
        self.hola_position[1] = msg.y
        self.hola_position[2] = msg.theta
        # print(self.hola_position)
    
    def end_signal_Cb(self, msg):
        print(msg.data)

    def is_ready(self):
        condition = self.x_goals == [] or \
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

            reached_pos = self.goal_index < len(self.x_goals[self.contour_index]) - 1
            reached_last_pos = self.goal_index == len(self.x_goals[self.contour_index]) - 1
            reached_contour = self.contour_index < len(self.x_goals) - 1
            # rospy.loginfo("pos: "+str(reached_pos) + " last_pos: "+str(reached_last_pos) + " contour: "+str(reached_contour))

            rospy.loginfo("Goal reached " + str(self.goal_index)+": "+str(self.goal_position))
            self.goal_position = [
                self.x_goals[self.contour_index][self.goal_index], 
                self.y_goals[self.contour_index][self.goal_index], 
                self.theta_goals[self.contour_index][self.goal_index]
            ]

            # changing contour
            if((reached_contour) and reached_last_pos):
                rospy.loginfo("Changing Contour: " + str(self.contour_index)+": "+str(self.goal_position))
                self.contour_index += 1
                self.goal_index = 0

            # next goal
            if(reached_pos):
                self.goal_index += 1
                rospy.sleep(0.1)
            


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
        
        size_img = (500,500)
        # max_points = 20

        img_path = "/mnt/STORAGE/project/hola_bot/src/task5b/src/smile.png"
        # img_path = "/mnt/STORAGE/project/hola_bot/src/task5b/src/snapchat.png"
        rospy.loginfo("Image: " + img_path)

        img = cv2.imread(img_path, 0)

        img = cv2.resize(img,size_img)
        black = np.zeros((size_img[0],size_img[1],3),np.uint8)

        _, thresh = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        xList, yList, wList = [], [], []

        x_goals, y_goals, theta_goals = [], [], []

        for i in range(0,len(contours)):

            xList, yList, wList = [], [], []

            len_cont = len(contours[i])

            if(hierarchy[0][i][3] != -1):
                for j in range(0,len_cont, 10):
                    black[contours[i][j][0][1]][contours[i][j][0][0]] = 255
                    xList.append(contours[i][j][0][0])
                    yList.append(contours[i][j][0][1])
                    wList.append(0)
            
                x_goals.append(xList)
                y_goals.append(yList)
                theta_goals.append(wList)
        
        for cont in range(len(x_goals)):
            x_goals[cont].append(x_goals[cont][0])
            y_goals[cont].append(y_goals[cont][0])
            theta_goals[cont].append(theta_goals[cont][0])
        
        # print(x_goals, len(x_goals))
        # print(y_goals, len(y_goals))
        # cv2.imwrite("out_def.png",black)

        self.x_goals = x_goals
        self.y_goals = y_goals
        self.theta_goals = theta_goals

        # self.publish_contours()

    def function_mode(self):
        
        # take few points from 0 to 2*PI and generate setpoints in x, y and theta arrays
        t = np.linspace(0, 2*PI, num=self.max_setpoints)       
        x = lambda t: 200*math.cos(t) + 250
        y = lambda t: 100*math.sin(2*t) + 250
        theta = lambda t: (PI/4)*math.sin(t) # you may need to add a phase shift
        
        self.x_goals = np.array([np.array([x(i) for i in t])])
        self.y_goals = np.array([np.array([y(i) for i in t])])
        self.theta_goals = np.array([np.array([theta(i) for i in t])])

        self.x_goals = list(self.x_goals)
        self.y_goals = list(self.x_goals)
        self.theta_goals = list(self.theta_goals)
        # self.publish_contours()

    def publisher(self):
        self.goal_publisher.publish(self.vel)
        # rospy.loginfo("Goal: "+str([self.vel.linear.x, self.vel.linear.y, self.vel.angular.z]))
        
        # Pen up-down mechanism
        if(self.goal_index == 0):
            self.penData.data = 0
        else:
            self.penData.data = 1
        self.penPub.publish(self.penData)

        # publishing contour
        self.cData.data = str([self.x_goals,self.y_goals])
        self.contourPub.publish(self.cData)


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
                    self.x_goals[self.contour_index][self.goal_index], 
                    self.y_goals[self.contour_index][self.goal_index], 
                    self.theta_goals[self.contour_index][self.goal_index]
                ]
            except IndexError as e:
                rospy.logerr(e)
                continue

            self.position_controller()
            # print(self.hola_position)

            # print(self.vel)
            self.publisher()
            self.next_goal()


if __name__ == "__main__":
    try:
        control = PathPlanner()
        # control.function_mode()
        control.image_mode()
        control.main()
    except rospy.ROSInterruptException:
        pass

