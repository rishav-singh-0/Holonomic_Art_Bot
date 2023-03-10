#!/usr/bin/env python3

'''
* Team Id : HB#1254
* Author List : Rishav
* Filename: path_planner.py
            Path Planner decides path(list of goals) for bot to follow.
            Also performs the task of position controller.
* Theme: Hola Bot -- Specific to eYRC 2022-23
* Functions: 
    PathPlanner:
        aruco_feedback_Cb(msg), end_signal_Cb(msg), is_ready(), threshold_box(), next_goal(), 
        cleanup(), safety_check(), position_controller(), image_mode(), function_mode(),
        publish_contours(), publisher(), main()
* Global Variables: None

* Node: path_planner_node:
            This python file runs a ROS-node of name path_planner_node which
            gives the required velocity to reach next goal. Also implements the 
            pen up-down mechanism.

        This node publishes and subscribes the following topics:
                PUBLICATIONS            SUBSCRIBTIONS
                /path_plan             /detected_aruco
                /contours              /endSignal
                /penStatus
                /taskStatus
'''


################### IMPORT MODULES #######################

import rospy
from cv_basics.msg import aruco_data
from geometry_msgs.msg import Twist         # velocity
from std_msgs.msg import String             # x and y setpoints/pixel list
from std_msgs.msg import Int32              # taskStatus

import numpy as np
import cv2
import math		# If you find it useful
from math import pi as PI


class PathPlanner():

    def __init__(self):
        ################## GLOBAL VARIABLES ######################

        self.x_goals = []
        self.y_goals = []
        self.theta_goals = []

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
        self.contours = []
        self.penData = Int32()
        self.penData.data = 0       # Publish 1 when the pen is down(drawing) and 0 when the pen is up
        
        # see if all the destinations are reached
        self.taskCompleted = 0      # 0 means not completed

        # task status
        self.taskStatus = Int32()
        self.taskStatus.data = 0    # indicating start of the run, 0 means running, 1 means taskEnd

        #################### ROS Node ############################

        rospy.init_node('path_planner_node')

        self.rate = rospy.Rate(200)

        self.goal_publisher = rospy.Publisher('/path_plan', Twist, queue_size=10)
        
        self.contourPub = rospy.Publisher('/contours', String, queue_size=10)
        self.penPub = rospy.Publisher('/penStatus', Int32, queue_size=10)
        self.taskStatusPub = rospy.Publisher('/taskStatus', Int32, queue_size=10)

        rospy.Subscriber('/endSignal',Int32, self.end_signal_Cb)
        rospy.Subscriber('/detected_aruco', aruco_data, self.aruco_feedback_Cb)
        
    ##################### FUNCTION DEFINITIONS #######################

    def aruco_feedback_Cb(self, msg):
        self.hola_position[0] = msg.x
        self.hola_position[1] = msg.y
        self.hola_position[2] = msg.theta
        # print(self.hola_position)
    
    def end_signal_Cb(self, msg):
        if msg.data == 1:
            self.cleanup()

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
            reached_last_contour = self.contour_index == len(self.x_goals) - 1
            # rospy.loginfo("pos: "+str(reached_pos) + " last_pos: "+str(reached_last_pos) + " contour: "+str(reached_contour))

            rospy.loginfo("Goal reached " + str(self.goal_index)+": "+str(self.goal_position))
            self.goal_position = [
                self.x_goals[self.contour_index][self.goal_index], 
                self.y_goals[self.contour_index][self.goal_index], 
                self.theta_goals[self.contour_index][self.goal_index]
            ]

            # changing to next contour
            if((reached_contour) and reached_last_pos):
                rospy.loginfo("Changing Contour: " + str(self.contour_index)+": "+str(self.goal_position))
                self.contour_index += 1
                self.goal_index = 0
            
            # changing to next goal
            if(reached_pos):
                self.goal_index += 1
                rospy.sleep(0.1)
            
            # finish job if all setpoints are reached
            if(reached_last_pos and reached_last_contour):
                self.taskCompleted = 1
                rospy.loginfo("Run Finished!")
                self.cleanup()
            
    def cleanup(self):
        if self.taskCompleted == 1:
            self.taskStatus.data = 1
        self.taskStatusPub.publish(self.taskStatus)
        self.goal_publisher.publish(self.vel)
        self.penData.data = 0
        self.penPub.publish(self.penData)
        self.rate.sleep()
        rospy.signal_shutdown("Run Finished!")
        exit(0)

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

        # img_path = "/mnt/STORAGE/project/hola_bot/src/task5b/src/smile.png"
        # img_path = "/mnt/STORAGE/project/hola_bot/src/task5b/src/snapchat.png"
        img_path = "/mnt/STORAGE/project/hola_bot/src/task5b/src/robotFinal.png"
        rospy.loginfo("Image: " + img_path)

        img = cv2.imread(img_path, 0)

        img = cv2.resize(img,size_img)
        black = np.zeros((size_img[0],size_img[1],3),np.uint8)

        _, thresh = cv2.threshold(img,30,255,cv2.THRESH_BINARY)
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        self.contours = contours

        xList, yList, wList = [], [], []

        x_goals, y_goals, theta_goals = [], [], []
        x_goals_tot, y_goals_tot = [], []

        for i in [0, 2, 3, 7, 8, 9, 11, 13]:

            xList, yList, wList = [], [], []

            # if(hierarchy[0][i][3] != -1):
            if(True):
                xList, yList, wList = [], [], []
                len_cont = len(contours[i])
                for j in range(0,len_cont):
                    black[contours[i][j][0][1]][contours[i][j][0][0]] = 255
                    xList.append(contours[i][j][0][0])
                    yList.append(contours[i][j][0][1])
                x_goals_tot.append(xList)
                y_goals_tot.append(yList)

                # perimeter = cv2.arcLength(contours[i], closed=True)
                # epsilon = 0.008*perimeter
                epsilon = 0.006*size_img[1]
                contours[i] = cv2.approxPolyDP(contours[i], epsilon, closed=True)

                xList, yList, wList = [], [], []
                len_cont = len(contours[i])
                for j in range(0,len_cont):
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
        
        self.cData.data = str([x_goals_tot, y_goals_tot])
        
        # print(x_goals, len(x_goals))
        # print(y_goals, len(y_goals))
        # cv2.imwrite("out_def.png",black)

        self.x_goals = x_goals
        self.y_goals = y_goals
        self.theta_goals = theta_goals

        self.publish_contours()

    def function_mode(self):
        
        # take few points from 0 to 2*PI and generate setpoints in x, y and theta arrays
        t = np.linspace(0, 2*PI, num=self.max_setpoints)       
        x = lambda p: 200*math.cos(p) + 250
        y = lambda p: 100*math.sin(2*p) + 250
        theta = lambda p: (PI/4)*math.sin(p + PI/2) # you may need to add a phase shift
        
        x_goals, y_goals, theta_goals = [], [], []
        for index in range(len(t)):
            x_goals.append(int(x(t[index])))
            y_goals.append(int(y(t[index])))
            theta_goals.append(int(theta(t[index])))
        
        self.x_goals = [x_goals]
        self.y_goals = [y_goals]
        self.theta_goals = [theta_goals]
        self.cData.data = str([self.x_goals, self.y_goals])

        self.publish_contours()

    def publish_contours(self):
        # publishing contour
        # self.cData.data = str([self.x_goals, self.y_goals])
        self.contourPub.publish(self.cData)
        self.rate.sleep()

    def publisher(self):
        
        # Publishing required velocity in x, y, theta form
        self.goal_publisher.publish(self.vel)
        # rospy.loginfo("Goal: "+str([self.vel.linear.x, self.vel.linear.y, self.vel.angular.z]))
        
        # Pen up-down mechanism
        if(self.goal_index == 0):
            self.penData.data = 0
        else:
            self.penData.data = 1
        self.penPub.publish(self.penData)
        
        # task status publisher
        self.taskStatus.data = 1 if (self.taskCompleted==1) else 0
        self.taskStatusPub.publish(self.taskStatus)

        # publishing contour
        self.publish_contours()

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

