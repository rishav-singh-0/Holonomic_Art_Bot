#!/usr/bin/env python3

'''
*****************************************************************************************
Takes setpoints from path planner and calculates force required for each wheel to reach 
this setpoint
*****************************************************************************************
'''

# Author List:	Rishav Singh, Kashyap Joshi
# Filename:		controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user
import socket
import numpy as np

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
from std_srvs.srv import Empty			# for shutdown hook
from geometry_msgs.msg import Twist         # velocity

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

class Controller():

    def __init__(self):
        ################## GLOBAL VARIABLES ######################

        # force vectors initialization
        self.right_wheel = Wrench()
        self.left_wheel = Wrench()
        self.front_wheel = Wrench()

        # variables for P controller
        self.const_force = [2.1*200.0, 4*0.002, 4*0.00]	# [kp, kd, ki]
        self.err_prev = { 'front':0, 'left':0, 'right':0 }
        self.err_sum = [0, 0, 0]

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

        # socket connection
        self.connection =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_conn = 0

        #################### ROS Node ############################

        rospy.init_node('controller_node')

        signal.signal(signal.SIGINT, self.signal_handler)
        self.rate = rospy.Rate(200)

        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
        self.front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

        rospy.Subscriber('path_plan', Twist, self.path_goals_callback)
        
    ##################### FUNCTION DEFINITIONS #######################

    def path_goals_callback(self, msg):
        self.vel['x'] = msg.linear.x
        self.vel['y'] = msg.linear.y
        self.vel['w'] = msg.angular.z

    def is_ready(self):
        condition = self.vel['x'] == None or \
                    self.vel['y'] == None
        return condition
    
    def connect_socket(self):

        ip = "192.168.43.129"     # IP address of laptop after connecting it to WIFI hotspot

        self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.connection.bind((ip, 8002))
        self.connection.listen()
        self.socket_conn, addr = self.connection.accept()
        rospy.loginfo(f"Connected by {addr}")
    
    def socket_send_data(self):
        # data = list(self.wheel_force.values())
        data = str([round(i, 2) for i in self.wheel_force.values()]) + '\n'
        rec_data = self.socket_conn.recv(1024)
        # rospy.loginfo(data)
        # rospy.loginfo(rec_data)
        self.socket_conn.sendall(str.encode(data))
        rospy.sleep(0.1)

    def signal_handler(self, sig, frame):
        print('Clean-up !')
        
        # closing socket connection
        self.connection.close()
        
        # publishing 0 force to each wheels
        force_zero = Wrench()
        force_zero.force.x = force_zero.force.y = force_zero.force.z = 0
        self.right_wheel_pub.publish(force_zero)
        self.front_wheel_pub.publish(force_zero)
        self.left_wheel_pub.publish(force_zero)
        
        # resetting gazebo world
        # self.reset_world()

        print("cleanup done")
        sys.exit(0)

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
            self.wheel_force[i] = self.const_force[0]*self.wheel_force[i] \
                                    + self.const_force[1]*(self.wheel_force[i] - self.err_prev[i])
                                # + self.const_force[2]*self.sum[i]
        self.err_prev = self.wheel_force
        # self.err_sum = [self.sum[0] + self.wheel_force['front'], self.sum[1] + self.wheel_force['left'], self.sum[2] + self.wheel_force['right']]

        
    def publish_force(self):
        self.front_w.force.x = self.wheel_force['front']
        self.left_w.force.x = self.wheel_force['left']
        self.right_w.force.x = self.wheel_force['right']

        self.front_wheel_pub.publish(self.front_w)
        self.left_wheel_pub.publish(self.left_w)
        self.right_wheel_pub.publish(self.right_w)

    def main(self):

        while not rospy.is_shutdown():

            self.rate.sleep()
            
            # checking if the subscibed variables are on position
            if self.is_ready():
                print("Waiting!")
                continue
            
            # calculate force of each wheel according to given velocities
            self.inverse_kinematics()

            # publish forces
            self.publish_force()

            # send the speeds/forces to esp32
            self.socket_send_data()


if __name__ == "__main__":
    try:
        control = Controller()
        control.connect_socket()
        control.main()
    except rospy.ROSInterruptException:
        pass

