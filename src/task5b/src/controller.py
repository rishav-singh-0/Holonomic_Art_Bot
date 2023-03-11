#!/usr/bin/env python3

'''
* Team Id : HB#1254
* Author List : Rishav, Kashyap
* Filename: path_planner.py
            Takes velocity setpoints from path_planner_node and calculates force required for
            each wheel to reach this setpoint
* Theme: Hola Bot -- Specific to eYRC 2022-23
* Functions: 
    Controller:
        path_goals_callback(msg), pen_status_callback(msg), task_status_callback(msg), 
        is_ready(), connect_socket(), socket_send_data(), signal_handler(sig, frame),
        cleanup(), inverse_kinematics(), publish_force(), main()
* Global Variables: None

* Node: controller_node:
            This python file runs a ROS-node of name controller_node which
            gives the force required for each wheel to reach given setpoint.
            And gives all the required data of wheel speeds and penStatus to 
            the Bot through socket connection.

        This node publishes and subscribes the following topics:
                PUBLICATIONS            SUBSCRIBTIONS
                /right_wheel_force      /path_plan
                /front_wheel_force      /penStatus
                /left_wheel_force       /taskStatus
'''


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		    # To handle Signals by OS/user
import socket
import math
import numpy as np

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from std_msgs.msg import Int32              # penStatus
from geometry_msgs.msg import Twist         # velocity


class Controller():
    '''
    Purpose:
    ---
    Take data from the path_planner in form of the topics like penStatus, path_plan
    and taskStatus and process it and using that information drive the PID controller
    and control the forces provided to the wheels.
    
    Input Arguments:
    ---
    None
    '''

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
        
        # drawing pen status
        self.penStatus = 0

        #################### ROS Node ############################

        rospy.init_node('controller_node')

        signal.signal(signal.SIGINT, self.signal_handler)
        self.rate = rospy.Rate(200)

        self.right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
        self.front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

        rospy.Subscriber('/path_plan', Twist, self.path_goals_callback)
        rospy.Subscriber('/penStatus', Int32, self.pen_status_callback)
        rospy.Subscriber('/taskStatus', Int32, self.task_status_callback)
        
    ##################### FUNCTION DEFINITIONS #######################

    def path_goals_callback(self, msg):
        '''
        Purpose:
        ---
        Callback function which collects the velocity information from the
        topic "/path_plan"
        
        Input Arguments:
        ---
        msg :  Twist

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when the information is being published on the
        topic "/path_plan" and updates the self.vel
        '''

        self.vel['x'] = msg.linear.x
        self.vel['y'] = msg.linear.y
        self.vel['w'] = msg.angular.z
        
    def pen_status_callback(self, msg):
        '''
        Purpose:
        ---
        Callback function that collects the status of pen(up/down) from the
        topic "/penStatus"
        
        Input Arguments:
        ---
        msg :  Int32

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when the information is being published on the topic "/penStatus" 
        and self.penStatus is also updated on each call
        '''

        self.penStatus = msg.data
    
    def task_status_callback(self, msg):
        '''
        Purpose:
        ---
        Callback function that collects the status of pen from the topic
        "/taskStatus". Cleans all the properties when the task is completed.
        
        Input Arguments:
        ---
        msg :  Int32

        Returns:
        ---
        None

        Example call:
        ---
        Called when the information is being published on the topic "/taskStatus"
        '''

        self.taskStatus = msg.data
        
        # cleanup when task is completed
        if self.taskStatus == 1:
            self.cleanup()

    def is_ready(self):
        '''
        Purpose:
        ---
        Checks if the node is ready to run by checking if the required
        parameters are available/updated
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        Boolean

        Example call:
        ---
        self.is_ready()
        '''

        condition = (self.vel['x'] == None)
        return condition
    
    def connect_socket(self):
        '''
        Purpose:
        ---
        It will establish socket connection between bot and base station.
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.connect_socket()
        '''

        ip = "192.168.43.129"     # IP address of laptop after connecting it to WIFI hotspot

        self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.connection.bind((ip, 8002))
        self.connection.listen()
        self.socket_conn, addr = self.connection.accept()
        rospy.loginfo(f"Connected by {addr}")
    
    def socket_send_data(self):
        '''
        Purpose:
        ---
        Sends the speeds/forces and penStatus data to the socket client(in our
        case its esp32)
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.socket_send_data()
        '''

        data = [round(i, 2) for i in self.wheel_force.values()]
        data.append(self.penStatus)
        data = str(data) + '\n'
        rec_data = self.socket_conn.recv(1024)
        rospy.loginfo(data)
        # rospy.loginfo(rec_data)
        self.socket_conn.sendall(str.encode(data))
        rospy.sleep(0.1)

    def signal_handler(self, sig, frame):
        '''
        Purpose:
        ---
        Handels the signal generated by the OS/USER (Ex: SIGINT) and runs
        cleanup function when signal is generated
        
        Input Arguments:
        ---
        sig, frame

        Returns:
        ---
        None

        Example call:
        ---
        Called when signal is generated
        '''

        rospy.logdebug(str(sig)+": Signal called!")
        self.cleanup()
    
    def cleanup(self):
        '''
        Purpose:
        ---
        Close the socket connection, publish zero force to stop the bot and it
        will reset the gazebo world. Basically does the cleanup work before
        exitting.
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.cleanup()
        '''

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
        self.rate.sleep()
        # rospy.signal_shutdown("Controller: Finished Job!")
        sys.exit(0)

    def inverse_kinematics(self):
        '''
        Purpose:
        ---
        Calculates the force needed for each wheels to reach desired velocity
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.inverse_kinematics()
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
        '''
        Purpose:
        ---
        Publishes the forces to front, left, right wheels.
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.inverse_kinematics()
        '''

        self.front_w.force.x = self.wheel_force['front']
        self.left_w.force.x = self.wheel_force['left']
        self.right_w.force.x = self.wheel_force['right']

        self.front_wheel_pub.publish(self.front_w)
        self.left_wheel_pub.publish(self.left_w)
        self.right_wheel_pub.publish(self.right_w)

    def main(self):
        '''
        Purpose:
        ---
        Execute the required methods in order to control and continuining the
        communication with bot.
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.main()
        '''

        while not rospy.is_shutdown():

            self.rate.sleep()
            
            # checking if the subscibed variables are on position
            if self.is_ready():
                print("Waiting in Controller!")
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

