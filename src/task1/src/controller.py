#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type: Twist

from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

class PositionController():
    def __init__(self):
        # position as [x, y, theta]
        self.hola_position = [0, 0, 0]
        self.goal_position = [0, 0, 0]

        # destination positions for this task
        self.x_goals = [1, -1, -1, 1, 0]
        self.y_goals = [1, 1, -1, -1, 0]
        self.theta_goals = [math.pi/4, 3*math.pi/4, -3*math.pi/4, -math.pi/4, 0] 
        self.index = 0

        # Declare a Twist message
        self.vel = Twist()

        # variables for P controller
        self.kp = [0.7, 0.7]
        self.error_global = [0, 0, 0]
        self.error_local = [0, 0]       # only needs [x, y]

        # Initialze Node
        rospy.init_node('controller')
        
        # Initialze Publisher and Subscriber

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odometryCb)

        # Initialise variables that may be needed for the control loop
        # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
        # and also Kp values for the P Controller

        # For maintaining control loop rate.
        self.rate = rospy.Rate(100)


    def odometryCb(self, msg):

        # Write your code to take the msg and update the three variables
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        self.hola_position[0] = msg.pose.pose.position.x
        self.hola_position[1] = msg.pose.pose.position.y
        self.hola_position[2] = euler_from_quaternion([x,y,z,w])[2]

    def threshold_box(self):
        condition = abs(self.error_global[0]) < 0.05 and \
            abs(self.error_global[1]) < 0.05 and \
            abs(self.error_global[2]) < 1
        if(condition):
            print("Threshold reached!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
        return condition
    
    def next_goal(self):
        condition = self.threshold_box()
        print(condition,end=" ")
        if(condition):
            self.index += 1
            self.goal_position = [
                self.x_goals[self.index], 
                self.y_goals[self.index], 
                self.theta_goals[self.index]
            ]
            print()
            print(self.index)
            print()


    def p_controller(self):
        
        self.goal_position = [self.x_goals[0], self.y_goals[0], self.theta_goals[0]]

        while not rospy.is_shutdown():

            # Find error (in x, y and theta) in global frame
            # the /odom topic is giving pose of the robot in global frame
            # the desired pose is declared above and defined by you in global frame
            # therefore calculate error in global frame
            self.error_global[0] = self.goal_position[0] - self.hola_position[0]
            self.error_global[1] = self.goal_position[1] - self.hola_position[1]
            self.error_global[2] = self.goal_position[2] - self.hola_position[2]

            # (Calculate error in body frame)
            # But for Controller outputs robot velocity in robot_body frame, 
            # i.e. velocity are define is in x, y of the robot frame, 
            # Notice: the direction of z axis says the same in global and body frame
            # therefore the errors will have have to be calculated in body frame.
            w = self.error_global[2]
            self.error_local[0] = self.error_global[0]*math.cos(self.hola_position[2]) + self.error_global[1]*math.sin(self.hola_position[2])
            self.error_local[1] = -self.error_global[0]*math.sin(self.hola_position[2]) + self.error_global[1]*math.cos(self.hola_position[2])

            # Finally implement a P controller 
            # to react to the error with velocities in x, y and theta.
            vel_x = self.kp[0] * self.error_local[0]
            vel_y = self.kp[0] * self.error_local[1]
            vel_z = self.kp[1] * self.error_global[2]

            # Safety Check
            # make sure the velocities are within a range.
            # for now since we are in a simulator and we are not dealing with actual physical limits on the system 
            # we may get away with skipping this step. But it will be very necessary in the long run.
            if(-2 >vel_x or vel_x>2):
                if(vel_x<-1):
                    vel_x = -0.5
                if(vel_x>1):
                    vel_x = 0.5
            
            if(-2 >vel_y or vel_y>2):
                if(vel_y<-1):
                    vel_y = -0.5
                if(vel_y>1):
                    vel_y = 0.5

            self.vel.linear.x = vel_x
            self.vel.linear.y = vel_y
            self.vel.angular.z = vel_z
            # print(self.error_global, self.error_local)
            # print(vel_x, vel_y, vel_z)
            # print(self.error_local[0], self.error_local[1], self.error_global[2])
            print("vel: ",end = "")
            print(vel_x,vel_y,vel_z)
            print("error: ",end = "")
            print(self.error_local[0],self.error_local[1], w)

            self.pub.publish(self.vel)
            self.rate.sleep()
            self.next_goal()

	

if __name__ == "__main__":
    try:
        control = PositionController()
        control.p_controller()
    except rospy.ROSInterruptException:
        pass
