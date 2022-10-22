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

class Controller():
    def __init__(self):
        # position as [x, y, theta]
        self.hola_position = [0, 0, 0]
        self.goal_position = [1, 1, math.pi/4]

        # Declare a Twist message
        self.vel = Twist()

        # variables for P controller
        self.kp = 2
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

    def main(self):

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
            self.error_local[0] = self.error_global[0]*math.cos(w) + self.error_global[1]*math.sin(w)
            self.error_local[1] = self.error_global[0]*math.sin(w) + self.error_global[1]*math.cos(w)

            # Finally implement a P controller 
            # to react to the error with velocities in x, y and theta.
            vel_x = self.kp * self.error_local[0]
            vel_y = self.kp * self.error_local[1]
            vel_z = self.kp * self.error_global[2]

            # Safety Check
            # make sure the velocities are within a range.
            # for now since we are in a simulator and we are not dealing with actual physical limits on the system 
            # we may get away with skipping this step. But it will be very necessary in the long run.

            self.vel.linear.x = vel_x
            self.vel.linear.y = vel_y
            self.vel.angular.z = vel_z
            # print(self.error_global, self.error_local)
            print(vel_x, vel_y, vel_z)

            self.pub.publish(self.vel)
            self.rate.sleep()

	

if __name__ == "__main__":
    try:
        control = Controller()
        control.main()
    except rospy.ROSInterruptException:
        pass
