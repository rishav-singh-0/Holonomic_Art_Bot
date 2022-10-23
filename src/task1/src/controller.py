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

# auto eval
from geometry_msgs.msg import PoseArray


x_goals, y_goals, theta_goals = [], [], []

class PositionController():
    def __init__(self):
        # position as [x, y, theta]
        self.hola_position = [0, 0, 0]
        self.goal_position = [0, 0, 0]

        # destination positions for this task
        # self.x_goals = [1, -1, -1, 1, 0]
        # self.y_goals = [1, 1, -1, -1, 0]
        # self.theta_goals = [math.pi/4, 3*math.pi/4, -3*math.pi/4, -math.pi/4, 0] 
        self.index = 0

        # Declare a Twist message
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0

        # variables for P controller
        self.kp = [0.9, 1.8]
        self.error_global = [0, 0, 0]
        self.error_local = [0, 0]       # only needs [x, y]

        # Initialze Node
        rospy.init_node('controller')
        
        # Initialze Publisher and Subscriber

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odometryCb)
        rospy.Subscriber('/task1_goals', PoseArray, self.task1_goals_Cb)

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
    
    def task1_goals_Cb(self, msg):
        global x_goals, y_goals, theta_goals

        x_goals.clear()
        y_goals.clear()
        theta_goals.clear()

        for waypoint_pose in msg.poses:
            x_goals.append(waypoint_pose.position.x)
            y_goals.append(waypoint_pose.position.y)

            orientation_q = waypoint_pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            theta_goal = euler_from_quaternion (orientation_list)[2]
            theta_goals.append(theta_goal)

    def threshold_box(self):
        condition = (abs(self.error_global[0]) <= 0.05) and (abs(self.error_global[1]) <= 0.05) and (abs(math.degrees(self.error_global[2])) < 1)
        # if(condition):
        #     print("Threshold reached!\n\n\n")
        return condition
    
    def next_goal(self):
        condition = self.threshold_box()
        # print(condition,end=" ")
        if(condition):
            rospy.sleep(0.5)
            self.index += 1
            if(self.index>=(len(x_goals)-1)):
                self.index = len(x_goals)-1
            self.goal_position = [
                x_goals[self.index], 
                y_goals[self.index], 
                theta_goals[self.index]
            ]
            # print('\n',self.index,'\n')

    def safety_check(self, vel):
        if(vel < -2):
            return -1
        if(vel > 2):
            return 1
        return vel

    def p_controller(self):
        
        while not rospy.is_shutdown():
            if (x_goals == [] or x_goals == None):
                self.rate.sleep()
                continue
            rospy.loginfo(x_goals)

            self.goal_position = [
                x_goals[self.index], 
                y_goals[self.index], 
                theta_goals[self.index]
            ]

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
            vel_x = self.safety_check(vel_x)
            vel_y = self.safety_check(vel_y)

            self.vel.linear.x = vel_x
            self.vel.linear.y = vel_y
            self.vel.angular.z = vel_z
            # print(self.error_local[0], self.error_local[1], self.error_global[2])
            # print("vel: ", vel_x, vel_y, vel_z)
            # print("error: ", self.error_local[0], self.error_local[1], w)

            rospy.loginfo(self.vel)
            self.pub.publish(self.vel)
            self.rate.sleep()
            self.next_goal()

	

if __name__ == "__main__":
    try:
        control = PositionController()
        control.p_controller()
    except rospy.ROSInterruptException:
        pass
