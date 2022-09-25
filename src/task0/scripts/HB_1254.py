#!/usr/bin/env python3

'''
creating a node named, /ebot_controller:
and publish geometry_msgs/Twist to /cmd_vel
'''

import rospy
from geometry_msgs.msg import Twist     # to use 'geometry_msgs/Twist' message type from '/cmd_vel' topic
import math


class Controller():
    def __init__(self):
        # Initializing node to communicating with the ROS Master 
        rospy.init_node('ebot_controller')
        
        # declaring that node is publishing to the 'cmd_vel' topic using the message type 'geometry_msgs.Twist'
        self.pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10) 

        self.velocity_msg = Twist()
        self.velocity_msg.linear.x = 0
        self.velocity_msg.angular.z = 0

        self.speed = 1
        self.angular_speed = 1

        self.flag = True

        # self.pub_vel.publish(self.velocity_msg)

    def distance(self,v,w,angle): return abs((angle)*v/w)

    def follow_circle(self,angle):
        # speed = 1   
        self.velocity_msg.linear.x = self.speed
        self.velocity_msg.angular.z = self.angular_speed
        self.pub_vel.publish(self.velocity_msg)
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        while(current_distance <= self.distance(self.velocity_msg.linear.x,self.velocity_msg.angular.z,angle) + 0.2):
            self.pub_vel.publish(self.velocity_msg)
            self.rate.sleep()   
            t1 = rospy.Time.now().to_sec()
            current_distance = self.speed*(t1-t0)
            # rospy.loginfo("Moving in Circle")
            print("My turtleBot is: Moving in circle!!")
            print()
            self.velocity_msg.linear.x = self.speed
            self.velocity_msg.angular.z = self.angular_speed
            self.pub_vel.publish(self.velocity_msg)
    
    def rotate(self,sudo_angle):
        t0 = rospy.Time.now().to_sec()
        
        ori_rad = 0
        while(ori_rad <= (math.degrees(sudo_angle))):
            self.rate.sleep()  
            t1 = rospy.Time.now().to_sec()
            print("My turtleBot is: Rotating!")
            print()
            ori_rad = (180)*(1*(t1-t0))/(math.pi)
            self.velocity_msg.angular.z = self.angular_speed
            self.pub_vel.publish(self.velocity_msg)
        
        
    def go_straight(self,no_of_times_radius):
        self.speed = 1   
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        while(current_distance <= (no_of_times_radius*(2/2))):
            self.rate.sleep()  
            self.pub_vel.publish(self.velocity_msg)
            t1 = rospy.Time.now().to_sec()
            print("My turtleBot is: Moving Straight!!!")
            print()
            current_distance = self.speed*(t1-t0)
            
            self.velocity_msg.linear.x = self.speed
            self.velocity_msg.angular.z = 0
            self.pub_vel.publish(self.velocity_msg)

    def reset(self):
        self.velocity_msg.linear.x = 0
        self.velocity_msg.angular.z = 0
        self.pub_vel.publish(self.velocity_msg)



        

    


    
        





    def main(self):

        # global flag
        if(self.flag):


            self.rate.sleep()       # putting some delay so subscribers work perfectely from start
            self.follow_circle((math.pi))
            self.reset()
            self.rotate((math.pi/2))
            self.reset()
            self.go_straight(2)
            self.reset()
            print("Done!")
            self.flag = False
        
    


if __name__ == '__main__':
    try:
        controller = Controller()
        while not rospy.is_shutdown():
            controller.main()
            # self.pub_vel.publish(self.velocity_msg)
    except rospy.ROSInterruptException as e:
        print(e)
