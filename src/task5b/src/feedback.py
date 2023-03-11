#!/usr/bin/env python3

'''
* Team Id : HB#1254
* Author List : Rishav
* Filename: feedback.py
            Takes raw feed from camera and calculates the localization(x, y, theta) 
            of bot with respect to arena
* Theme: Hola Bot -- Specific to eYRC 2022-23
* Functions: 
    Feedback:
        pen_status_callback(msg), end_signal_Cb(msg), draw_path(), centroid(arr), 
        create_vector(point_1, point_2), unit_vector(vector), 
        angle_between(vector_1, vector_2), side_length(point_1, point_2), 
        callback(data), publish(), main()
* Global Variables: None

* Node: aruco_feedback_node:
            This python file runs a ROS-node of name aruco_feedback_node which
            gives the current odom_data of bot ie. x, y and theta.

        This node publishes and subscribes the following topics:
                PUBLICATIONS            SUBSCRIBTIONS
                /detected_aruco         /usb_cam/image_raw
                                        /penStatus
                                        /endSignal
'''


######################## IMPORT MODULES ##########################

import rospy 				
from sensor_msgs.msg import Image 	    # Image is the message type for images in ROS
from cv_bridge import CvBridge	        # Package to convert between ROS and OpenCV Images
import cv2				                # OpenCV Library
from cv_basics.msg import aruco_data    # odom data publisher format
import math
import numpy as np
from collections import deque           # For storing drawn points
from std_msgs.msg import Int32          # penStatus


class Feedback():
    def __init__(self):
        ########################## GLOBALS #############################

        # self.aruco_msg = Pose2D()
        self.aruco_msg = aruco_data()
        self.current_frame = np.empty([])

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        self.bot_centroid = [250, 250]
        self.theta = 0
        
        # drawing pen status
        self.penStatus = 0
        self.drawn = deque(maxlen=10000)

        #################### ROS Node ################################

        rospy.init_node('aruco_feedback_node')
        # rospy.Subscriber('overhead_cam/image_raw', Image, self.callback)
        rospy.Subscriber('usb_cam/image_raw', Image, self.callback)
        self.aruco_publisher = rospy.Publisher('/detected_aruco', aruco_data, queue_size=10)
        rospy.Subscriber('/penStatus', Int32, self.pen_status_callback)
        rospy.Subscriber('/endSignal',Int32, self.end_signal_Cb)

    ##################### FUNCTION DEFINITIONS #######################

    def pen_status_callback(self, msg):
        '''
        Purpose:
        ---
        Callback function which collects the status of task from the topic
        "/penStatus"
        
        Input Arguments:
        ---
        msg :  Int32

        Returns:
        ---
        None

        Example call:
        ---
        It is called when the information is being published on the topic
        "/penStatus" and variable will also be updated on each call
        '''

        self.penStatus = msg.data
    
    def end_signal_Cb(self, msg):
        '''
        Purpose:
        ---
        Callback function which collects the information related to ending
        the signal in order to finish the run.
        
        Input Arguments:
        ---
        msg :  Int32

        Returns:
        ---
        None

        Example call:
        ---
        It is called when the information is being published on the topic
        "/endSignal"
        '''
        
        if msg.data == 1:
            rospy.signal_shutdown("Aruco Feedback: Run Finished!")
            exit(0)

    def draw_path(self):
        '''
        Purpose:
        ---
        Plots the points on the video feed collected from overhead camera
        when pen is in the draw mode. (Only for testing)
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.draw_path()
        '''

        if(self.penStatus == 1):
            self.drawn.appendleft((int(self.bot_centroid[0]), int(self.bot_centroid[1])))
        	# loop over the set of tracked points

        for i in range(1, len(self.drawn)):
            # # if either of the tracked points are None, ignore them
            # if self.drawn[i - 1] is None or self.drawn[i] is None:
            #     continue
     
            # draw the connecting lines
            # cv2.line(self.current_frame, self.drawn[i - 1], self.drawn[i], (0, 0, 255), 1)
            cv2.circle(self.current_frame, self.drawn[i], radius=0, color=(0, 0, 255), thickness=2)

    def centroid(self, arr):
        '''
        Purpose:
        ---
        Calculates the centroid of the shape/object by any two vectors of any
        shape/object
        
        Input Arguments:
        ---
        arr: [[(float)],[(float)]]

        Returns:
        ---
        centroid of the shape(in form of (x,y)): [(float), (float)]

        Example call:
        ---
        self.centroid(vector)
        '''

        length = arr.shape[0]
        sum_x = np.sum(arr[:, 0])
        sum_y = np.sum(arr[:, 1])
        return [sum_x/length, sum_y/length]

    def create_vector(self, point_1, point_2):
        '''
        Purpose:
        ---
        Takes 2 points and determines line vector of the shape
        
        Input Arguments:
        ---
        point_1: [x, y], point_2: [x, y]

        Returns:
        ---
        vector of the points

        Example call:
        ---
        self.create_vector(point_1, point_2) 
        '''
        return np.array(np.array(point_2) - np.array(point_1))

    def unit_vector(self, vector):
        '''
        Purpose:
        ---
        Returns the unit vector of the given vector.
        
        Input Arguments:
        ---
        vector

        Returns:
        ---
        vector

        Example call:
        ---
        self.unit_vector(vector)
        '''
        return vector / np.linalg.norm(vector)
    
    def angle_between(self, vector_1, vector_2):
        '''
        Purpose:
        ---
        It returns the angle in radians between vectors 'v1' and 'v2'
        
        Input Arguments:
        ---
        vector v1 and v2

        Returns:
        ---
        angle in radian

        Example call:
        ---
        self.angle_between(vector1, vector2)
        '''

        v1_u = self.unit_vector(vector_1)
        v2_u = self.unit_vector(vector_2)
        theta = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

        if(vector_1[0]>0):
            return -theta
        return theta

    def side_length(self, point_1, point_2):
        '''
        Purpose:
        ---
        Takes two points and gives length between them
        
        Input Arguments:
        ---
        points point_1 and point_2

        Returns:
        ---
        length

        Example call:
        ---
        self.side_length(point1, point2)
        '''
        return pow(pow(point_1[0]-point_2[0], 2) + pow(point_1[1]-point_2[1], 2), 0.5)

    def callback(self, data):
        '''
        Purpose:
        ---
        It's callback function which collects the information from the topic
        "usb_cam/image_raw" and calculates current location and orientation of
        bot in x, y and theta format
        
        Input Arguments:
        ---
        data :  Image type

        Returns:
        ---
        None

        Example call:
        ---
        Automatically called when the information is being published on the
        topic "/path_plan" and variables
        '''

        # Bridge is Used to Convert ROS Image message to OpenCV image
        br = CvBridge()
        # rospy.loginfo("receiving camera frame")
        get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
        self.current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
        # cv2.imwrite("/home/rishav/out.png", self.current_frame)
        
        # Detecting aruco marker
        (corners, ids, _) = cv2.aruco.detectMarkers(self.current_frame, self.aruco_dict, parameters=self.aruco_params)
        # # marking the detected area
        cv2.aruco.drawDetectedMarkers(self.current_frame, corners, ids)

        # ids, corners = aruco_ids, aruco_corners
        # print(corners)
        
        arena = {"4": [0, 0], "8": [0, 0], "10": [0, 0], "12":[0, 0]}
        length_ids = len(ids)

        cv2.putText(self.current_frame, f"{self.bot_centroid} {math.degrees(self.theta)}",
            (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)

        aruco_bot = np.array([])
        
        # self.draw_path()

        # Camera window
        cv2.imshow("Camera Window", self.current_frame)
        cv2.waitKey(10)  # adding delay

        try:
            # raise exception if any one aruco disappears
            if(length_ids != 5):
                raise Exception("5 Arucos not visible!")

            # # creating arena and bot locations
            for i in range(0, length_ids):
                if(ids[i][0] == 15):
                    aruco_bot = corners[i][0]
                elif(ids[i][0] == 4):
                    arena["4"] = self.centroid(corners[i][0])
                elif(ids[i][0] == 8):
                    arena["8"] = self.centroid(corners[i][0])
                elif(ids[i][0] == 10):
                    arena["10"] = self.centroid(corners[i][0])
                elif(ids[i][0] == 12):
                    arena["12"] = self.centroid(corners[i][0])
                else:
                    raise Exception("Bot not visible!")

            
            # if only bot aruco is visible
            # if(ids[0][0] == 15):
            #     aruco_bot = corners[0][0]
                            

        except Exception as e:
            # rospy.logerr(e)
            return

        # calculating x, y by taking cetroid of the quadilateral
        bot_centroid = np.array(self.centroid(aruco_bot))

        # taking 2 points for determining line vector(box axis)
        v1 = self.create_vector(bot_centroid, aruco_bot[0]/2 + aruco_bot[1]/2) 	#bot axis
        # v2 = np.array([0, -1])										#camera axis
        v2 = self.create_vector(arena["12"], arena["4"])
        # angle between bot axis and camera axis vectors will determine bot orietation
        theta = self.angle_between(v1, v2)
        # print(x, y, theta)
        
        # scaling centroid of bot wrt camera frane and arena frame
        # currently taking aruco id "4" as reference
        bot_centroid = self.create_vector(arena["4"], bot_centroid)
        side_len_x = self.side_length(arena["4"], arena["8"])
        bot_centroid[0] = bot_centroid[0]*500/side_len_x
        side_len_y = self.side_length(arena["4"], arena["12"])
        bot_centroid[1] = bot_centroid[1]*500/side_len_y
        
        # print(bot_centroid[0], bot_centroid[1], theta)
        
        self.bot_centroid[0] = round(bot_centroid[0], 3)
        self.bot_centroid[1] = round(bot_centroid[1], 3)
        self.theta = theta

        self.publish()

        # adding delay
        rospy.sleep(0.001)

    def publish(self):
        '''
        Purpose:
        ---
        Publishes the location(x, y) and oriantation(theta) of the bot in form
        of x, y, theta
        
        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.aruco_publisher.publish()
        '''

        self.aruco_msg.x = self.bot_centroid[0]
        self.aruco_msg.y = self.bot_centroid[1]
        self.aruco_msg.theta = self.theta
        self.aruco_publisher.publish(self.aruco_msg)
        
    def main(self):
        '''
        Purpose:
        ---
        Keeps the node alive
        
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

        rospy.spin()
  
if __name__ == '__main__':
    feedback = Feedback()
    feedback.main()

