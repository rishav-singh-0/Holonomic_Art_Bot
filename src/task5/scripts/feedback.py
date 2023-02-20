#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


######################## IMPORT MODULES ##########################

import numpy as np				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation


class Feedback():
    def __init__(self):
        ############################ GLOBALS #############################

        self.aruco_msg = Pose2D()
        self.current_frame = np.empty([])

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        self.bot_centroid = [250, 250]
        self.theta = 0

        #################### ROS Node ############################

        rospy.init_node('aruco_feedback_node')
        # rospy.Subscriber('overhead_cam/image_raw', Image, self.callback)
        rospy.Subscriber('usb_cam/image_raw', Image, self.callback)
        # rospy.Subscriber('usb_cam/image_rect', Image, self.callback)
        self.aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)

    ##################### FUNCTION DEFINITIONS #######################

    def centroid(self, arr):
        length = arr.shape[0]
        sum_x = np.sum(arr[:, 0])
        sum_y = np.sum(arr[:, 1])
        return [sum_x/length, sum_y/length]

    def create_vector(self, point_1, point_2):
        return np.array(np.array(point_2) - np.array(point_1))

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)
    
    def angle_between(self, v1, v2):
        """ 
        Returns the angle in radians between vectors 'v1' and 'v2'
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        theta = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
        # print(v1_u, v2_u, theta)

        if(v1[0]>0):
            return -theta
        return theta

    def side_length(self, p1, p2):
        """
        Takes two points and gives length between them
        """
        return pow(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2), 0.5)

    def callback(self, data):
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

        cv2.putText(self.current_frame, f"{self.bot_centroid} {self.theta}",
            (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)

        # Camera window
        cv2.imshow("Camera Window", self.current_frame)
        cv2.waitKey(10)  # adding delay

        try:
            # raise exception if any one aruco disappears
            if(length_ids != 5):
                raise Exception("5 Arucos not visible!")

            # creating arena and bot locations
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
        self.publish(bot_centroid[0], bot_centroid[1], theta)
        
        self.bot_centroid[0] = round(bot_centroid[0], 3)
        self.bot_centroid[1] = round(bot_centroid[1], 3)
        self.theta = round(theta, 5)

        # adding delay
        rospy.sleep(0.001)

    def publish(self, x, y, theta):
        self.aruco_msg.x = x
        self.aruco_msg.y = y
        self.aruco_msg.theta = theta
        self.aruco_publisher.publish(self.aruco_msg)
        
    def main(self):
        rospy.spin()
  
if __name__ == '__main__':
    feedback = Feedback()
    feedback.main()

