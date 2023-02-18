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


aruco_ids = [[10], [12], [15], [8], [4]]

aruco_corners = [
    np.array([[
        [336., 400.],
        [351., 399.],
        [349., 418.],
        [334., 420.]]], ),

    np.array([[
        [90., 381.],
        [104., 384.],
        [105., 403.],
        [91., 401.]]], ),

    np.array([[
        [316., 312.],
        [321., 288.],
        [338., 295.],
        [333., 319.]]], ),

    np.array([[
        [340.,  60.],
        [354.,  64.],
        [354.,  81.],
        [340.,  78.]]], ),

    np.array([[
        [100.,  54.],
        [113.,  52.],
        [111.,  70.],
        [98.,  72.]]], )

]

class Feedback():
    def __init__(self):
        ############################ GLOBALS #############################

        self.aruco_msg = Pose2D()
        self.current_frame = np.empty([])

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

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

    def undistort_img(self, img):
        '''
        camera_matrix:
          rows: 3
          cols: 3
          data: [446.5657371341725, 0, 320.5898932104892, 0, 444.2909319311765, 255.2938346227155, 0, 0, 1]
        distortion_model: plumb_bob
        distortion_coefficients:
          rows: 1
          cols: 5
          data: [-0.3777466493179079, 0.1200715128128287, -0.004146316879421532, -0.003046305956113639, 0]
        rectification_matrix:
          rows: 3
          cols: 3
          data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
        projection_matrix:
          rows: 3
          cols: 4
          data: [338.8027648925781, 0, 314.7816814632406, 0, 0, 386.9388122558594, 257.0193612639923, 0, 0, 0, 1, 0]
        '''
        
        # camera_matrix
        K = np.array([[446.5657371341725, 0, 320.5898932104892], [0, 444.2909319311765, 255.2938346227155], [0, 0, 1]])
        
        # distortion_coefficients
        D = np.array([-0.3777466493179079, 0.1200715128128287, -0.004146316879421532, -0.003046305956113639])
        # D = np.array([0.1200715128128287, -0.004146316879421532, -0.003046305956113639, 0])
        # D = np.array([-0.3777466493179079, 0.1200715128128287, -0.004146316879421532, 0])
        
        # rectification_matrix
        R = np.eye(3)
        
        # projection_matrix
        P = np.array([[338.8027648925781, 0, 314.7816814632406, 0], [0, 386.9388122558594, 257.0193612639923, 0], [0, 0, 1, 0]])
        
        # dimentions
        img_dim = (500, 500)
        
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, R, P, img_dim, cv2.CV_16SC2)
        # print(map1, map2)

        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, img_dim, np.eye(3), balance=balance)
        # map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, img_dim, cv2.CV_16SC2)
        # undist_image = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        return undistorted_img



    def callback(self, data):
        # Bridge is Used to Convert ROS Image message to OpenCV image
        br = CvBridge()
        # rospy.loginfo("receiving camera frame")
        get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
        self.current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
        
        # self.current_frame = self.undistort_img(self.current_frame)
        
        # Detecting aruco marker
        (corners, ids, _) = cv2.aruco.detectMarkers(self.current_frame, self.aruco_dict, parameters=self.aruco_params)
        # # marking the detected area
        cv2.aruco.drawDetectedMarkers(self.current_frame, corners)

        # Camera window
        cv2.imshow("Camera Window", self.current_frame)
        cv2.waitKey(10)  # adding delay
        # ids, corners = aruco_ids, aruco_corners
        # return 
        
        arena = {"4": 0, "8": 0, "10": 0, "12":0}

        # taking the 1st detected aruco marker
        try:
            length_ids = len(ids)
            if(length_ids != 5):
                return

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
            rospy.logerr(e)
            return
        

        print(arena)
        ideal_arena = np.array([[0, 0], [500, 0], [500, 500], [0, 500]])
        
        print(
        	self.create_vector(arena["4"],arena["8"]),
        	self.create_vector(arena["8"],arena["10"]), 
        	self.create_vector(arena["10"],arena["12"]), 
        	self.create_vector(arena["12"],arena["4"])
        	)
        print()
        
        
        # calculating x, y by taking cetroid of the quadilateral
        x, y = self.centroid(aruco_bot)

        # taking 2 points for determining line vector(box axis)
        v1 = self.create_vector(np.array([x, y]), aruco_bot[0]/2 + aruco_bot[1]/2) 	#bot axis
        v2 = np.array([0, -1])										#camera axis
        # angle between bot axis and camera axis vectors will determine bot orietation
        theta = self.angle_between(v1, v2)
        # print(x, y, theta)
            
        self.publish(x, y, theta)

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

