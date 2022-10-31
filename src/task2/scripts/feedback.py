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

		self.aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)
		self.aruco_msg = Pose2D()
		self.current_frame = np.empty([])

		self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		self.aruco_params = cv2.aruco.DetectorParameters_create()

		#################### ROS Node ############################

		rospy.init_node('aruco_feedback_node')
		rospy.Subscriber('overhead_cam/image_raw', Image, self.callback)

	##################### FUNCTION DEFINITIONS #######################

	def create_vector(self, point_1, point_2):
		return np.array(point_2 - point_1)

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

		if(v1[1]>0):
			return -theta
		return theta

	def callback(self, data):
		# Bridge is Used to Convert ROS Image message to OpenCV image
		br = CvBridge()
		# rospy.loginfo("receiving camera frame")
		get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
		self.current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)

		# Detecting aruco marker
		(corners, ids, _) = cv2.aruco.detectMarkers(self.current_frame, self.aruco_dict, parameters=self.aruco_params)
		# marking the detected area
		cv2.aruco.drawDetectedMarkers(self.current_frame, corners)

		# taking the 1st detected aruco marker
		ar = corners[0][0]

		# for now calculating x and y by halfing 2 adjacent sides
		x = int(np.average([ar[0][0], ar[1][0]]))
		y = int(np.average([ar[0][1], ar[3][1]]))

		# taking 2 points for determining line vector(box axis)
		v1 = self.create_vector(ar[0], ar[1]) 	#bot axis
		v2 = np.array([1, 0])					#camera axis
		# angle between bot axis and camera axis vectors will determine bot orietation
		theta = self.angle_between(v1, v2)
		print(x, y, theta)
			
		self.publish(x, y, theta)
		
		# cv2.imshow("Camera Window", self.current_frame)
		# cv2.waitKey(200)

		# adding delay
		rospy.sleep(0.01)

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
