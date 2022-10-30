#!/usr/bin/env python3


import numpy as np				# If you find it required
import cv2				# OpenCV Library
import math				# If you find it required


ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def aruco_dec(image):
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    print(corners, ids, rejected,'\n')
    cv2.aruco.drawDetectedMarkers(image, corners)
    return image


def main():
    # path = '/home/rishav/Downloads/marker33.webp'
    path = '/home/rishav/output.png'
    image = cv2.imread(path)
    image_resized = cv2.resize(image, (500, 500), interpolation = cv2.INTER_LINEAR)

    for _ in range(3):
        gray = aruco_dec(image_resized)
        cv2.imshow("image", gray)
        cv2.waitKey()

if __name__ == '__main__':
    main()


# #####################################################################################################

# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, self.current_frame.shape[::-1], None, None)
# print(ret, mtx, dist, rvecs, tvecs)

# #####################################################################################################

# projection_matrix = [1171.5121418959693, 0.0, 640.5, -0.0, 0.0, 1171.5121418959693, 640.5, 0.0, 0.0, 0.0, 1.0, 0.0]
# marker_length = 500/21
# camera_matrix = np.array([1171.5121418959693, 0.0, 640.5, 0.0, 1171.5121418959693, 640.5, 0.0, 0.0, 1.0])
# dist_matrix = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
# rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_matrix)
# print(rvec, tvec)
