
import cv2
import numpy as np

def image_mode():
    
    size_img = (500,500)
    # max_points = 20

    # img_path = "/mnt/STORAGE/project/hola_bot/src/task5b/src/"
    img_path = "/mnt/STORAGE/project/hola_bot/src/aruco/eyantra_logo.png"
    # file_name = "smile.png"
    # file_name = "snapchat.png"
    file_name = ""

    img = cv2.imread(img_path+file_name, 0)

    img = cv2.resize(img,size_img)
    black = np.zeros((size_img[0],size_img[1],3),np.uint8)

    # Applying threshold (just in case gray is not binary image).
    _, thresh = cv2.threshold(img, thresh=127, maxval=255, type=cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours,hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    
    xList, yList, wList = [], [], []

    x_goals, y_goals, theta_goals = [], [], []

    for i in range(0,len(contours)):

        xList, yList, wList = [], [], []


        if(hierarchy[0][i][3] != -1):
        # if(True):
            print(len(contours[i]))
            # perimeter = cv2.arcLength(contours[i], closed=True)
            # epsilon = 0.008*perimeter
            epsilon = 0.006*size_img[1]
            contours[i] = cv2.approxPolyDP(contours[i], epsilon, closed=True)
            print(len(contours[i]))

            len_cont = len(contours[i])
            for j in range(0,len_cont, 1):
                black[contours[i][j][0][1]][contours[i][j][0][0]] = 255
                xList.append(contours[i][j][0][0])
                yList.append(contours[i][j][0][1])
                wList.append(0)
                # print(contours[i][j][0])
                # print(contours[i][j][0][1])
        
            x_goals.append(xList)
            y_goals.append(yList)
            theta_goals.append(wList)
    
    for cont in range(len(x_goals)):
        x_goals[cont].append(x_goals[cont][0])
        y_goals[cont].append(y_goals[cont][0])
        theta_goals[cont].append(theta_goals[cont][0])
    
    print(x_goals, len(x_goals))
    # print(y_goals, len(y_goals))
    cv2.imwrite(img_path+"out_def.png",black)

image_mode()