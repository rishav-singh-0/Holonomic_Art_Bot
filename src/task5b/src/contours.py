import cv2
import numpy as np

size_img = (500,500)
max_points = 20

def findcont(img): 

    _, thresh = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    return contours,hierarchy

#-------------------------------------Load Image ---------------------------------------
img = cv2.imread('/mnt/STORAGE/project/hola_bot/src/task5b/src/snapchat.png',0)
# img = cv2.imread('smile.png',0)

# cv2.imshow("Image", img)
# print(img)

# img = cv2.imread('snapchat.png',0)
img = cv2.resize(img,size_img)
black = np.zeros((size_img[0],size_img[1],3),np.uint8)


contours,hierarchy = findcont(img)                              # calling fun to get hierarchy and contour  
xList,yList,xListFinal,yListFinal = [],[],[],[]

for i in range(0,len(contours)):

    xList.clear()
    yList.clear()

    len_cont = len(contours[i])

    for j in range(0,len_cont, 10):
        if(hierarchy[0][i][3] != -1):
            black[contours[i][j][0][1]][contours[i][j][0][0]] = 255
            xList.append(contours[i][j][0][0])
            yList.append(contours[i][j][0][1])
    
    xListFinal.append(xList)
    yListFinal.append(yList)

print(xListFinal)
cv2.imwrite("out_def.png",black)