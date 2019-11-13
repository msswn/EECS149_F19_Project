import numpy as np
import cv2 as cv

img = cv.imread('square_twocars.jpg')
edge = cv.Canny(img,75,150)


circles = cv.HoughCircles(edge,cv.HOUGH_GRADIENT,1,50,param1=100,param2=30,minRadius=125,maxRadius=200)

circles = np.uint16(np.around(circles))

for i in circles[0,:]:
   # draw the outer circle
   cv.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
   # draw the center of the circle
   cv.circle(img,(i[0],i[1]),2,(0,0,255),3)

print(circles[0,:])

cv.imwrite("HoughCircles_square_twocars.jpg", img)
