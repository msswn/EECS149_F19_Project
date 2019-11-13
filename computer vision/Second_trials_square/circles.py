import numpy as np
import cv2 as cv

img = cv.imread('3colorcircle.jpg')
edge = cv.Canny(img,75,150)


circles = cv.HoughCircles(edge,cv.HOUGH_GRADIENT,1,250,param1=100,param2=30,minRadius=250,maxRadius=500)

circles = np.uint16(np.around(circles))

for i in circles[0,:]:
   # draw the outer circle
   cv.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
   # draw the center of the circle
   cv.circle(img,(i[0],i[1]),2,(0,0,255),3)
   print(img[i[0],i[1]])

cv.imwrite("HoughCircles_3colorcircle.jpg", img)
