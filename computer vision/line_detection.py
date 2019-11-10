import numpy as np
import cv2 as cv

# First use Canny Edge Detection to find all the edges on the graph
img1 = cv.imread('simple_thick_pi_model.jpg',0)
img1 = cv.Canny(img1,100,200)

# Use Hough Line Transform to find line segments of the edges
lineslist = cv.HoughLinesP(img1,0.1,np.pi/10,15,9,4)
xsum = 0
ysum = 0
c = 0
for line in lineslist:
    for x1,y1,x2,y2 in line:
        xsum += x1 + x2
        ysum += y1 + y2
        c += 1
        cv.line(img1,(x1,y1),(x2,y2),(255,255,255),20)
xm = xsum/(2*c)
ym = ysum/(2*c)
cv.circle(img1, (xm, ym), 10, (255,255,255))
cv.imwrite('hough_simple_thick_pi_model.jpg',img1)
