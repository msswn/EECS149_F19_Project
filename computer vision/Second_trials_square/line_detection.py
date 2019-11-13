import numpy as np
import cv2 as cv

# First use Canny Edge Detection to find all the edges on the graph
img1 = cv.imread('square.jpg',0)
edge1 = cv.Canny(img1,75,150)

# Use Hough Line Transform to find line segments of the edges
lineslist = cv.HoughLinesP(edge1,0.1,np.pi/10,30,20,10)
print(lineslist)
xsum = 0
ysum = 0
c = 0
for line in lineslist:
    for x1,y1,x2,y2 in line:
        xsum += x1 + x2
        ysum += y1 + y2
        c += 1
        cv.line(edge1,(x1,y1),(x2,y2),(255,255,255),25)
xm = xsum/(2*c)
ym = ysum/(2*c)
print(xm, ym)
cv.circle(edge1, (xm, ym), 10, (255,255,255))
cv.imwrite('hough_square.jpg',edge1)
