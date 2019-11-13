import numpy as np
import cv2 as cv
import array
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contours_hierarchy/py_contours_hierarchy.html

img = cv.imread('square_twocars.jpg')

##
##circles
##
edge = cv.Canny(img,75,150)
circles = cv.HoughCircles(edge,cv.HOUGH_GRADIENT,1,50,param1=100,param2=30,minRadius=125,maxRadius=200)
circles = np.uint16(np.around(circles))

for i in circles[0,:]:
   # draw the outer circle
   cv.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
   # draw the center of the circle
   cv.circle(img,(i[0],i[1]),2,(0,0,255),3)

print(circles[0,:])

##
##squares
##

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

ret,thresh = cv.threshold(gray,127,255,1)

contours,h = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
Squarelist = []

for cnt in contours:
    approx = cv.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True)
    if len(approx)==4:

        M = cv.moments(cnt)
        if (M['m00'] > 1000 and M['m00'] < 1000000):
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            Squarelist.append([M['m00'], cx, cy])

            cv.drawContours(img,[cnt],0,(0,0,255),-1)
            print("Shape: Square, Area: %f, Centroid:(%f, %f)" %(M['m00'], cx, cy))
    # elif len(approx)==5:
    #     print "pentagon"
    #     cv.drawContours(img,[cnt],0,255,-1)
    # elif len(approx)==3:
    #     print "triangle"
    #     cv.drawContours(img,[cnt],0,(0,255,0),-1)
    # elif len(approx) == 9:
    #     print "half-circle"
    #     cv.drawContours(img,[cnt],0,(255,255,0),-1)

    # elif len(approx) > 15:
    #     M = cv.moments(cnt)
    #     if (M['m00'] > 100):
    #         cx = int(M['m10']/M['m00'])
    #         cy = int(M['m01']/M['m00'])
    #
    #         cv.drawContours(img,[cnt],0,(0,0,255),-1)
    #         print("Shape: Circle, Area: %f, Centroid:(%f, %f)" %(M['m00'], cx, cy))

##
##Still needs ti be fixed whether the square will be detected once or twice?
##
Square = []
for i in range(len(Squarelist)):
    for j in range(i+1, len(Squarelist)):
        a1, x1, y1 = Squarelist[i]
        a2, x2, y2 = Squarelist[j]
        ad = np.abs(a1-a2)/a2
        cd = np.sqrt((x1-x2)**2 + (y1 -y2)**2)
        if (ad < 0.21 and cd < 10):
            ax = (x1 + x2)/2
            ay = (y1 + y2)/2
            cv.circle(img, (ax, ay), 20, (255,255,255))
            Square.append([ax, ay, x1, y1, x2, y2])
print(Square)




cv.imwrite('detection_square_twocars.jpg',img)
