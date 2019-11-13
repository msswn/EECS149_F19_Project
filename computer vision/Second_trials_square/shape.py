import numpy as np
import cv2 as cv
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contours_hierarchy/py_contours_hierarchy.html

img = cv.imread('square.jpg')
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

ret,thresh = cv.threshold(gray,127,255,1)

contours,h = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

for cnt in contours:
    approx = cv.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True)
    if len(approx)==4:

        M = cv.moments(cnt)
        if (M['m00'] > 10):
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            print "square"
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
    #     if (M['m00'] > 10):
    #         cx = int(M['m10']/M['m00'])
    #         cy = int(M['m01']/M['m00'])
    #
    #         print "square"
    #         cv.drawContours(img,[cnt],0,(0,0,255),-1)
    #         print("Shape: Circle, Area: %f, Centroid:(%f, %f)" %(M['m00'], cx, cy))
    #


cv.imwrite('shapedetection_square.jpg',img)
