import numpy as np
import cv2 as cv
import array
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contours_hierarchy/py_contours_hierarchy.html

img = cv.imread('Image24.jpg')
edges1 = cv.Canny(img,75,125)

##
##circles
##
edge = cv.Canny(img,75,100)
circles = cv.HoughCircles(edge,cv.HOUGH_GRADIENT,1,50,param1=150,param2=30,minRadius=20,maxRadius=50)
circles = np.uint16(np.around(circles))

circles_found = [];
for i in circles[0,:]:
   # draw the outer circle
   cv.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
   # draw the center of the circle
   cv.circle(img,(i[0],i[1]),2,(0,0,255),3)

print(circles[0,:])

##
##squares
##


#


contours,h = cv.findContours(edges1, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
edgecolor = [235, 220, 212]
Squarelist = []

for cnt in contours:
    approx = cv.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True)
    if len(approx)==4:

        M = cv.moments(cnt)
        if (M['m00'] > 10000 and M['m00'] < 25000):
            countedge = 0
            a, b, c, d = approx
            e1x, e1y = (a[0][0] + b[0][0])/2, (a[0][1] + b[0][1])/2
            e2x, e2y = (b[0][0] + c[0][0])/2, (b[0][1] + c[0][1])/2
            e3x, e3y = (c[0][0] + d[0][0])/2, (c[0][1] + d[0][1])/2
            e4x, e4y = (d[0][0] + a[0][0])/2, (d[0][1] + a[0][1])/2
            e1c = img[e1y, e1x]
            e2c = img[e2y, e2x]
            e3c = img[e3y, e3x]
            e4c = img[e4y, e4x]
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if(np.sqrt((e1c[0]-edgecolor[0])**2 + (e1c[1]-edgecolor[1])**2 + (e1c[2]-edgecolor[2])**2) < 100):
                countedge += 1
            if(np.sqrt((e2c[0]-edgecolor[0])**2 + (e2c[1]-edgecolor[1])**2 + (e2c[2]-edgecolor[2])**2) < 100):
                countedge += 1
            if(np.sqrt((e3c[0]-edgecolor[0])**2 + (e3c[1]-edgecolor[1])**2 + (e3c[2]-edgecolor[2])**2) < 100):
                countedge += 1
            if(np.sqrt((e4c[0]-edgecolor[0])**2 + (e4c[1]-edgecolor[1])**2 + (e4c[2]-edgecolor[2])**2) < 100):
                countedge += 1
            if (countedge >= 2):
                Squarelist.append([M['m00'], [a, b, c, d], [cx, cy], [e1x, e1y, e1c], [e2x, e2y, e2c], [e3x, e3y, e3c], [e4x, e4y, e4c], [cnt]])

            cv.circle(img, (cx, cy), 20, (255,255,255))
            print("Shape: Square, Area: %f, Centroid:(%f, %f)" %(M['m00'], cx, cy))

for i in range(len(Squarelist)):
    print(Squarelist[i][0:7])
    cv.drawContours(img,Squarelist[i][-1],0,(0,0,255),-1)


cv.imwrite('detection_Image24.jpg',img)
