import numpy as np
import cv2 as cv
# from matplotlib import pyplot as plt

img1 = cv.imread('square.jpg',0)
edges1 = cv.Canny(img1,75,125)
cv.imwrite('result_square.jpg', edges1)


img2 = cv.imread('3colorcircle.jpg',0)
edges2 = cv.Canny(img2,100,200)
cv.imwrite('result_3colorcircle.jpg', edges2)

img3 = cv.imread('square_twocars.jpg',0)
edges3 = cv.Canny(img3,100,200)
cv.imwrite('result_square_twocars.jpg', edges3)
