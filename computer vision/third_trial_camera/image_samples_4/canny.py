import numpy as np
import cv2 as cv
# from matplotlib import pyplot as plt

img1 = cv.imread('Image24.jpg',0)
edges1 = cv.Canny(img1,75,125)
cv.imwrite('result_Image24.jpg', edges1)
