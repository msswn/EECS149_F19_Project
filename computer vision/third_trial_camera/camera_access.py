import numpy as np
import cv2 as cv
# from matplotlib import pyplot as plt


camera = cv.VideoCapture(1)
images_taken = 0

while images_taken < 10:
	return_value, image = camera.read()
	if return_value:
		cv.imwrite('picture{0}.jpg'.format(images_taken), image)
		images_taken = images_taken + 1
	else: 
		print("Error! Image not captured")
