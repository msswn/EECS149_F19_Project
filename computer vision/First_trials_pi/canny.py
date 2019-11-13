import numpy as np
import cv2 as cv
# from matplotlib import pyplot as plt

img1 = cv.imread('simple_thick_pi_model.jpg',0)
edges1 = cv.Canny(img1,100,200)
cv.imwrite('result_simple_thick_pi_model.jpg', edges1)


img2 = cv.imread('simple_thin_pi_model.jpg',0)
edges2 = cv.Canny(img2,100,200)
cv.imwrite('result_simple_thin_pi_model.jpg', edges2)

img3 = cv.imread('3_pis.jpg',0)
edges3 = cv.Canny(img3,100,200)
cv.imwrite('result_3_pis.jpg', edges3)

img4 = cv.imread('smallpi.jpg',0)
edges4 = cv.Canny(img4,100,200)
cv.imwrite('result_smallpi.jpg', edges4)

img5 = cv.imread('spotwithkobuki.jpg',0)
edges5 = cv.Canny(img5,100,200)
cv.imwrite('result_spotwithkobuki.jpg', edges5)
# plt.subplot(121),plt.imshow(img,cmap = 'gray')
# plt.title('Original Image'), plt.xticks([]), plt.yticks([])
# plt.subplot(122),plt.imshow(edges,cmap = 'gray')
# plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
# plt.save("result.jpg")
