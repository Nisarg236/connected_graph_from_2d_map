import cv2
import numpy as np
from skimage import morphology
bw_map = cv2.imread("map_pringle_dilate.png")
# kernel = np.ones((20, 20), np.uint8)

gray = cv2.cvtColor(bw_map, cv2.COLOR_BGR2GRAY)
im = cv2.threshold(bw_map, 100, 255, cv2.THRESH_BINARY)[1]#if roads white then use binary_inv, if roads black use binary
im=255-im

# im = cv2.erode(im, kernel, iterations=1)
# im = cv2.dilate(im, kernel, iterations=1)

# showim(im)
# kernel = np.asanyarray([[1,1,1], [1,1,1], [1,1,1]], np.float32)/9.0
# im = cv2.filter2D(im, -1, kernel)
# showim(im)
# im = cv2.ximgproc.thinning(gray, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

im = morphology.skeletonize(im > 0)
im = im.astype(np.uint8)
# im=np.vstack([im,im,im])
im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY )

# gray = cv2.bitwise_and(bw_map, bw_map, mask = im)

# im = cv2.threshold(im, 100, 255, cv2.THRESH_BINARY)[1]
# detect corners with the goodFeaturesToTrack function.
corners = cv2.goodFeaturesToTrack(im, 1000, 0.375, 50)
# showim(im)
corners = np.int0(corners)
  
# we iterate through each corner, 
# making a circle at each point that we think is a corner.
for i in corners:
    x, y = i.ravel()
    cv2.circle(bw_map, (x, y), 30, (0,0,255), -1)
print(len(corners))

cv2.imwrite("123.png",im)