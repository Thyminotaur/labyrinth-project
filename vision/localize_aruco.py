import numpy as np
import cv2 as cv
import cv2.aruco as aruco

dict_id = aruco.DICT_6X6_50
id = 1

arucoDict = aruco.Dictionary_get(dict_id)

# Load terrain scene
test_scene = cv.imread("../data/terrain_1.png", cv.IMREAD_GRAYSCALE)
h,w = test_scene.shape
w //= 2
h //= 2

# Resize terrain scene
test_scene = cv.resize(test_scene, (w, h))

# Detect aruco
arucoParams = cv.aruco.DetectorParameters_create()

(corners, ids, rejected) = cv.aruco.detectMarkers(test_scene, arucoDict,
	parameters=arucoParams)

# Draw detected 
result = cv.cvtColor(test_scene, cv.COLOR_GRAY2BGR)
c = np.int32(corners[0][0])
for i in range(4):
  cv.line(result, c[i], c[(i+1)%4], (255, 0, 0), 2)

# c[0]        TOP LEFT
# c[1]        BOTTOM RIGHT
# c[2]        BOTTOM LEFT
# c[3]        TOP LEFT
top_middle = (c[0]+c[3])//2
center = (c[0]+c[1]+c[2]+c[3])//4

cv.line(result, center, top_middle, (0, 0, 255), 2)

cv.imshow("hello", result)
cv.imwrite("../data/localize.png", result)
cv.waitKey()
