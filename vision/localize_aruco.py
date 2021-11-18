import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from vision_utils import *

# Load terrain scene
test_scene = cv.imread("../data/terrain_1.png", cv.IMREAD_GRAYSCALE)
h,w = test_scene.shape
w //= 2
h //= 2

# Resize terrain scene
test_scene = cv.resize(test_scene, (w, h))

# Localize
(c, center, angle) = localize_thymio(test_scene)

# Draw detected 
result = cv.cvtColor(test_scene, cv.COLOR_GRAY2BGR)
for i in range(4):
  cv.line(result, c[i], c[(i+1)%4], (255, 0, 0), 2)

forward = np.array([np.sin(angle)*40, np.cos(angle)*40], dtype=np.int32)
cv.line(result, center, center+forward, (0, 0, 255), 2)

# cv.drawContours(result, [c], 0, (0, 255, 0), -1)

# show result
cv.imshow("hello", result)
cv.imwrite("../data/localize.png", result)
cv.waitKey()
