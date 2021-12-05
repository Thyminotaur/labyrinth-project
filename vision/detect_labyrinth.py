import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from vision_utils import *
import time

# Load terrain scene
test_scene = cv.imread("../global_trajectory_real_resized_th.png", cv.IMREAD_GRAYSCALE)
h,w = test_scene.shape
# w //= 2
# h //= 2

print("hello world")

# Resize terrain scene
# test_scene = cv.resize(test_scene, (w, h))

result = detect_labyrinth(test_scene)

result = cv.cvtColor(result, cv.COLOR_GRAY2BGR)

# result[center[1], center[0]] = (0, 0, 255)

# total_t = stop - start
# print(f'It took {int(total_t*1000.0)} ms')

# show result
cv.imshow("scene", test_scene)
cv.imshow("hello", result)
cv.imwrite("../data/detect_labyrinth.png", result)
cv.waitKey()
