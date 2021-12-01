import numpy as np
import cv2 as cv
import cv2.aruco as aruco

from vision_utils import *

# 21.9 mm
cam_int = find_camera_intrinsics("../data/calibrate_camera", 21.9e-3)

marker_test = cv.imread(
estimate
