import numpy as np
import cv2 as cv
import cv2.aruco as aruco

from vision_utils import *

# 21.9 mm
find_camera_intrinsics("../data/calibrate_camera", 21.9e-3)
