import numpy as np
import cv2 as cv
import cv2.aruco as aruco

from vision_utils import *
import pickle

# 21.9 mm
# cam_int = find_camera_intrinsics("../data/calibrate_camera", 21.9e-3)

cam_int = load_predefined_camera_int("../data/camera.bin")
marker_test = cv.imread("../data/terrain_test.jpg")

# f = open("camera.bin", "wb")
# pickle.dump(cam_int, f)


# marker_test = cv.resize(marker_test, (w//2, h//2))

detected = detect_aruco(marker_test)
(center, c) = get_pos_aruco(detected, 2)

rvecs, tvecs = estimate_aruco_axis(marker_test, detected, 2, cam_int)

# [[[ 2.53948468  0.98920067 -0.11648686]]]
# [[[ 0.0101194  -0.0045403   0.14878134]]]
imgpts = compute_offset_elevation(cam_int, rvecs, tvecs, 0.1)

cv.drawMarker(marker_test, np.int32(imgpts), (0, 0, 255), markerSize=40, thickness=4)

h, w, _ = marker_test.shape
marker_test = cv.resize(marker_test, (w//2, h//2))


cv.imshow("marker test", marker_test)
cv.waitKey()
