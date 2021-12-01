import numpy as np
import cv2 as cv
import cv2.aruco as aruco

from vision_utils import *
import pickle

# 21.9 mm
# cam_int = find_camera_intrinsics("../data/calibrate_camera", 21.9e-3)

cam_int = load_predefined_camera_int("../data/camera.bin")

cam = cv.VideoCapture(0, cv.CAP_DSHOW)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

while True:
  # marker_test = cv.imread("../data/terrain_test.jpg")

  ret_val, marker_test = cam.read()
  marker_test = cv.cvtColor(marker_test, cv.COLOR_BGR2GRAY)

  # f = open("camera.bin", "wb")
  # pickle.dump(cam_int, f)


  # marker_test = cv.resize(marker_test, (w//2, h//2))

  detected = detect_aruco(marker_test)
  (center, c) = get_pos_aruco(detected, 2)


  rvecs, tvecs = estimate_aruco_axis(marker_test, detected, 2, cam_int, marker_length=13e-3)

  imgpts = compute_offset_elevation(cam_int, rvecs, tvecs, 0.1)

  if imgpts is not None:
    cv.drawMarker(marker_test, np.int32(imgpts), (0, 0, 255), markerSize=40, thickness=4)

  h, w = marker_test.shape
  marker_test = cv.resize(marker_test, (w//2, h//2))


  cv.imshow("marker test", marker_test)

  if cv.waitKey(1) == 27: 
    break  # esc to quit

cv.destroyAllWindows()
