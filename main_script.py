import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from vision.vision_utils import *

# Init ArUco parameters
dict_id = aruco.DICT_6X6_50
arucoDict = aruco.Dictionary_get(dict_id)
arucoParams = cv.aruco.DetectorParameters_create()

# Open Camera
cam = cv.VideoCapture(0, cv.CAP_DSHOW)

cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

M = None
while True:
  if M is None:
    M = calibrate_corners(cam)
    if M is None:
      break

  ret_val, img = cam.read()

  dst = crop_labyrinth(img, M)

  # laby = detect_labyrinth(dst, 100)
  detected = detect_aruco(dst)
  (_, center, angle) = localize_thymio(dst, detected)

  if center is not None:
    cv.drawMarker(dst, np.int32(center), (0, 0, 255))
    forward = np.array([np.sin(angle)*40, np.cos(angle)*40], dtype=np.int32)
    cv.line(dst, np.int32(center), np.int32(center+forward), (255, 0, 0), 2)

  dst_gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
  laby = detect_labyrinth(dst_gray, 130)

  # cv.imshow('my webcam', img)
  cv.imshow('transformed', dst)
  cv.imshow('labyrinth', laby)

  key = cv.waitKey(1)
  if key == ord('s'):
    cv.imwrite('data/big_labyrinth.png', laby)

  if key == ord('c'):
    M = None
  if key == 27: 
    break  # esc to quit

cv.destroyAllWindows()
print("Done!")
