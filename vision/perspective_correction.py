import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from vision_utils import *

# Init ArUco parameters
dict_id = aruco.DICT_6X6_50
arucoDict = aruco.Dictionary_get(dict_id)
arucoParams = cv.aruco.DetectorParameters_create()

# Open Camera
cam = cv.VideoCapture(0, cv.CAP_DSHOW)

M = None
while True:
  if M is None:
    M = calibrate_corners(cam)
    if M is None:
      break

  ret_val, img = cam.read()

  dst = crop_labyrinth(img, M)

  cv.imshow('my webcam', img)
  cv.imshow('transformed', dst)

  key = cv.waitKey(1)
  if key == ord('c'):
    M = None
  if key == 27: 
    break  # esc to quit

cv.destroyAllWindows()
print("Done!")
