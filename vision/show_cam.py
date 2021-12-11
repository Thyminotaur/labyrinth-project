import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from vision_utils import *

cam = cv.VideoCapture(0, cv.CAP_DSHOW)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
while True:
  ret_val, img = cam.read()
  # (c, center, angle) = localize_thymio(img)
  # if c is not None:
  #   for i in range(4):
  #     cv.line(img, c[i], c[(i+1)%4], (255, 0, 0), 2)
  cv.imshow('my webcam', img)
  if cv.waitKey(1) == 27: 
    break  # esc to quit
cv.destroyAllWindows()
