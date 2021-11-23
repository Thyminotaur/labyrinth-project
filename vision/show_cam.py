import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from vision_utils import *

cam = cv.VideoCapture(0)

while True:
  ret_val, img = cam.read()
  gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
  (c, center, angle) = localize_thymio(gray)
  if c is not None:
    for i in range(4):
      cv.line(img, c[i], c[(i+1)%4], (255, 0, 0), 2)
  cv.imshow('my webcam', img)
  if cv.waitKey(1) == 27: 
    break  # esc to quit
cv.destroyAllWindows()
