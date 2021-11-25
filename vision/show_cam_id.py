import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from vision_utils import *

# Init ArUco parameters
dict_id = aruco.DICT_6X6_50
arucoDict = aruco.Dictionary_get(dict_id)

# Open Camera
cam = cv.VideoCapture(0, cv.CAP_DSHOW)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

while True:
  ret_val, img = cam.read()

  (corners, ids, rejected) = cv.aruco.detectMarkers(img, arucoDict,
    parameters=arucoParams)

  if len(corners) == 0:
    continue

  for i, id in enumerate(ids):
    c = np.int32(corners[i][0])
    center = (c[0]+c[1]+c[2]+c[3])//4
    for i in range(4):
      cv.line(img, c[i], c[(i+1)%4], (255, 0, 0), 2)
    cv.putText(img, f'id = {id[0]}', center, cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255))

  cv.imshow('my webcam', img)
  if cv.waitKey(1) == 27: 
    break  # esc to quit
cv.destroyAllWindows()
