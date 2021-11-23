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

# Corners dictionary
corner_ids = {
    10: 0,
    8: 1,
    15: 3,
    6: 2,
}

res_w = 84*2
res_h = 118*2
dst = np.zeros((res_w, res_h))

while True:
  ret_val, img = cam.read()

  gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
  (corners, ids, rejected) = cv.aruco.detectMarkers(img, arucoDict,
    parameters=arucoParams)

  valid_corners = 0
  cs = [None]*4

  # Get corners positions
  if ids is not None:
    for i, id in enumerate(ids):
      c = corners[i][0]
      id = id[0]
      center = (c[0]+c[1]+c[2]+c[3])/4
      # for i in range(4):
        # cv.line(img, np.int32(c[i]), np.int32(c[(i+1)%4]), (255, 0, 0), 2)
      # cv.putText(img, f'id = {id}', np.int32(center), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255))

      if id in corner_ids:
        idx = corner_ids[id]
        cs[idx] = center
        valid_corners += 1

  # If 4 corners, draw quad
  if valid_corners == 4:

    # Do perspective correction
    pts1 = np.array([cs[0], cs[1], cs[3], cs[2]])
    pts2 = np.float32([[res_h,res_w], [0, res_w], [res_h, 0], [0, 0]])

    M = cv.getPerspectiveTransform(pts1,pts2)
    dst = cv.warpPerspective(img,M,(res_h,res_w))

    for i in range(4):
      cv.line(img, np.int32(cs[i]), np.int32(cs[(i+1)%4]), (0, 255, 0), 2)

  cv.imshow('my webcam', img)
  cv.imshow('transformed', dst)

  key = cv.waitKey(1)
  if key == 27: 
    break  # esc to quit

  if key == ord('s'): 
    cv.imwrite("../data/aruco_corners.png", img)
    cv.imwrite("../data/aruco_corners_crop.png", dst)
    break  # esc to quit
cv.destroyAllWindows()
print("Done!")
