import numpy as np
import cv2 as cv
import cv2.aruco as aruco

dict_id = aruco.DICT_6X6_50
arucoDict = aruco.Dictionary_get(dict_id)

def localize_thymio(img):
  # Detect aruco
  arucoParams = cv.aruco.DetectorParameters_create()

  (corners, ids, rejected) = cv.aruco.detectMarkers(img, arucoDict,
    parameters=arucoParams)


  if len(corners) == 1:
    c = np.int32(corners[0][0])
  else:
    return (None, None, None)


  # Compute center
  center = (c[0]+c[1]+c[2]+c[3])//4

  # c[0]        TOP LEFT
  # c[1]        BOTTOM RIGHT
  # c[2]        BOTTOM LEFT
  # c[3]        TOP LEFT
  # Compute orientation
  top_middle = (c[0]+c[3])//2
  dir = top_middle - center
  angle = np.arctan2(dir[0], dir[1])

  return (c, center, angle)


def detect_labyrinth(img, wall_size):
  h,w = img.shape

  # Localize
  (c, center, angle) = localize_thymio(img)

  # Remove AruCo
  cv.drawContours(img, [c], 0, 255, -1)

  # Threshold
  _, th = cv.threshold(img,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)

  # Remove noise
  kernel = np.ones((5,5),np.uint8)
  th = cv.morphologyEx(th, cv.MORPH_OPEN, kernel)

  # Dilate walls
  kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(wall_size,wall_size))

  th = cv.dilate(th,kernel,iterations = 1)

  # Extract grid
  desired_w = 100
  desired_h = (desired_w * h) // w

  # Extract grid cells
  th = cv.resize(th, (desired_w, desired_h))
  _, th = cv.threshold(th,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)

  center[0] = int(np.round(center[0]*desired_w/w))
  center[1] = int(np.round(center[1]*desired_h/h))

  return th, c, center, angle
