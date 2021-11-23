import numpy as np
import cv2 as cv
import cv2.aruco as aruco

# ArUco dictionary
dict_id = aruco.DICT_6X6_50
arucoDict = aruco.Dictionary_get(dict_id)
arucoParams = cv.aruco.DetectorParameters_create()

## A0 paper ratio
res_w = 84*2
res_h = 118*2

# Corners dictionary
corner_ids = {
    10: 0,
    8: 1,
    15: 3,
    6: 2,
}

def localize_thymio(img):
  # Detect aruco
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

def detect_aruco(img):
  return cv.aruco.detectMarkers(img, arucoDict,
    parameters=arucoParams)

def get_labyrinth_perspective(img):
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

      if id in corner_ids:
        idx = corner_ids[id]
        cs[idx] = center
        valid_corners += 1

  # If 4 corners, crop
  if valid_corners == 4:

    # Do perspective correction
    pts1 = np.array([cs[0], cs[1], cs[3], cs[2]])
    pts2 = np.float32([[res_h,res_w], [0, res_w], [res_h, 0], [0, 0]])

    M = cv.getPerspectiveTransform(pts1,pts2)

    # for i in range(4):
      # cv.line(img, np.int32(cs[i]), np.int32(cs[(i+1)%4]), (0, 255, 0), 2)
    return M
  else:
    return None

def crop_labyrinth(img, M):
    return cv.warpPerspective(img,M,(res_h,res_w))

def calibrate_corners(cam):
  while True:
    ret_val, img = cam.read()

    M = get_labyrinth_perspective(img)
    if M is not None:
      return M

    cv.putText(img, f'Corners not detected!', (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255))
    cv.imshow('my webcam', img)

    if cv.waitKey(1) == 27: 
      return None

