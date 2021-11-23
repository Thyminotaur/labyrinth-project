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

# Ids
thymio_id = 2

# Detect all AruCo in image
def detect_aruco(img):
  detected = cv.aruco.detectMarkers(img, arucoDict,
    parameters=arucoParams)
  return detected 

# Get position of ArUco marker
def get_pos_aruco(img, detected, search_id):
  (corners, ids, rejected) = detected

  if ids is not None:
    for i, id in enumerate(ids):
      c = corners[i][0]
      if id[0] == search_id:
        center = (c[0]+c[1]+c[2]+c[3])/4
        return center, c
  return (None, None)

# Given the ID, gives the position, angle of the Thymio
# assuming an ArUco is put on top of it
def localize_thymio(img, detected):
  # Detect aruco
  (center, c) = get_pos_aruco(img, detected, thymio_id)

  if not center:
    return (None, None, None)

  # c[0]        TOP LEFT
  # c[1]        BOTTOM RIGHT
  # c[2]        BOTTOM LEFT
  # c[3]        TOP LEFT
  # Compute orientation
  top_middle = (c[0]+c[3])/2
  dir = top_middle - center
  angle = np.arctan2(dir[0], dir[1])

  return (c, center, angle)

# Replace all detected AruCo markers with
# white square.
# img must be grayscale.
def erase_aruco(img, detected):
  (corners, _, _) = detected

  if corners is not None:
    for i in range(len(corners)):
      c = corners[i][0]

      # Remove AruCo
      cv.drawContours(img, [c], 0, 255, -1)


# Gives a binary grid representation of 
# the labyrinth
# img must be grayscale
def detect_labyrinth(img, wall_size):
  h,w = img.shape

  # Remove AruCo
  detected = detect_aruco(img)
  erase_aruco(img, detected)

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

  return th

# Get perspective transform from img from ArUco corners
# Returns none if corners are not detected
def get_labyrinth_perspective(img):
  detected = detect_aruco(img)

  cs = [None]*4
  for (id, idx) in corner_ids.items():
    (center, _) = get_pos_aruco(img, detected, id)
    if center is not None:
      cs[idx] = center
    else:
      return None

  # Do perspective correction
  pts1 = np.array([cs[0], cs[1], cs[3], cs[2]])
  pts2 = np.float32([[res_h,res_w], [0, res_w], [res_h, 0], [0, 0]])

  return cv.getPerspectiveTransform(pts1,pts2)

# Given perspective transform, crops the original image
def crop_labyrinth(img, M):
    return cv.warpPerspective(img,M,(res_h,res_w))

# Utility function to wait for corner calibration
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
