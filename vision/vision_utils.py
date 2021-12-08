import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import glob
import pickle
from scipy.interpolate import LinearNDInterpolator

# ArUco dictionary
dict_id = aruco.DICT_6X6_50
arucoDict = aruco.Dictionary_get(dict_id)
arucoParams = cv.aruco.DetectorParameters_create()

## A0 paper ratio
res_w = 720
res_h = 1020

# Corners dictionary
corner_ids = {
    10: 3,
    8: 2,
    15: 0,
    6: 1,
}

# Ids
thymio_id = 2

# Thymio offset
offset_interp = None

# Vision thresholds
WALL_THRESHOLD = 5000

# Detect all AruCo in image
def detect_aruco(img):
  detected = cv.aruco.detectMarkers(img, arucoDict,
    parameters=arucoParams)
  return detected 

# Get position of ArUco marker
def get_pos_aruco(detected, search_id):
  (corners, ids, rejected) = detected

  if ids is not None:
    for i, id in enumerate(ids):
      c = corners[i][0]
      if id[0] == search_id:
        center = (c[0]+c[1]+c[2]+c[3])/4
        return (center, c)
  return (None, None)

# Given the ID, gives the position, angle of the Thymio
# assuming an ArUco is put on top of it
def localize_thymio(detected):
  # Detect aruco
  (center, c) = get_pos_aruco(detected, thymio_id)

  if center is None:
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
      c = np.int32(corners[i][0])

      # Remove AruCo
      cv.drawContours(img, [c], 0, 255, -1)


# Gives a binary grid representation of 
# the labyrinth
# img must be grayscale
def detect_labyrinth(img):
  h,w = img.shape

  # Remove AruCo
  detected = detect_aruco(img)
  erase_aruco(img, detected)

  # Threshold
  th = cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV,31,2)
  
  cv.imwrite("global_trajectory_real_resized_th.png", th)

  # Remove noise
  kernel = np.ones((10,10),np.uint8)
  th = cv.morphologyEx(th, cv.MORPH_OPEN, kernel)

  # Detect connected components
  num_labels, labels, stats, _ = cv.connectedComponentsWithStats(th, 8, cv.CV_32S)

  # Pick wall components
  result = np.zeros_like(th)
  for i in range(1, num_labels):
    if stats[i, cv.CC_STAT_AREA] > WALL_THRESHOLD:
      result[labels == i] = 255

  cv.imwrite("global_trajectory_real_resized_test.png", th)

  # Dilate along both x and y direction
  kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(20, 20))
  result = cv.dilate(result,kernel,iterations = 1)
  result = dilate_walls_max(result, [5, 5], [10, 10], [20, 20])

  result = dilate_walls_max(result, [5, 5],[10, 0], [20, 20])
  result = dilate_walls_max(result, [5, 5], [0, 10], [20, 20])

  return result

# Dilate until different number of components
def dilate_walls_max(img, init_wall_size, wall_inc, wall_margin):
  wall_size = init_wall_size

  num_components_previous, _, _, _ = cv.connectedComponentsWithStats(img, 8, cv.CV_32S)

  # Check that the dilate at first doesn't affect the connected components
  # count
  kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(wall_size[0]+wall_margin[0], wall_size[1]+wall_margin[1]))
  dilated = cv.dilate(img,kernel,iterations = 1)

  num_labels, _, _, _ = cv.connectedComponentsWithStats(dilated, 8, cv.CV_32S)

  if num_labels < num_components_previous:
    return img

  # Dilate by increment and check that the number of 
  # connected components stay the same
  while True:
    wall_size[0] += wall_inc[0]
    wall_size[1] += wall_inc[1]

    kernel =cv.getStructuringElement(cv.MORPH_ELLIPSE,(wall_size[0]+wall_margin[0], wall_size[1]+wall_margin[1]))
    dilated = cv.dilate(img,kernel,iterations = 1)

    num_labels, _, _, _ = cv.connectedComponentsWithStats(dilated, 8, cv.CV_32S)

    if num_components_previous > num_labels:
      wall_size[0] -= wall_inc[0]
      wall_size[1] -= wall_inc[1]

      kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,wall_size)
      return cv.dilate(img,kernel,iterations = 1)

    num_components_previous = num_labels

# Get perspective transform from img from ArUco corners
# Returns none if corners are not detected
def get_labyrinth_perspective(img):
  detected = detect_aruco(img)

  cs = [None]*4
  for (id, idx) in corner_ids.items():
    (center, _) = get_pos_aruco(detected, id)
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

    if img is None:
      continue

    M = get_labyrinth_perspective(img)
    if M is not None:
      return M

    cv.putText(img, f'Corners not detected!', (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255))
    cv.imshow('my webcam', img)

    if cv.waitKey(1) == 27: 
      return None

# Do adaptive thresholding for weird images
def do_adaptive_threshold(img):
  th = cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY,31,2)
  return th

def draw_aruco(img, corners):
    c = np.int32(corners)
    for i in range(4):
      cv.line(img, c[i], c[(i+1)%4], (255, 0, 0), 2)

def transform_perspective_point(M, p):
  px = (M[0,0]*p[0] + M[0,1]*p[1] + M[0,2]) / ((M[2,0]*p[0] + M[2,1]*p[1] + M[2,2]))
  py = (M[1,0]*p[0] + M[1,1]*p[1] + M[1,2]) / ((M[2,0]*p[0] + M[2,1]*p[1] + M[2,2]))
  return (px, py)

def load_z_offset_data(path):
  global offset_interp

  f = open(path, "rb")
  saved = pickle.load(f)

  (thymio_pos, offset_pos) = saved

  offset_interp = LinearNDInterpolator(thymio_pos, offset_pos, 0)


def get_z_offset(center):
  if offset_interp is not None:
    return offset_interp(center[0], center[1])
  else:
    print("No interpolator for z offset (please use load_z_offset_data() first)")
    return [0, 0]
