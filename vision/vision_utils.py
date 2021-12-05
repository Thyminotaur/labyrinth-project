import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import glob
import pickle

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
def localize_thymio(img, detected):
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
def detect_labyrinth(img, wall_size):
  h,w = img.shape

  # Remove AruCo
  detected = detect_aruco(img)
  erase_aruco(img, detected)


  # Threshold
  #_, th = cv.threshold(img,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
  th = cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV,31,2)

  cv.imwrite("global_trajectory_real_resized_th.png", th)

  # Remove noise
  kernel = np.ones((10,10),np.uint8)
  kernel2 = np.ones((25,25),np.uint8)
  th = cv.morphologyEx(th, cv.MORPH_OPEN, kernel)
  th = cv.dilate(th, kernel2, iterations=1)

  # Detect connected components
  num_labels, labels, stats, _ = cv.connectedComponentsWithStats(th, 8, cv.CV_32S)
  sort_ind = np.argsort(stats[:, cv.CC_STAT_AREA])

  # Pick two biggest components
  # -1 is all the components
  result = np.zeros_like(th)
  result[labels == sort_ind[-2]] = 255
  result[labels == sort_ind[-3]] = 255

  # temp code
  # result = th
  cv.imwrite("global_trajectory_real_resized_test.png", th)

  # Dilate walls
  kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,wall_size)
  result = cv.dilate(result,kernel,iterations = 1)

  return result

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

# Find camera intrinsics
def find_camera_intrinsics(folder_path, square_width, cx=9, cy=6):
  # termination criteria
  criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

  # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
  objp = np.zeros((cx*cy,3), np.float32)
  objp[:,:2] = np.mgrid[0:cx,0:cy].T.reshape(-1,2)

  # Arrays to store object points and image points from all the images.
  objpoints = [] # 3d point in real world space
  imgpoints = [] # 2d points in image plane.

  images = glob.glob(f'{folder_path}/*.jpg')

  for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (cx,cy),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
      objpoints.append(objp)

      corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
      imgpoints.append(corners2)

      # Draw and display the corners
      # img = cv.drawChessboardCorners(img, (cx,cy), corners2,ret)
      # cv.imshow('img',img)
      # cv.waitKey(500)

  ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
  cv.destroyAllWindows()

  cam_int = (mtx, dist, rvecs, tvecs)

  return  cam_int

def draw_aruco(img, corners):
    c = np.int32(corners)
    for i in range(4):
      cv.line(img, c[i], c[(i+1)%4], (255, 0, 0), 2)

def estimate_aruco_axis(img, detected, aruco_id, cam_int, marker_length=6e-3):
  (mtx, dist, rvecs, tvecs) = cam_int

  (corners, ids, rejected) = detected
  
  if ids is None:
    return None, None
  
  all_corners = [corners[i][0] for i, id in enumerate(ids) if id == thymio_id]

  thymio_index = None
  for i, id in enumerate(ids):
    if id == thymio_id:
      thymio_index = i

  (c, corners) = get_pos_aruco(detected, aruco_id)

  if c is not None and thymio_index is not None:
    local_rvecs, local_tvecs, _ = cv.aruco.estimatePoseSingleMarkers(all_corners, marker_length, mtx, dist)

    # R = cv.Rodrigues(local_rvecs[0])[0]
    # R = R @ np.array([
      # [1, 0, 0],
      # [0, 0, 1],
      # [0,-1, 0],
    # ])

    # if 0 < R[1,1] < 1:
      # cv.putText(img, f'flipped!', (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255))
 
    cv.aruco.drawAxis(img, mtx, dist, local_rvecs[0], local_tvecs[0], 0.01)
    return local_rvecs[0], local_tvecs[0]
  else:
    return None, None


def write_predefined_camera_int(path, cam_int):
  f = open(path, "wb")
  pickle.dump(cam_int, f)

def load_predefined_camera_int(path):
  f = open(path, "rb")
  return pickle.load(f)

def compute_offset_elevation(cam_int, rvecs, tvecs, elevation):
  (mtx, dist, _, _) = cam_int

  if rvecs is not None:
    imgpts, _ = cv.projectPoints(np.float32([[0, 0, -4e-3]]), rvecs, tvecs, mtx, dist)
    return np.float32(imgpts[0][0])
  else:
    return None

def transform_perspective_point(M, p):
  px = (M[0,0]*p[0] + M[0,1]*p[1] + M[0,2]) / ((M[2,0]*p[0] + M[2,1]*p[1] + M[2,2]))
  py = (M[1,0]*p[0] + M[1,1]*p[1] + M[1,2]) / ((M[2,0]*p[0] + M[2,1]*p[1] + M[2,2]))
  return (px, py)
