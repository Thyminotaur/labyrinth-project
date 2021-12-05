import numpy as np
import cv2 as cv
import cv2.aruco as aruco

from vision_utils import *

cam = cv.VideoCapture(0, cv.CAP_DSHOW)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

load_z_offset_data("../data/z_offset.bin")

while True:
  ret_val, img = cam.read()
  img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

  detected = detect_aruco(img_gray)

  (_, center, angle) = localize_thymio(img_gray, detected)

  if center is not None:
    offset = get_z_offset(center)
    cv.drawMarker(img, np.int32(center+np.array(offset)), (0, 255, 255), markerSize=40, thickness=4)
    cv.drawMarker(img, np.int32(center), (0, 0, 255), markerSize=40, thickness=4)

  h, w, _ = img.shape
  img = cv.resize(img, (3*w//4, 3*h//4))

  cv.imshow("a", img)

  if cv.waitKey(1) == 27: 
    break  # esc to quit
cv.destroyAllWindows()
