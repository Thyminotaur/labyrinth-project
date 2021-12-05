import numpy as np
import cv2 as cv
import cv2.aruco as aruco

from vision_utils import *

import pickle

cam = cv.VideoCapture(0, cv.CAP_DSHOW)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

offset_x = 0
offset_y = 0

corner_idx = 1
thymio_pos = []
offset_pos = []

while True:
  ret_val, img = cam.read()
  img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

  detected = detect_aruco(img_gray)

  (_, center, angle) = localize_thymio(img_gray, detected)

  key = cv.waitKey(1)
  if key == ord('w'):
    offset_y += 1
  elif key == ord('s'):
    offset_y -= 1
  if key == ord('a'):
    offset_x += 1
  elif key == ord('d'):
    offset_x -= 1
  elif key == ord('e'):
    if center is not None:
      corner_idx = corner_idx + 1
      thymio_pos.append((center[0], center[1]))
      offset_pos.append((offset_x, offset_y))

      if corner_idx > 4:
        break
  elif key == 27: 
    break  # esc to quit

  if center is not None:
    cv.putText(img, f'({center[0]},{center[1]}) ({offset_x},{offset_y})', (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255))
  cv.putText(img, f'Corner {corner_idx} (press e to save and go to next corner)', (20, 80), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255))


  if center is not None:
    cv.drawMarker(img, np.int32(center+np.array([offset_x, offset_y])), (0, 255, 255), markerSize=40, thickness=4)
    cv.drawMarker(img, np.int32(center), (0, 0, 255), markerSize=40, thickness=4)

  h, w, _ = img.shape
  img = cv.resize(img, (3*w//4, 3*h//4))

  cv.imshow("thymio position", img)
cv.destroyAllWindows()
saved = (thymio_pos, offset_pos)

f = open("../data/z_offset.bin", "wb")
pickle.dump(saved, f)
