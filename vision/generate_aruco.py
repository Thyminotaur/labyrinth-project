import numpy as np
import cv2 as cv
import cv2.aruco as aruco

dict_id = aruco.DICT_6X6_50
id = 1


arucoDict = aruco.Dictionary_get(dict_id)

while True:
  tag = np.zeros((300, 300, 1), dtype="uint8")
  aruco.drawMarker(arucoDict, id, 300, tag, 1)

  cv.imwrite("aruco.png", tag)
  cv.imshow("ArUco Tag", tag)
  key = cv.waitKey()
  
  if key == ord('q'):
    break

  id = id + 1
