import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from vision_utils import *


src = cv.imread("../data/chessboard_test.jpeg", cv.IMREAD_GRAYSCALE)
src = do_adaptive_threshold(src)
cv.imshow("chessboard", src)
cv.waitKey()
