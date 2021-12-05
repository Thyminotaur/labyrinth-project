import numpy as np
import cv2 as cv
from scipy.interpolate import LinearNDInterpolator

interp = LinearNDInterpolator([
  [0.0, 0.0],
  [5.0, 0.0],
  [5.0, 5.0],
  [0.0, 5.0]], [
    [3, 2], 
    [0, 2], 
    [-3, 1], 
    [1, 1]],
  0)

print(interp(5, 5))
