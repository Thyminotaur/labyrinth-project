import numpy as np
import cv2 as cv

A = np.array([4,2,1,3])
I = np.argsort(A)
print(A[I[-1]])
