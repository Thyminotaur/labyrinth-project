# import numpy as np
# import cv2 as cv
# import cv2.aruco as aruco

# index = 0
# arr = []
# while True:
    # cam = cv.VideoCapture(index)
    # if not cam.isOpened():
        # break
    # else:
        # arr.append(index)
    # index += 1

# print(arr)

# print(arr)
import cv2
import numpy as np

all_camera_idx_available = []

for camera_idx in range(10):
    cap = cv2.VideoCapture(camera_idx)
    if cap.isOpened():
        print(f'Camera index available: {camera_idx}')
        all_camera_idx_available.append(camera_idx)
        cap.release()

print(all_camera_idx_available)
