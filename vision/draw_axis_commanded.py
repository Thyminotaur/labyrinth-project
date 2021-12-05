import numpy as np
import cv2 as cv
import cv2.aruco as aruco

from vision_utils import *
import pickle

from tdmclient import ClientAsync

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

client = ClientAsync()

# 21.9 mm
# cam_int = find_camera_intrinsics("../data/calibrate_camera", 21.9e-3)

cam_int = load_predefined_camera_int("../data/camera.bin")

cam = cv.VideoCapture(0, cv.CAP_DSHOW)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

async def prog():
  node = await client.wait_for_node()
  await node.lock()

  while True:
    # marker_test = cv.imread("../data/terrain_test.jpg")

    ret_val, img = cam.read()
    marker_test = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # f = open("camera.bin", "wb")
    # pickle.dump(cam_int, f)


    # marker_test = cv.resize(marker_test, (w//2, h//2))

    detected = detect_aruco(marker_test)
    (center, c) = get_pos_aruco(detected, 2)


    rvecs, tvecs = estimate_aruco_axis(img, detected, 2, cam_int, marker_length=13e-3)

    imgpts = compute_offset_elevation(cam_int, rvecs, tvecs, 0.1)

    if imgpts is not None:
      cv.drawMarker(img, np.int32(imgpts), (0, 0, 255), markerSize=40, thickness=4)

    # h, w = marker_test.shape
    # img = cv.resize(img, (w//2, h//2))


    cv.imshow("marker test", img)

    key = cv.waitKey(1)
    if key == ord('a'):
      await node.set_variables(motors(50, 0))
      await client.sleep(1)
      await node.set_variables(motors(0, 0))
    elif key == ord('d'):
      await node.set_variables(motors(0, 50))
      await client.sleep(1)
      await node.set_variables(motors(0, 0))
    elif key == ord('w'):
      await node.set_variables(motors(50, 50))
      await client.sleep(1)
      await node.set_variables(motors(0, 0))
    elif key == ord('s'):
      await node.set_variables(motors(-50, -50))
      await client.sleep(1)
      await node.set_variables(motors(0, 0))

    elif key == 27: 
      break  # esc to quit

  await node.unlock()
  cv.destroyAllWindows()

client.run_async_program(prog)
