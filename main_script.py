import numpy as np
import cv2 as cv
from vision.vision_utils import *
from navigation.nav_global_utils import *
from obstacle_avoidance.src.obstacle_avoid_short import *
from tdmclient import ClientAsync

# motors
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

# init tdm client
# client = ClientAsync()

# Open Camera
cam = cv.VideoCapture(0, cv.CAP_DSHOW)

cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

# Detect corners
# Compute camera transformation matrix
M = None
while True:
  M = calibrate_corners(cam)
  if M is not None:
    break
  if M is None:
    break

if M:
  # Get the labyrinth map
  ret_val, img = cam.read()
  dst = crop_labyrinth(img, M)
  dst_gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
  labyrinth_map = detect_labyrinth(dst_gray, 130)

  # Localize thymio
  detected = detect_aruco(dst_gray)
  (_, center, _) = localize_thymio(dst_gray, detected)

  # Compute the trajectory 
  # TODO [David]

# async def prog():
def prog():
  # node = await client.wait_for_node()
  # await node.lock()

  while M:
    ret_val, img = cam.read()
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    dst = crop_labyrinth(img, M)

    # Localize thymio
    detected = detect_aruco(dst)
    (_, center, angle) = localize_thymio(dst, detected)

    # Do obstacle avoidance
    # TODO [Sylvain]
    ## ------------------------------------------
    ## Full code in         obstacle_avoidance.src.obstacle_avoid_full
    ## Implemented code in  obstacle_avoidance.src.obstacle_avoid_short
    ## ------------------------------------------

    # (motor_left, motor_right) = obstacle_avoidance_short(variables)

    #with ClientAsync() as client:
    #async def prog():
        #with await client.lock() as node:
            #await node.watch(variables=True)
            #node.add_variables_changed_listener(obstacle_avoidance_full)
           #await client.sleep()
    #client.run_async_program(prog)

    if center is not None:
      # Do trajectory with position
      # TODO [Stephen]
      pass
    else:
      # Do trajectory without position information
      # TODO [Stephen]
      pass

    # cv.imshow('my webcam', img)
    cv.imshow('transformed', dst)
    # cv.imshow('labyrinth', laby)

    key = cv.waitKey(1)
    if key == 27: 
      break  # esc to quit

  # await node.unlock()
  cv.destroyAllWindows()

# client.run_async_program(prog)
prog()
