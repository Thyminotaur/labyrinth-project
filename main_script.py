import numpy as np
import random as rand
import math
import cv2 as cv
from vision.vision_utils import *
from navigation.nav_global_utils import *
from obstacle_avoidance.src.obstacle_avoid_short import *
from tdmclient import ClientAsync
import pdb
from motion.motion_utils import *
    
regulator = motors_regulator()
thymio = robot_position()

#init tdm client
client = ClientAsync()

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

if M is not None:
  # Get the labyrinth map
  ret_val, img = cam.read()
  dst = crop_labyrinth(img, M)
  dst_gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
  labyrinth_map = detect_labyrinth(dst_gray, (120,100))
  initial_center = None
  
  while initial_center is None:  
    # Localize thymio
    ret_val, img = cam.read()
    dst = crop_labyrinth(img, M)
    dst_gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
    detected = detect_aruco(dst_gray)
    (_, initial_center, _) = localize_thymio(dst_gray, detected)

  if True:
      ### Compute the trajectory TODO David
      cv.imwrite("labyrinth_map_real.png", labyrinth_map)

      ## Set parameters
      start = [(int(initial_center[1]),int(initial_center[0]))]
      goal = find_goal(labyrinth_map)
      movement_type = "4N"
      cost = [1, np.sqrt(2), 5]

      # resize for faster computation
      scale_factor = 10
      labyrinth_map_reduced, start, goal = resize_data(labyrinth_map, start, goal, scale_factor)

      # check feasibility of start and goal
      feasible, start = check_feasibility(labyrinth_map_reduced, start, goal)

      if feasible:
          labyrinth_map_reduced_color = cv.cvtColor(labyrinth_map_reduced, cv.COLOR_GRAY2BGR)
          labyrinth_map_reduced_color[start[0]] = (0, 255, 0)
          for pos in goal:
              labyrinth_map_reduced_color[pos] = (0, 0, 255)
          cv.imwrite("labyrinth_map_reduced_real.png", labyrinth_map_reduced)

          print("Start and goal are feasible. Starting A*")

          ## find the trajectory
          # A star call to find the exit with least cost motion
          global_path, closedSet = A_Star(start, goal, labyrinth_map_reduced, cost, movement_type)

          if global_path:
              path_founded = True
              # Remove unnecessary intermediate position
              global_trajectory = get_linear_trajectory(global_path)

              # Resize to original size
              global_trajectory = scale_up_trajectory(global_trajectory, scale_factor)

              # Convert from matrix indexes to cartesian coordinates
              global_trajectory = [(x, y) for y, x in global_trajectory]
          else:
              pass
      else:
          print("Current initial configuration NOT feasible")
          path_founded = False
          global_trajectory = []
      #pdb.set_trace()

async def prog():
  node = await client.wait_for_node()
  await node.lock()

  cam_int = load_predefined_camera_int("data/camera.bin")

  distance = 0
  point_to_go = [0, 0]
  actual_point = 0
  is_finished = False

  while M is not None:
    ret_val, img = cam.read()
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    dst = crop_labyrinth(img_gray, M)
    dst_th = do_adaptive_threshold(dst)
    dst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    # Localize thymio
    detected_original = detect_aruco(img_gray)
    detected = detect_aruco(dst_th)
    (_, center, angle) = localize_thymio(dst_th, detected)

    

    rvecs, tvecs = estimate_aruco_axis(img, detected_original, 2, cam_int, marker_length=13e-3)
    imgpts = compute_offset_elevation(cam_int, rvecs, tvecs, -1)

    if imgpts is not None:
      (_, center_original, _) = localize_thymio(img_gray, detected_original)
      cv.drawMarker(img, np.int32(center_original), (0, 0, 255), markerSize=40, thickness=4)
      cv.drawMarker(img, np.int32(imgpts), (0, 255, 255), markerSize=40, thickness=4)
    else:
      cv.putText(img, f'no imgpts!', (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255))

    offset_center = None
    if imgpts is not None:
      offset_center = transform_perspective_point(M, imgpts)
      cv.drawMarker(dst, np.int32(offset_center), (0, 255, 255), markerSize=40, thickness=4)

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

    # deactivate temporaily motion control
    #center = None
           
    center = offset_center
    if (center is not None) and (angle is not None):
      actual_point, point_to_go, is_finished = set_point_to_go(actual_point, point_to_go, global_trajectory, distance, is_finished)

      # Position and orientation of the thymio
      thymio.alpha = 180.0 * angle / math.pi
      thymio.position = center
      thymio.x = center[0]
      thymio.y = center[1]

      distance = compute_distance(thymio.position, point_to_go)

      cv.circle(dst, (int(thymio.x), int(thymio.y)), 5, (255,255,255))
      cv.line(dst, (int(thymio.x), int(thymio.y)), (point_to_go[0], point_to_go[1]), (255,255,255), 5)

      alpha_c = compute_angle(thymio.position, point_to_go)
      alpha_e =  thymio.alpha - alpha_c            

      motor_L, motor_R = compute_motor_speed(alpha_e, regulator, is_finished)

      #await node.set_variables(motors(0,0))

      await node.set_variables(motors(int(motor_L), int(motor_R)))
      pass
    else:
      # Do trajectory without position information
      # TODO [Stephen]
      await node.set_variables(motors(0,0))
      pass

    # set the path in red
    global_path = np.asarray(global_trajectory, np.int32)
    global_path = global_path.reshape((-1, 1, 2))
    cv.polylines(dst,[global_path],False,(255, 255, 0))
    
    cv.imshow('my webcam', img)
    cv.imshow('transformed', dst)
    # cv.imshow('labyrinth', laby)

    key = cv.waitKey(1)
    if key == 27: 
      break  # esc to quit

  await node.unlock()
  cv.destroyAllWindows()

client.run_async_program(prog)
#prog()
