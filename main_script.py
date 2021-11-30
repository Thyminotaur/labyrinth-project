import numpy as np
import random as rand
import math
import cv2 as cv
from vision.vision_utils import *
from navigation.nav_global_utils import *
from obstacle_avoidance.src.obstacle_avoid_short import *
from tdmclient import ClientAsync

# Regulators
class motors_regulator:
    Kp = 4
    Kd = 1
    Ki = 1

class robot_position:
    x = 0.0
    y = 0.0
    alpha = 0.0

PID = motors_regulator()
thymio_position = robot_position()

# motors
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

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
  labyrinth_map = detect_labyrinth(dst_gray, 130)

  # Localize thymio
  detected = detect_aruco(dst_gray)
  (_, center, _) = localize_thymio(dst_gray, detected)

  if False:
      ### Compute the trajectory TODO David
      ## Set parameters
      # resize for faster computation
      h,w = labyrinth_map.shape
      scale_factor = 25
      reduced_w = w // scale_factor
      reduced_h = h // scale_factor
      labyrinth_map_reduced = cv.resize(labyrinth_map, (reduced_w, reduced_h), interpolation=cv.INTER_AREA)

      # set the objectives
      goal = find_exit(labyrinth_map_reduced)
      start = [(center[0] // scale_factor, center[1] // scale_factor)]

      # set movement type: "4N" or "8N" and cost of motion: [straight,diag,turn]
      movement_type = "8N"
      cost = [1, np.sqrt(2), 3]

      ## find the trajectory
      # A star call to find the exit with least cost motion
      global_path, closedSet = A_Star(start, goal, labyrinth_map_reduced, cost, movement_type)

      # Remove unnecessary intermediate position
      global_trajectory = get_linear_trajectory(global_path)

      # Resize to original size
      global_trajectory = scale_up_trajectory(global_trajectory, scale_factor)

      # Convert from matrix indexes to cartesian coordinates
      global_trajectory = [(x, y) for y, x in global_trajectory]

async def prog():
#def prog():
  node = await client.wait_for_node()
  await node.lock()

  distance = 0
  point_to_go = [700, 300]

  while M is not None:
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
        angle = 180.0 * angle / math.pi

    if center is not None:
      # Do trajectory with position
      # TODO [Stephen]
    
        
        thymio_position.x = center[0]
        thymio_position.y = center[1]

        cv.circle(dst, (int(thymio_position.x), int(thymio_position.y)), 50, (255,255,255))
        cv.line(dst, (int(thymio_position.x), int(thymio_position.y)), (point_to_go[0], point_to_go[1]), (255,255,255), 5)
        
        thymio_position.alpha = angle

        alpha_c = -180*math.atan2(point_to_go[1] - thymio_position.y, point_to_go[0] - thymio_position.x)/math.pi


        alpha_e =  thymio_position.alpha - alpha_c            

        print("\nx : " + str(center[0]))
        print("y : " + str(center[1]))
        
        print("\nconsigne : " + str(alpha_c))
        print("robot : " + str(thymio_position.alpha))
        print("erreur : " + str(alpha_e))

        if(distance < 50):
            point_to_go[0] = rand.randint(100, 600)
            point_to_go[1] = rand.randint(100, 400)

        distance = math.sqrt(pow(thymio_position.x - point_to_go[0], 2) + pow(thymio_position.y - point_to_go[1], 2))

        print("Distance : " + str(distance))
        
        motor_L = PID.Kd * distance + PID.Kp * alpha_e
        motor_R = PID.Kd * distance -PID.Kp * alpha_e
        #print("R : " + str(motor_R))
        #print("L : " + str(motor_L))

        
        await node.set_variables(motors(int(motor_L), int(motor_R)))
        #await node.set_variables(motors(0, 0))
        pass
    else:
      # Do trajectory without position information
      # TODO [Stephen]
      await node.set_variables(motors(0,0))
      pass

    # cv.imshow('my webcam', img)
    cv.imshow('transformed', dst)
    # cv.imshow('labyrinth', laby)

    key = cv.waitKey(1)
    if key == 27: 
      break  # esc to quit

  await node.unlock()
  cv.destroyAllWindows()

client.run_async_program(prog)
#prog()
