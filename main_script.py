import numpy as np
import random as rand
import math
import cv2 as cv
from vision.vision_utils import *
from navigation.nav_global_utils import *
from obstacle_avoidance.src.obstacle_avoid_short import *
from tdmclient import ClientAsync
import pdb

# Regulators
class motors_regulator:
    Kp = 4
    Kd = 0.5
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
  labyrinth_map = detect_labyrinth(dst_gray, (140,120))
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
#def prog():
  node = await client.wait_for_node()
  await node.lock()

  distance = 0
  point_to_go = [0, 0]
  actual_point = 0
  is_finished = False

  while M is not None:
    ret_val, img = cam.read()
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img = do_adaptive_threshold(img)

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

    # deactivate temporaily motion control
    #center = None
           

    if center is not None:
      # Do trajectory with position
      # TODO [Stephen]

      # At the beginning set point to go to the first point of the list
      if actual_point == 0:
        point_to_go = list(global_trajectory[0])
        actual_point = 1

      # Point to go = Position of the robot if we reach the end of the list
      elif actual_point >= len(global_trajectory) and distance < 5:
        is_finished = True

      # If the robot is close to the point to go -> Next point
      elif distance < 5:
        point_to_go = list(global_trajectory[actual_point])
        actual_point += 1


      # Position and orientation of the thymio
      thymio_position.alpha = 180.0 * angle / math.pi
      thymio_position.x = center[0]
      thymio_position.y = center[1]


      distance = math.sqrt(pow(thymio_position.x - point_to_go[0], 2) + pow(thymio_position.y - point_to_go[1], 2))

      #if(distance < 20):
      #  point_to_go[0] = rand.randint(100, 600)
      #  point_to_go[1] = rand.randint(100, 400)
        
      cv.circle(dst, (int(thymio_position.x), int(thymio_position.y)), 5, (255,255,255))
      cv.line(dst, (int(thymio_position.x), int(thymio_position.y)), (point_to_go[0], point_to_go[1]), (255,255,255), 5)

      alpha_c = -180*math.atan2(point_to_go[1] - thymio_position.y, point_to_go[0] - thymio_position.x)/math.pi
      alpha_e =  thymio_position.alpha - alpha_c            

      #print("\nx : " + str(center[0]))
      #print("y : " + str(center[1]))
      print("\nconsigne : " + str(alpha_c))
      print("robot : " + str(thymio_position.alpha))
      print("erreur : " + str(alpha_e))
      #print("Distance : " + str(distance))

      if is_finished:
        motor_L = 0
        motor_R = 0

      elif (alpha_e < 180 and alpha_e > -180) :
        motor_L = PID.Kd * (180-abs(alpha_e)) + PID.Kp * alpha_e
        motor_R = PID.Kd * (180-abs(alpha_e)) - PID.Kp * alpha_e

      elif alpha_e < -180 :
        alpha_e = 360 + alpha_e
        motor_L = PID.Kd * (180-abs(alpha_e)) + PID.Kp * alpha_e
        motor_R = PID.Kd * (180-abs(alpha_e)) - PID.Kp * alpha_e

      elif alpha_e > 180 :
        alpha_e = -360 + alpha_e
        motor_L = PID.Kd * (180-abs(alpha_e)) + PID.Kp * alpha_e
        motor_R = PID.Kd * (180-abs(alpha_e)) - PID.Kp * alpha_e

      print("\nmotor_L : " + str(motor_L))
      print("motor_R : " + str(motor_R))
     
      await node.set_variables(motors(int(motor_L), int(motor_R)))
      #await node.set_variables(motors(0, 0))
      pass
    else:
      # Do trajectory without position information
      # TODO [Stephen]
      await node.set_variables(motors(0,0))
      pass

    # set the path in red
    global_path = np.asarray(global_trajectory, np.int32)
    global_path = global_path.reshape((-1, 1, 2))
    cv.polylines(dst,[global_path],False,255)
    
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
