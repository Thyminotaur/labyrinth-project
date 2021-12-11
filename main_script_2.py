import numpy as np
import random as rand
import math
from time import sleep
from timeit import default_timer as timer
import cv2 as cv
from vision.vision_utils import *
from navigation.nav_global_utils import *
from obstacle_avoidance.src.obstacle_avoid_short import *
from filter.kalman import *
from tdmclient import ClientAsync, aw
import pdb
from motion.motion_utils import *
from obstacle_avoidance.obstacle_avoidance_utils import *
    
regulator = motors_regulator()
thymio = robot_position()

############################################## VISION ########################################

# Open Camera
cam = cv.VideoCapture(0, cv.CAP_DSHOW)

cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

# Detect corners
# Compute camera transformation matrix
M = None

print("\nSearching corners");

while True:
    M = calibrate_corners(cam)
    if M is not None:
        break
    if M is None:
        break

print("\nLabyrinth map % and find Thymio")

if M is not None:
    # Get the labyrinth map
    ret_val, img = cam.read()
    dst = crop_labyrinth(img, M)
    dst_gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
    labyrinth_map = detect_labyrinth(dst_gray)
    initial_center = None
  
    while initial_center is None:  
        # Localize thymio
        ret_val, img = cam.read()
        dst = crop_labyrinth(img, M)
        dst_gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
        detected = detect_aruco(dst_gray)
        (_, initial_center, _) = localize_thymio(detected)

#######################################PATH FINDING ##########################################
    print("\nStart path finding")

    ### Compute the trajectory 
    cv.imwrite("labyrinth_map_real.png", labyrinth_map)

    ## Set parameters
    start = [(int(initial_center[1]),int(initial_center[0]))]
    goal = find_goal(labyrinth_map)
    movement_type = "4N"
    cost = [1, np.sqrt(2), 1]

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
            print("\nGlobal path is wrong")
    
    else:
        print("Current initial configuration NOT feasible")
        path_founded = False
        global_trajectory = []

# Load calibration data for z offset
load_z_offset_data("data/z_offset.bin")

####################################### MAIN ##########################################

print("\nSTART MAIN")

distance = 0
point_to_go = [0, 0]
prev_point_to_go = [0,0]
actual_point = 0
is_finished = False


start_time = None
loop_time = 1/10
states = np.zeros((NB_STATES,1))
kalmanFilter = kalmanEKF()

#init tdm client
client = ClientAsync()
node = aw(client.wait_for_node())
aw(node.lock())

print("\nThymio connected")

while M is not None:

    aw(node.wait_for_variables({"prox.horizontal"}))

    ##################################### VISION ##################################
    ret_val, img = cam.read()
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    dst = crop_labyrinth(img_gray, M)
    dst_th = do_adaptive_threshold(dst)
    dst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    
    # Localize thymio
    detected = detect_aruco(img_gray)
    (_, center, angle) = localize_thymio(detected)

    offset_center = None
    if center is not None:
      offset = get_z_offset(center)
      offset_center = center + np.array(offset)
      offset_center = transform_perspective_point(M, offset_center)
      cv.drawMarker(dst, np.int32(offset_center), (0, 255, 255), markerSize=40, thickness=4)

    center = offset_center

    ################################ MOTION CONTROL ###############################

    ## START Kalman filter
    if start_time is not None: loop_time = timer() - start_time
    start_time = timer()
    print(loop_time)

    if (center is not None) and (angle is not None):
        camera_measure = [center[0]*PXL_TO_MM, -center[1]*PXL_TO_MM, angle]
    else:
        camera_measure = None

    speed_r_measure = node.v.motor.right.speed*SPEED_FACTOR_TO_WORLD
    speed_l_measure = node.v.motor.left.speed*SPEED_FACTOR_TO_WORLD
    speed_measure = [(speed_r_measure+speed_l_measure)/2, (speed_r_measure-speed_l_measure)/THYMIO_DIA]

    states, _ = kalmanFilter.filter(loop_time, speed_measure, camera_measure)
    center_filtered = (states[IDX_PX]*MM_TO_PXL,-states[IDX_PY]*MM_TO_PXL)
    angle_filtered = states[IDX_THETA]
    ## END Kalman filter

    #if angle is not None and center is not None:
    actual_point, point_to_go, prev_point_to_go, is_finished = set_point_to_go(center, actual_point, prev_point_to_go, point_to_go, global_trajectory, distance, is_finished)

    # Position and orientation of the thymio
    thymio.alpha = 180.0 * angle_filtered / math.pi
    #thymio.alpha = 180.0 * angle / math.pi
    thymio.position = center_filtered
    #thymio.position = center
    thymio.x = center_filtered[0]
    #thymio.x = center[0]
    thymio.y = center_filtered[1]
    #thymio.y = center[1]

#    print(str(thymio.x) + "\t" + str(thymio.y) + "\t" + str(thymio.alpha))


    distance = compute_distance(thymio.position, point_to_go)
    distance_tot = compute_distance(prev_point_to_go, point_to_go)

    alpha_c = compute_angle(thymio.position, point_to_go)
    alpha_e =  thymio.alpha - alpha_c

    prox = node.v.prox.horizontal

    # Compute regulator gain depending on the position of the robot wrt the actual and previous point
    regulator.Kp_angle, regulator.Kp_dist = compute_regulator_gain(distance, distance_tot)

    # Compute and set motors speed
    motor_L_obstacle, motor_R_obstacle = obstacle_avoidance_speed(prox)
    motor_L, motor_R = compute_motor_speed(alpha_e, regulator, is_finished)
    motor_L += motor_L_obstacle
    motor_R += motor_R_obstacle

    aw(node.set_variables(motors(motor_L, motor_R)))
    #aw(node.set_variables(motors(0, 0)))

    # Draw indications
    cv.circle(dst, (int(thymio.x), int(thymio.y)), 40, (255,255,255))
    cv.circle(dst, (int(thymio.x), int(thymio.y)), 20, (255,255,255))
    if center is not None:
        cv.circle(dst, (int(center[0]), int(center[1])), 30, (0,255,255))
    cv.line(dst, (int(thymio.x), int(thymio.y)), (point_to_go[0], point_to_go[1]), (255,255,255), 5)

##        ## TMP COMMENT FOR KALMAN TEST
##    else :
##        aw(node.set_variables(motors(0, 0)))
        #################################### SHOW IMAGE ################################

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

    #sleep(0.010)

"""################################## END ############################"""    

aw(node.set_variables(motors(0, 0)))
aw(node.unlock())
cv.destroyAllWindows()
    
