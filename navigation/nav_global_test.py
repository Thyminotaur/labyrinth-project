import numpy as np
import cv2 as cv
import math
import time
import vision.vision_utils as vision
import navigation.nav_global_utils as navG

plot_img = True

### DETECT LABYRINTH

# get image of the labyrinth
test_scene = cv.imread("../data/terrain_1.png", cv.IMREAD_GRAYSCALE)
h,w = test_scene.shape
labyrinth_big = vision.detect_labyrinth(test_scene, 100)

# resize for faster computation
scale_factor = 50
desired_w = w//scale_factor
desired_h = h//scale_factor
labyrinth = cv.resize(labyrinth_big, (desired_w, desired_h))

# set the objectives
goal = navG.find_exit(labyrinth)
start = [(900//scale_factor,100//scale_factor)]

### FIND TRAJECTORY
t_start = time.time()

# type; "4N" or "8N" and cost of motion; [straight,diag,turn]
movement_type = "8N"
cost = [1,math.sqrt(2),3]

# A star call to find the exit with least cost motion
global_path, closedSet = navG.A_Star(start, goal, labyrinth, cost, movement_type)

# Remove unnecessary intermediate position
global_trajectory = navG.get_linear_trajectory(global_path)

# Resize to original size
global_trajectory = navG.scale_up_trajectory(global_trajectory, scale_factor)

t_stop = time.time()
total_t = t_stop - t_start
print(f'It took {int(total_t*1000.0)} ms')
print("Length path: ", len(global_path))
print("Length trajectory: ",len(global_trajectory))
print("Length closedSet: ", len(closedSet))

### RESULTS
if plot_img:
    ## Results on the reduced size of the labyrinth
    # convert to BGR
    labyrinth = cv.cvtColor(labyrinth, cv.COLOR_GRAY2BGR)

    # set all visited nodes in blue
    for pos in closedSet:
        labyrinth[pos] = (150,0,0)

    # set the path in red
    for pos in global_path:
        labyrinth[pos] = (0,0,255)

    #save image
    cv.imwrite("../data/global_trajectory_resized.png", labyrinth)

    ## Results on the original size of the labyrinth
    # convert to BGR
    labyrinth_big = cv.cvtColor(labyrinth_big, cv.COLOR_GRAY2BGR)

    # set the path in red
    global_path = np.asarray([[x, y] for y, x in global_trajectory], np.int32)
    global_path = global_path.reshape((-1, 1, 2))
    cv.polylines(labyrinth_big,[global_path],False,(0,0,255))

    # set the coordinates of the trajectory in green
    for pos in global_trajectory:
        labyrinth_big[pos] = (0, 255, 0)

    # save image
    cv.imwrite("../data/global_trajectory.png", labyrinth_big)
