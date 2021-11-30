import numpy as np
import cv2 as cv
import time
import vision.vision_utils as vision
import navigation.nav_global_utils as navG

plot_img = True

### DETECT LABYRINTH

# # get image of the labyrinth
# test_scene = cv.imread("../data/terrain_1.png", cv.IMREAD_GRAYSCALE)
# h,w = test_scene.shape
# labyrinth_big = vision.detect_labyrinth(test_scene, 100)

# get data of the labyrinth
labyrinth_map = cv.imread("../global_trajectory_real.png", cv.IMREAD_GRAYSCALE)
#start = [tuple(reversed(pos)) for pos in center]
start = [(229,624)]
goal = navG.find_goal(labyrinth_map)

# resize for faster computation
scale_factor = 10
labyrinth_map_reduced, start, goal = navG.resize_data(labyrinth_map, start, goal, scale_factor)
cv.imwrite("../data/labyrinth_map_reduced.png", labyrinth_map_reduced)

# check feasibility of start and goal
feasible, start = navG.check_feasibility(labyrinth_map_reduced, start, goal)
if feasible:
    print("Start and goal are feasible. Starting A*")

    ### FIND TRAJECTORY
    t_start = time.time()

    # type; "4N" or "8N" and cost of motion; [straight,diag,turn]
    movement_type = "4N"
    cost = [1,np.sqrt(2),5]

    # A star call to find the exit with least cost motion
    global_path, closedSet = navG.A_Star(start, goal, labyrinth_map_reduced, cost, movement_type)

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
else:
    print("Start and goal ARE NOT feasible")

### RESULTS
if plot_img:
    ## Results on the reduced size of the labyrinth
    # convert to BGR
    labyrinth = cv.cvtColor(labyrinth_map_reduced, cv.COLOR_GRAY2BGR)

    # set all visited nodes in blue
    for pos in closedSet:
        labyrinth[pos] = (150,0,0)

    # set the path in red
    for pos in global_path:
        labyrinth[pos] = (0,0,255)

    #save image
    cv.imwrite("../data/global_trajectory_reduced.png", labyrinth)

    ## Results on the original size of the labyrinth
    # convert to BGR
    labyrinth_map = cv.cvtColor(labyrinth_map, cv.COLOR_GRAY2BGR)

    # set the path in red
    global_path = np.asarray([[x, y] for y, x in global_trajectory], np.int32)
    global_path = global_path.reshape((-1, 1, 2))
    cv.polylines(labyrinth_map,[global_path],False,(0,0,255))

    # set the coordinates of the trajectory in green
    for pos in global_trajectory:
        labyrinth_map[pos] = (0, 255, 0)

    # save image
    cv.imwrite("../data/global_trajectory.png", labyrinth_map)
