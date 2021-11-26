import math
import numpy as np
import pandas as pd
import cv2 as cv
import time
import vision.vision_utils as vision
import navigation.nav_global_utils as navG

plot_img = True

############# Test img ################################

# # get labyrinth occupancy grid
# test_scene = cv.imread("../data/terrain_1.png", cv.IMREAD_GRAYSCALE)
# h,w = test_scene.shape
# w //= 2
# h //= 2
# test_scene = cv.resize(test_scene, (w, h))
# labyrinth = vision.detect_labyrinth(test_scene, 100)
# #_, start, _ = localize_thymio(img, detected)
# #toto = navG.find_goal(labyrinth)
#
# start = (184,367)
# goal = (211,376)

################################################

########## Test matrix #########################
grid = np.ones((10, 10), dtype=int) * 255
grid[1,0:8] = 0
grid[3,1:10] = 0
grid[1:8,8] = 0
grid[2:10,3] = 0
grid[8,0:9] = 0
grid[5,0:3] = 0

goal = navG.find_exit(grid)
start = [(3,1)]

labyrinth = np.copy(grid)

for pos in goal:
    grid[pos] += 3
print (pd.DataFrame(grid))

###################################################

############# Find trajectory #####################

# cost of motion: [straight,diag,turn]
cost = [1,math.sqrt(2),2]
movement_type = "4N"

global_path, closedSet, h_cost = navG.A_Star(start, goal, labyrinth, cost, movement_type)

###################################################
global_path = [(y, x) for x, y in global_path] # reverse for indexing matrix
i = 1
for pos in global_path:
    labyrinth[pos] += i
    i+=1
# for pos in goal:
#     labyrinth[pos] += 3
# for pos in start:
#     labyrinth[pos] -= 1
print (pd.DataFrame(labyrinth))

h_cost_matrix = np.zeros_like(labyrinth)
for (k,v) in h_cost.items():
    h_cost_matrix[k]=v

print (pd.DataFrame(h_cost_matrix))

# if plot_img:
#     global_path = [(y, x) for x, y in global_path] # reverse for indexing matrix
#     for pos in closedSet:
#         labyrinth[pos] = 100
#     for pos in global_path:
#         labyrinth[pos] = 200
#     labyrinth[start] = 64
#     labyrinth[goal] = 192
#     print("Length path:", len(global_path))
#     cv.imwrite("../data/global_trajectory.png", labyrinth)

# labyrinth = np.zeros((11,11), dtype=int)
# labyrinth[5,1:10] = 1
# labyrinth[1:10,5] = 1
# start = (2,2)
# goal = (8,8)

# # cost of motion: [straight,diag,turn]
# cost = [1,math.sqrt(2),2]
# movement_type = "8N"
#
# global_path = navG.A_Star(start, goal, labyrinth, cost, movement_type)

# for pos in global_path:
#     labyrinth[pos]=2
# labyrinth[start]=3
# labyrinth[goal]=4
# print (pd.DataFrame(labyrinth))
# print("Total length:", len(global_path))
