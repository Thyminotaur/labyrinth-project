import numpy as np
import pandas as pd
import navigation.nav_global_utils as navG

# import cv2 as cv
# import vision.vision_utils as vu
# labyrinth = vu.detect_labyrinth(............)
# _,start,_ = localize_thymio(.............)


labyrinth = np.zeros((11,11), dtype=int)
labyrinth[5,1:10] = 1
labyrinth[1:10,5] = 1

start = (2,2)
goal = (8,8)

# cost of motion: [straight, turn]
cost = [1,5]

global_path = navG.A_Star(start, goal, labyrinth, cost)
path = [elem[0] for elem in global_path]
motion = [elem[1] for elem in global_path]

for pos in path:
    labyrinth[tuple(pos)]=2
labyrinth[start]=3
labyrinth[goal]=4
print (pd.DataFrame(labyrinth))
