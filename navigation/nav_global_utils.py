import numpy as np
import cv2 as cv

def resize_data(labyrinth, start, goal, scale_factor=10):
    h,w = labyrinth.shape
    desired_w = w // scale_factor
    desired_h = h // scale_factor
    labyrinth_reduced = cv.resize(labyrinth, (desired_w, desired_h), interpolation=cv.INTER_AREA)
    _, labyrinth_reduced = cv.threshold(labyrinth_reduced, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    new_start = list(set([(row // scale_factor, col // scale_factor) for row, col in start]))
    new_goal = list(set([(row // scale_factor, col // scale_factor) for row, col in goal]))
    return labyrinth_reduced, new_start, new_goal

def check_feasibility(labyrinth, start, goal):
    feasible = True
    initial_start = start[0]
    h, w = labyrinth.shape
    if (initial_start[0] < h) and (initial_start[1] < w) and (initial_start[0] > 0) and (initial_start[1] > 0):
        if labyrinth[initial_start]:
            print(f'Start position {start[0]} too close of a border: finding the closest feasible start...')
            movements = get_movements_8n()
            idx1 = 0
            idx2 = 1
            pos = initial_start
            while labyrinth[pos]:
                if idx1>=len(movements):
                    idx2+=1
                    idx1=0
                neighbor = (initial_start[0]+movements[idx1][0]*idx2,initial_start[1]+movements[idx1][1]*idx2)
                print(movements[idx1][0]*idx2)
                if (neighbor[0] < h) and (neighbor[1] < w) and (neighbor[0] > 0) and (neighbor[1] > 0):
                    pos = neighbor
                idx1+=1
            start[0] = pos
            print(f'The new start is {start[0]}')
    else:
        print(f'Start position {start[0]} is outside of the labyrinth {h}x{w}')
        feasible = False
    if not goal:
        print("No goal founded")
        feasible = False
    else:
        for pos in goal:
            if (pos[0] >= h) or (pos[1] >= w) or (pos[0] < 0) or (pos[1] < 0):
                print(f'Goal position {pos} is outside of the labyrinth {h}x{w}')
                feasible = False
    return feasible, start

def create_nodes_ID(grid):
    h, w = grid.shape
    nodes_ID = [(row, col) for row in range(h) for col in range(w)]
    return nodes_ID

def heuristic_cost(goal_ID, nodes_ID, norm_order):
    h = np.linalg.norm(np.asarray(nodes_ID) - np.asarray(goal_ID), ord=norm_order, axis=-1)
    return dict(zip(nodes_ID, h))

def motion_cost(previous, current, next, defined_cost):
    straight_cost = defined_cost[0]
    diag_cost = defined_cost[1]
    turn_cost = defined_cost[2]

    prev_motion = np.asarray(current) - np.asarray(previous)
    current_motion = np.asarray(next) - np.asarray(current)

    if current_motion.all():
        cost = diag_cost
    else:
        cost = straight_cost
    # add cost for turning
    if not(np.array_equal(prev_motion, current_motion)) and not( np.array_equal(prev_motion, [0,0])):
        cost = cost + turn_cost
    return cost

def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.append(cameFrom[current])
        current=cameFrom[current]
    return total_path

def get_movements_4n():
    """
    Get all possible 4-connectivity movements (up, down, left right).
    :return: list of movements [(dx, dy)]
    """
    return [(1, 0),
            (0, 1),
            (-1, 0),
            (0, -1)]

def get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1)
    (up, down, left, right and the 4 diagonals).
    :return: list of movements [(dx, dy)]
    """
    return [(1, 0),
            (0, 1),
            (-1, 0),
            (0, -1),
            (1, 1),
            (-1, 1),
            (-1, -1),
            (1, -1)]


def A_Star(start, goal, occupancy_grid, cost, movement_type="4N"):

    if movement_type == '4N':
        movements = get_movements_4n()
        LpNorm = 1
    elif movement_type == '8N':
        movements = get_movements_8n()
        LpNorm = 2
    else:
        raise ValueError('Unknown movement')

    # start from the goal towards the start
    tmp_start = start
    start = goal
    goal = tmp_start

    nodes_ID = create_nodes_ID(occupancy_grid)
    h = heuristic_cost(goal, nodes_ID, LpNorm)

    # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
    # Initially, only the start node is known.
    openSet = start

    # The set of visited nodes that no longer need to be expanded.
    closedSet = []

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
    cameFrom = dict()

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = dict(zip(nodes_ID, [np.inf for x in range(len(nodes_ID))]))

    # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
    fScore = dict(zip(nodes_ID, [np.inf for x in range(len(nodes_ID))]))
    for node in openSet:
        gScore[node] = 0
        fScore[node] = h[node]

    it = 1
    # while there are still elements to investigate
    while openSet != []:
        # the node in openSet having the lowest fScore[] value
        fScore_openSet = {key: val for (key, val) in fScore.items() if key in openSet}
        current = min(fScore_openSet, key=fScore_openSet.get)
        del fScore_openSet

        # If the goal is reached, reconstruct and return the obtained path
        if current in goal:
            return reconstruct_path(cameFrom, current), closedSet

        openSet.remove(current)
        closedSet.append(current)

        # for each neighbor of current:
        for dx, dy in movements:

            neighbor = (current[0] + dx, current[1] + dy)

            # if the node is not in the map, skip
            if (neighbor[0] >= occupancy_grid.shape[0]) or (neighbor[1] >= occupancy_grid.shape[1]) or (
                    neighbor[0] < 0) or (neighbor[1] < 0):
                continue

            # if the node is occupied or has already been visited, skip
            if (occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet):
                continue

            # motion cost is the cost from current to neighbor
            # tentative_gScore is the cost from start to the neighbor through current
            tentative_gScore = gScore[current] + motion_cost(cameFrom.get(current, current), current, neighbor, cost)

            if neighbor not in openSet:
                openSet.append(neighbor)

            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h[neighbor]

    # Open set is empty but goal was never reached
    print("No path found to goal")
    return [], closedSet

def find_goal(grid):
    mask = np.zeros_like(grid, dtype=bool)
    mask[:, 0] = True
    mask[:, -1] = True
    mask[0, :] = True
    mask[-1, :] = True

    grid_frame = np.ma.MaskedArray(grid, ~mask)

    goal = np.argwhere(grid_frame == 0)

    return [tuple(elem) for elem in goal]

def get_linear_trajectory(path):
    trajectory = []
    last_pos = path[0]
    last_motion = [0, 0]
    for next_pos in path[1:]:
        next_motion = np.asarray(next_pos) - np.asarray(last_pos)
        if not (np.array_equal(last_motion, next_motion)):
            trajectory.append(last_pos)
            last_motion = next_motion
        last_pos = next_pos
    trajectory.append(path[-1])
    return trajectory

def scale_up_trajectory(path, scale_factor):
    idx = 0
    for row, col in path:
        row = row * scale_factor + (scale_factor // 2)
        col = col * scale_factor + (scale_factor // 2)
        path[idx] = (row, col)
        idx +=1
    return path
