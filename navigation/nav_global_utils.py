import numpy as np

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

    if not(np.array_equal(prev_motion, current_motion)) and not( np.array_equal(prev_motion, [0,0])): # turning
        cost = cost + turn_cost

    return cost

def heuristic_cost(goal_ID, nodes_ID, norm_order):
    h = np.linalg.norm(np.asarray(nodes_ID) - np.asarray(goal_ID), ord=norm_order, axis=-1)
    return dict(zip(nodes_ID, h))

def find_exit(grid):
    mask = np.zeros_like(grid, dtype=bool)
    mask[:, 0] = True
    mask[:, -1] = True
    mask[0, :] = True
    mask[-1, :] = True

    grid_frame = np.ma.MaskedArray(grid, ~mask)

    exit = np.argwhere(grid_frame == 0)

    return [tuple(elem) for elem in exit]

def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.append(cameFrom[current])
        current=cameFrom[current]
    # reverse for global coords instead of matrix index: col = x, row = y
    total_path = [(x, y) for y, x in total_path]
    print("Length path = ", len(total_path))
    return total_path

def create_nodes_ID(grid):
    height, width = grid.shape
    nodes_ID = [(row, col) for row in range(height) for col in range(width)]
    # x, y = np.mgrid[0:height:1, 0:width:1] # reverse order as indexing matrix is different than x,y coords
    # pos = np.empty(x.shape + (2,))
    # pos[:, :, 0] = x
    # pos[:, :, 1] = y
    # pos = np.reshape(pos, (x.shape[0] * x.shape[1], 2))
    # nodes_ID = list([(int(x[0]), int(x[1])) for x in pos])
    return nodes_ID

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
            print("Length ClosedSet = ", len(closedSet))
            return reconstruct_path(cameFrom, current), closedSet, h

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