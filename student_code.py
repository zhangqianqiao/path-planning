import numpy as np
from collections import deque

def shortest_path(M, start, goal):
    """
      ######################################
      # 1. Initialization parameters
      # 2. Get prioritize exploration of frontier;
      #   -- get_lowest()
      # 3. Update forntier and explored
      # 4. Get adjacent point of the current boundary point
      #   -- actions()
      # 5. Explore the neighbors of the current boundary point ;
      #   update data in different situations .
      #   -- The neighbor not in expored and not in frontier;
      #   -- The neighbor in frontier;
      #   -- Other situations are not processed
      # 6. Determine whether the path reaches the goal;
      ########################################
      parameters:
         frontier -- Boundary point to be extended; set()
         explored -- Already explored area; set()
         g_score -- record the cost value of the roads; Accumulated by the path value; dict()
         f_score -- g_socre + disBetween(current_state + goal); dict()
         path -- Record how the roads to extension; key: neighbor_state; value: current_state ;
    """

    # 1.Initialization parameters
    frontier, explored, g_score, f_score, path = set([start]), set(), {}, {}, {}
    g_score[start] = 0

    while len(frontier) != 0:

        # 2. Get prioritize exploration of frontier;
        current_state = get_lowest(frontier, f_score, start)

        # 6.Whether the path reaches the goal;
        if current_state == goal:
            best_path = get_path(path, goal)
            return best_path

        # 3. Update forntier and explored
        frontier.remove(current_state)
        explored.add(current_state)

        # 4. Get adjacent point of the current boundary point;
        neighbor_state = actions(M, current_state)

        # 5. Explore the neighbors of the current boundary point ;
        for neighbor in neighbor_state:

            # Record the cost value of the current neighbor;
            current_gscore = g_score[current_state] + disBetween(M, current_state, neighbor)

            # The neighbor not in expored and not in frontier;
            if neighbor not in explored and neighbor not in frontier:
                g_score[neighbor] = current_gscore
                f_score[neighbor] = current_gscore + disBetween(M, neighbor, goal)
                path[neighbor] = current_state
                frontier.add(neighbor)

            # The neighbor in frontier;
            elif neighbor in frontier:
                if current_gscore < g_score[neighbor]:
                    g_score[neighbor] = current_gscore
                    f_score[neighbor] = current_gscore + disBetween(M, neighbor, goal)
                    path[neighbor] = current_state



    print("shorest path called")

    return 0

def get_lowest(frontier, f_score, start):
    '''
    Find the minimum value of the boundary point in frontier corresponding to the f_score.

    float("inf"): Infinite floating  number;
    '''

    if len(f_score) == 0:
        return start
    else:
        # Initialize the minimum value to infinite floating  number;
        lowest = float("inf")

        # Initialize the minimum boundary point to None;
        lowest_node = None

        # Compare each point in frontier to find the minimum value and return the lowest point
        for node in frontier:
            if f_score[node] < lowest:
                lowest = f_score[node]
                lowest_node = node
        return lowest_node



def actions(M, s):
    '''
        Call the map's roads function to return a list of locations adjacent to the current location;
    '''

    return M.roads[s]

def disBetween(M, a, b):
    '''
    Calculate the straight line distance between two points
    '''

    point_1 = M.intersections[a]
    point_2 = M.intersections[b]

    return np.sqrt((point_1[0] - point_2[0])**2 + (point_1[1] - point_2[1])**2)

def get_path(path, goal):
    """
     Reverse operation in the path dictionary to get the optimal roads;
     For example:
        path[goal] return last optimal path point, The optimal path point is again
        the key of the next optimal path point;

     Get the best roads by declaring a deque and adding path value to the left side of the deque;

    """
    # declaring deque
    best_path = deque()

    # initial the first path to the goal
    node = goal
    best_path.appendleft(node)

    # Look up the next path from the dictionary path
    while node in path:

        # Set up the current node by the value of the dictionary path;
        node = path[node]
        best_path.appendleft(node)

    return list(best_path)
