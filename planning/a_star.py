from queue import PriorityQueue
import numpy as np
from enum import Enum


class Action(Enum):
    """
    This is coming stright from Udacity exercise

    An action is represented by a 3 element tuple.
    
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x-1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x+1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y-1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y+1] == 1:
        valid.remove(Action.RIGHT)

    return valid


def heurestic(start,goal):
    """
    takes a start point and goal point
    returns the dsitance between the two points
    """
    h = np.linalg.norm(np.array(start)-np.array(goal))
    return h

def a_star(start,goal,grid):
    """
    A star is a path planning algorithem that finds the shortest path from start to goal
    using a cost function
    """
    branch = {}
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    found=False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node==start:
            current_cost=0.0
        else:
            current_cost = branch[current_node][0]

        if current_node==goal:
            print("Found a path")
            found = True
            break
        else:
            for action in valid_actions(grid,current_node):
                delta = action.delta
                cost = action.cost
                next_node = (current_node[0]+delta[0],current_node[1]+delta[1])
                branch_cost = current_cost + cost
                queue_cost = branch_cost + heurestic(current_node,goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost,next_node))


    path = []
    path_cost = 0
    if found:
        n = goal
        path_cost = branch[goal][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])
    else:
        print("No feasiable path found")

    return path[::-1], path_cost


start = (0, 0)
goal = (4, 4)

grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0]
])

"""
Testing a*
"""
path,cost = a_star(start,goal,grid)
if cost==10.0:
    print("checks out")
