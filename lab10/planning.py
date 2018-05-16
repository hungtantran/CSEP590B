#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo



def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
        
    # The set of nodes already evaluated
    closedSet = []

    # The set of currently discovered nodes that are not evaluated yet.
    #/ Initially, only the start node is known.
    start = grid.getStart()
    openSet = [start]

    # For each node, which node it can most efficiently be reached from.
    # If a node can be reached from many nodes, cameFrom will eventually contain the
    # most efficient previous step.
    cameFrom = {}

    # For each node, the cost of getting from the start node to that node.
    gScore = {}

    # The cost of going from start to start is zero.
    gScore[start] = 0

    # For each node, the total cost of getting from the start node to the goal
    # by passing by that node. That value is partly known, partly heuristic.
    fScore = {}

    # For the first node, that value is completely heuristic.
    goals = grid.getGoals()
    if len(goals) != 1:
        return
    goal = goals[0]
    fScore[start] = heuristic(start, goal)

    while openSet:
        # current = the node in openSet having the lowest fScore[] value
        current = None
        for node in openSet:
            if current == None or fScore[current] > fScore[node]:
                current = node
        grid.addVisited(current)
        
        if current == goal:
            grid.setPath(reconstruct_path(cameFrom, current))
            return

        openSet.remove(current)
        closedSet.append(current)

        neighbors = grid.getNeighbors(current)
        for neighbor in neighbors:
            if neighbor[0] in closedSet:
                # Ignore the neighbor which is already evaluated.
                continue		

            # Discover a new node
            if neighbor[0] not in openSet:
                openSet.append(neighbor[0])
            
            # The distance from start to a neighbor
            # the "dist_between" function may vary as per the solution requirements.
            tentative_gScore = gScore[current] + neighbor[1]
            if (neighbor[0] in gScore) and (tentative_gScore >= gScore[neighbor[0]]):
                # This is not a better path
                continue

            # This path is the best until now. Record it!
            cameFrom[neighbor[0]] = current
            gScore[neighbor[0]] = tentative_gScore
            fScore[neighbor[0]] = gScore[neighbor[0]] + heuristic(neighbor[0], goal) 

    return

def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom:
        current = cameFrom[current]
        total_path = [current] + total_path
    return total_path


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
        
    #return math.sqrt(math.pow(goal[0] - current[0], 2) + math.pow(goal[1] - current[1], 2))
    height = abs(goal[0] - current[0])
    width = abs(goal[1] - current[1])
    min = height
    max = width
    if min > max:
        min = width
        max = height
    heuristic_score = min * math.sqrt(2) + (max - min)
    return heuristic_score


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent
    
    while not stopevent.is_set():
        pass # Your code here


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

