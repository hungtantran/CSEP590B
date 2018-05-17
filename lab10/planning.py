#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import time
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps


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


def xyToCoor(grid, start, startRobot, x, y):
    return (
        int((x - startRobot[0])/grid.scale + start[0]) + 1,
        int((y - startRobot[1])/grid.scale + start[1]) + 1
    )

def coorToXY(grid, start, startRobot, coorX, coorY):
    return (
        (coorX + 1 -start[0]) * grid.scale + startRobot[0],
        (coorY + 1 -start[1]) * grid.scale + startRobot[1]
    )

def drive_from_to(grid, start, startRobot, robot, x, y):
    x1 = robot.pose.position.x
    y1 = robot.pose.position.y
    
    angle1 = robot.pose.rotation.angle_z.degrees
    desired_angle = math.degrees(math.atan(float(y - y1) / float(x - x1)))
    turn_angle = desired_angle - angle1
    robot.turn_in_place(degrees(turn_angle)).wait_for_completed()

    dist = math.sqrt(math.pow(x - x1, 2) + math.pow(y - y1, 2)) - 50
    action = robot.drive_straight(distance_mm(dist), speed_mmps(15))
    
    while not action.is_completed:
        x1 = robot.pose.position.x
        y1 = robot.pose.position.y
        newStart = xyToCoor(grid, start, startRobot, x1, y1)
        if grid.getStart() != newStart:
            grid.clearStart()
            grid.setStart(newStart)


def drive_toward_cube(grid, start, startRobot, robot, cube):
    drive_from_to(grid, start, startRobot, robot, cube.pose.position.x, cube.pose.position.y)


def drive_to_center(grid, start, startRobot, robot):
    x, y = coorToXY(grid, start, startRobot, grid.width/2, grid.height/2)
    drive_from_to(grid, start, startRobot, robot, x, y)


def findCube(grid, startRobot, robot: cozmo.robot.Robot):
    start = grid.getStart()

    robot.set_head_angle(degrees(0)).wait_for_completed()

    grid.clearGoals()
    grid.addGoal((grid.width/2, grid.height/2))
    found = False
    while True:
        print()
        print("1. Another loop")
        rCoorX, rCoorY = xyToCoor(grid, start, startRobot, robot.pose.position.x, robot.pose.position.y)
        reach_center = (rCoorX == grid.width/2) and (rCoorY == grid.height/2)
        print("reach_center = ", reach_center)
        grid.clearStart()
        grid.setStart((rCoorX, rCoorY))

        try:
            # Update visible objects
            print("2. Update objects")
            objects = robot.world.visible_objects
            grid.clearObstacles()
            for obj in objects:
                if obj.cube_id == cozmo.objects.LightCube1Id:
                    goalCoorX, goalCoorY = xyToCoor(grid, start, startRobot, obj.pose.position.x, obj.pose.position.y)
                    grid.clearGoals()
                    grid.addGoal((goalCoorX, goalCoorY))
                    found = True
                else:
                    objCoorX, objCoorY = xyToCoor(grid, start, startRobot, obj.pose.position.x, obj.pose.position.y)
                    print(obj)
                    print(robot.pose)
                    print(objCoorX, objCoorY)
                    grid.addObstacle((objCoorX, objCoorY))

            # Find the goal (cube1), update the grid and drive toward it
            if found:
                print("3. Found goals")
                drive_toward_cube(grid, start, startRobot, robot)
                return

            if reach_center:
                print("4. Reach center")
                # Once reach center, keep turning in place
                robot.turn_in_place(degrees(20)).wait_for_completed()
            else:
                print("5. Drive toward center")
                # If not reach center, try to reach center
                # Update current robot position as start and recalculate path to center and drive toward it, 1 cell at a time
                grid.clearPath()
                print("6. Finding path")
                astar(grid, heuristic)

                path = grid.getPath()
                print("7. Found path ", path)
                if grid.checkPath() and len(path) > 1:
                    x, y = coorToXY(grid, start, startRobot, path[1][0], path[1][1])
                    drive_from_to(grid, start, startRobot, robot, x, y)
        except Exception as e:
            print("Exception")
            print(e)
            pass

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
    
    x1 = robot.pose.position.x
    y1 = robot.pose.position.y
    while not stopevent.is_set():
        findCube(grid, (x1, y1), robot)

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

