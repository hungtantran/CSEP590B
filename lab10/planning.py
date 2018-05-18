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

CUBE_SIZE = 50.0
ROBOT_WIDTH = 50.0
ROBOT_LENGTH = 75.0

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
        int(float(x - startRobot[0])/grid.scale + start[0]),
        int(float(y - startRobot[1])/grid.scale + start[1])
    )

def coorToXY(grid, start, startRobot, coorX, coorY):
    return (
        (coorX - start[0]) * grid.scale + startRobot[0] + 25,
        (coorY - start[1]) * grid.scale + startRobot[1] + 25
    )

def lightCubeXYToBorderCoors(grid, start, startRobot, x, y):
    offset = [-50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50]
    cornersXY = []
    for i in [-50, 50]:
        for off in offset:
            cornersXY.append((x + i, y + off))
    for i in [-50, 50]:
        for off in offset:
            cornersXY.append((x + off, y + i))

    cornersCoors = []
    for xy in cornersXY:
        cornersCoors.append(xyToCoor(grid, start, startRobot, xy[0], xy[1]))
    return cornersCoors

def turn_to(grid, start, startRobot, robot, coorX, coorY):
    x, y = coorToXY(grid, start, startRobot, coorX, coorY)

    x1, y1 =front_robot(robot)
    
    angle1 = robot.pose.rotation.angle_z.degrees
    desired_angle = math.degrees(math.atan(float(y - y1) / float(x - x1)))
    turn_angle = desired_angle - angle1
    robot.turn_in_place(degrees(turn_angle)).wait_for_completed()
    if abs(turn_angle) > 20:
        return True
    return False

def drive_straight_to(grid, start, startRobot, robot, coorX, coorY):
    print("Drive straight")
    x, y = coorToXY(grid, start, startRobot, coorX, coorY)

    x1, y1 = front_robot(robot)
    dist = math.sqrt(math.pow(x - x1, 2) + math.pow(y - y1, 2))
    action = robot.drive_straight(distance_mm(dist), speed_mmps(15))
    
    while not action.is_completed:
        x1, y1 = front_robot(robot)
        newStart = xyToCoor(grid, start, startRobot, x1, y1)
        if grid.getStart() != newStart:
            grid.clearStart()
            grid.setStart(newStart)
    print("Drive straight done")


def update_obstacles(grid, start, startRobot, robot):
    objects = robot.world.visible_objects
    for obj in objects:
        if obj.cube_id != cozmo.objects.LightCube1Id:
            cornersCoors = lightCubeXYToBorderCoors(grid, start, startRobot, obj.pose.position.x, obj.pose.position.y)
            for corner in cornersCoors:
                grid.addObstacle((corner[0], corner[1]))


def front_robot(robot):
    x = robot.pose.position.x
    y = robot.pose.position.y
    #angle = robot.pose.rotation.angle_z.degrees
    #return (x + math.cos(math.radians(angle)) * 40, y + math.sin(math.radians(angle)) * 40)
    return (x, y)


def findCube(grid, robot: cozmo.robot.Robot):
    startRobot = front_robot(robot)
    start = grid.getStart()
    robot.set_head_angle(degrees(0)).wait_for_completed()

    grid.clearGoals()
    grid.addGoal((grid.width/2, grid.height/2))
    goal = None
    angle = None
    reach_center = False
    while True:
        print()
        print("1. Another loop")
        curRobot = front_robot(robot)
        rCoorX, rCoorY = xyToCoor(grid, start, startRobot, curRobot[0], curRobot[1])
        reach_center = reach_center or (
            (rCoorX == grid.width/2 or rCoorX == grid.width/2 + 1 or rCoorX == grid.width/2 - 1) and
            (rCoorY == grid.height/2 or rCoorY == grid.height/2 + 1 or rCoorY == grid.height/2 -1))
        print("reach_center = ", reach_center)
        grid.clearStart()
        grid.setStart((rCoorX, rCoorY))

        try:
            # Update visible objects
            print("2. Update objects")
            update_obstacles(grid, start, startRobot, robot)

            # Find the goal (cube1), update the grid and drive toward it
            if goal:
                print("8. Found goals")
                update_obstacles(grid, start, startRobot, robot)
                grid.clearPath()
                print("9. Finding path")
                astar(grid, heuristic)

                path = grid.getPath()
                print("10. Found path ", path)
                if grid.checkPath() and len(path) > 2:
                    if turn_to(grid, start, startRobot, robot, path[1][0], path[1][1]):
                        continue
                    drive_straight_to(grid, start, startRobot, robot, path[1][0], path[1][1])
                # Reach goal:
                if len(path) <= 2:
                    turn_to(grid, start, startRobot, robot, goal[0], goal[1])
                    stopevent.set()
                    return
                continue

            # Found the goal (cube1)
            objects = robot.world.visible_objects
            for obj in objects:
                if obj.cube_id == cozmo.objects.LightCube1Id:

                    cornersCoors = lightCubeXYToBorderCoors(grid, start, startRobot, obj.pose.position.x, obj.pose.position.y)
                    for corner in cornersCoors:
                        grid.addObstacle((corner[0], corner[1]))

                    coorX, coorY = xyToCoor(grid, start, startRobot, obj.pose.position.x, obj.pose.position.y)
                    angle = obj.pose.rotation.angle_z.degrees
                    if angle < 45 and angle > -45:
                        coorX -= 3
                    elif angle > 45 and angle < 135:
                        coorY -= 3
                    elif angle > 135 or angle < -135:
                        coorX += 3
                    else:
                        coorY += 3
                    grid.clearGoals()
                    goal = (coorX, coorY)
                    grid.addGoal((coorX, coorY))
                    continue

            if reach_center:
                print("4. Reach center")
                grid.clearPath()
                grid.clearGoals()
                # Once reach center, keep turning in place
                robot.turn_in_place(degrees(20)).wait_for_completed()
            else:
                print("5. Drive toward center")

                update_obstacles(grid, start, startRobot, robot)
                # If not reach center, try to reach center
                # Update current robot position as start and recalculate path to center and drive toward it, 1 cell at a time
                grid.clearPath()
                print("6. Finding path")
                astar(grid, heuristic)

                path = grid.getPath()
                print("7. Found path ", path)
                if grid.checkPath() and len(path) > 1:
                    if turn_to(grid, start, startRobot, robot, path[1][0], path[1][1]):
                        continue
                    drive_straight_to(grid, start, startRobot, robot, path[1][0], path[1][1])
        except Exception as e:
            print("Exception")
            print(e)
            return

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
        findCube(grid, robot)

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

