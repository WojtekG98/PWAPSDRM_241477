from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
import PlanujSciezke
import matplotlib.pyplot as plt
from math import sqrt
from math import pi
import Astar
import random


N = 100.0
radius = 10
center = [N / 2, N / 2]
radius2 = 10
center2 = [3 * N / 4, N / 2]

def isStateValid(state):
    x = state.getX()
    y = state.getY()
    return sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2) > radius and sqrt((x - center2[0]) ** 2 + (y - center2[1]) ** 2) > radius2


def plan(space, planner, runTime, start, goal):
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    ss.setStartAndGoalStates(start, goal)
    if planner == 'RRT':
        ss.setPlanner(og.RRT(ss.getSpaceInformation()))
    elif planner == 'Astar':
        ss.setPlanner(Astar.Astar(ss.getSpaceInformation()))
    else:
        print('Bad planner')
    solved = ss.solve(runTime)
    if solved:
        path = ss.getSolutionPath()
        path.interpolate(1000)
        return path.printAsMatrix()
        # return ss.getSolutionPath().printAsMatrix()
    else:
        print("No solution found.")
        return None


def print_path_txt(path):
    plt.axis([0, N, 0, N])
    verts = []
    for line in path.split("\n"):
        x = []
        for item in line.split():
            x.append(float(item))
        if len(x) is not 0:
            verts.append(list(x))
    x = []
    y = []
    yaw = []
    for i in range(0, len(verts)):
        x.append(verts[i][0])
        y.append(verts[i][1])
        yaw.append(verts[i][2])
    print(x)
    print(y)
    for i in range(0, len(yaw)):
        yaw[i] = yaw[i]*180/pi
    print(yaw)


if __name__ == '__main__':
    space = ob.ReedsSheppStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    space.setBounds(bounds)
    # Set our robot's starting state to be random
    start = ob.State(space)
    start[0], start[1] = random.randint(0, N / 2), random.randint(0, N / 2)
    while not sqrt((start[0] - center[0]) ** 2 + (start[1] - center[1]) ** 2) > radius \
            or not \
            sqrt((start[0] - center2[0]) ** 2 + (start[1] - center2[1]) ** 2) > radius2:
        start[0], start[1] = random.randint(0, N / 2), random.randint(0, N / 2)

    # Set our robot's goal state to be random
    goal = ob.State(space)
    goal[0], goal[1] = random.randint(N / 2, N), random.randint(N / 2, N)
    while not sqrt((goal[0] - center[0]) ** 2 + (goal[1] - center[1]) ** 2) > radius \
            or not \
            sqrt((goal[0] - center2[0]) ** 2 + (goal[1] - center2[1]) ** 2) > radius2:
        goal[0], goal[1] = random.randint(N / 2, N), random.randint(N / 2, N)
    path = plan(space, 'RRT', 30, start, goal)
    if path:
        PlanujSciezke.plot_path(path, 'r-')
    path = plan(space, 'Astar', 30, start, goal)
    if path:
        PlanujSciezke.plot_path(path, 'b-')
        print_path_txt(path)
    plt.plot(start[0], start[1], 'g*')
    plt.plot(goal[0], goal[1], 'y*')
    circle1 = plt.Circle(center, radius, color='k')
    circle2 = plt.Circle(center2, radius2, color='k')
    plt.gcf().gca().add_artist(circle1)
    plt.gcf().gca().add_artist(circle2)
    plt.legend(('RRT', 'A*'))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
