from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
from math import sqrt
from math import pi
import Astar
import RRT
import random

N = 100.0
radius = 0
center = [N / 4 + 10, N / 4 + 10]
radius2 = 00
center2 = [3 * N / 4 - 10, 3 * N / 4 - 10]


def isStateValid(state):
    x = state.getX()
    y = state.getY()
    #return (x - center[0]) ** 2 + (y - center[1]) ** 2 > radius ** 2 \
    #       and sqrt((x - center2[0]) ** 2 + (y - center2[1]) ** 2) > radius2
    return x > 75 or x < 80 and y < 39 or y > 41


def plan(space, planner, runTime, start, goal):
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    ss.setStartAndGoalStates(start, goal)
    if planner == 'RRT':
        ss.setPlanner(RRT.RRT(ss.getSpaceInformation()))
    elif planner == 'Astar':
        ss.setPlanner(Astar.Astar(ss.getSpaceInformation()))
    elif planner.lower() == "rrtconnect":
        ss.setPlanner(og.RRTConnect(ss.getSpaceInformation()))
    elif planner.lower() == "est":
        ss.setPlanner(og.EST(ss.getSpaceInformation()))
    else:
        print('Bad planner')
    solved = ss.solve(runTime)
    if solved:
        ss.simplifySolution()
        path = ss.getSolutionPath()
        # print(path.printAsMatrix())
        path.interpolate(1000)
        return path.printAsMatrix()
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
    for i in range(0, len(yaw)):
        yaw[i] = yaw[i] * 180 / pi
    return x, y, yaw


def plot_path(path, style, LowB, HighB):
    plt.axis([LowB, HighB, LowB, HighB])
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
    plt.plot(x, y, style)


if __name__ == '__main__':
    space = ob.ReedsSheppStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    space.setBounds(bounds)
    # Set our robot's starting state to be random
    start = ob.State(space)
    start[0], start[1] = 5, 5#random.randint(0, N), random.randint(0, N)
    while not sqrt((start[0] - center[0]) ** 2 + (start[1] - center[1]) ** 2) > radius \
            or not \
            sqrt((start[0] - center2[0]) ** 2 + (start[1] - center2[1]) ** 2) > radius2:
        start[0], start[1] = random.randint(0, N / 2), random.randint(0, N / 2)

    # Set our robot's goal state to be random
    goal = ob.State(space)
    goal[0], goal[1] = 35, 95 #random.randint(0, N), random.randint(0, N)
    while not sqrt((goal[0] - center[0]) ** 2 + (goal[1] - center[1]) ** 2) > radius \
            or not \
            sqrt((goal[0] - center2[0]) ** 2 + (goal[1] - center2[1]) ** 2) > radius2:
        goal[0], goal[1] = random.randint(N / 2, N), random.randint(N / 2, N)
    rrt_path = plan(space, 'RRT', 1000, start, goal)
    if rrt_path:
        plot_path(rrt_path, 'r-', 0, N)
        #print(print_path_txt(rrt_path))
    a_path = plan(space, 'Astar', 1000, start, goal)
    if a_path:
        plot_path(a_path, 'b-', 0, N)
        print_path_txt(a_path)
    rrtconnect_path = plan(space, 'rrtconnect', 1000, start, goal)
    if rrtconnect_path:
        plot_path(rrtconnect_path, 'g-', 0, N)
        print_path_txt(rrtconnect_path)
    est_path = plan(space, 'est', 1000, start, goal)
    if est_path:
        plot_path(est_path, 'm-', 0, N)
        #print(print_path_txt(est_path))
    plt.plot(start[0], start[1], 'g*')
    plt.plot(goal[0], goal[1], 'y*')
    circle1 = plt.Circle(center, radius, color='k')
    circle2 = plt.Circle(center2, radius2, color='k')
    plt.gcf().gca().add_artist(circle1)
    plt.gcf().gca().add_artist(circle2)
    plt.legend(('RRT', 'A*', 'RRT-Connect', 'EST'))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
