import sys
from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
from math import sqrt
from math import pi
import Astar
import RRT
import random
import naroznik
import ciesnina
import losoweprzeszkody
import labirynt


cies = 0
lab = 1
loso = 0
naro = 0

N = 100.0
radius = 20
center = [N / 4 + 10, N / 4 + 10]
radius2 = 20
center2 = [3 * N / 4 - 10, 3 * N / 4 - 10]


def isStateValid(state):
    x = state.getX()
    y = state.getY()
    return (x - center[0]) ** 2 + (y - center[1]) ** 2 > radius ** 2 \
           and sqrt((x - center2[0]) ** 2 + (y - center2[1]) ** 2) > radius2



def plan(space, planner, runTime, start, goal):
    ss = og.SimpleSetup(space)
    if naro == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(naroznik.isStateValid))
    if cies == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(ciesnina.isStateValid))
    if loso == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(losoweprzeszkody.isStateValid))
    if lab == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(labirynt.isStateValid))

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
    print(planner, ":")
    solved = ss.solve(runTime)
    if solved:
        ss.simplifySolution()
        path = ss.getSolutionPath()
        print("Info:    Path length:", path.length())
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
    for vert in verts:
        vert[2] = vert[2]*180/pi
    #x = []
    #y = []
    #yaw = []
    #for i in range(0, len(verts)):
    #    x.append(verts[i][0])
    #    y.append(verts[i][1])
    #    yaw.append(verts[i][2])
    #for i in range(0, len(yaw)):
    #    yaw[i] = yaw[i] * 180 / pi
    return verts


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
    # Set our robot's starting and goal states to be random
    start, goal = ob.State(space), ob.State(space)
    if naro == 1:
        start[0], start[1] = 18, 18
        goal[0], goal[1] = 60, 60
    if cies == 1:
        start[0], start[1] = 10, 10
        goal[0], goal[1] = 20, 80
    if loso == 1:
        start[0], start[1] = 25, 25
        goal[0], goal[1] = 90, 80
    if not lab == 1:
        start[0], start[1] = 5, 90
        goal[0], goal[1] = 90, 5
    else:
        start[0], start[1] = random.randint(0, N), random.randint(0, N)
        goal[0], goal[1] = random.randint(0, N), random.randint(0, N)
        if lab == 1:
            while not labirynt.isStateValid2(start):
                start[0], start[1] = random.randint(0, N), random.randint(0, N)
            while not labirynt.isStateValid2(goal):
                goal[0], goal[1] = random.randint(0, N), random.randint(0, N)
        if loso == 1:
            while not losoweprzeszkody.isStateValid2(start):
                start[0], start[1] = random.randint(0, N/2), random.randint(0, N/2)
            while not losoweprzeszkody.isStateValid2(goal):
                goal[0], goal[1] = random.randint(N/2, N), random.randint(N/2, N)
        if cies == 1:
            while not ciesnina.isStateValid2(start):
                start[0], start[1] = random.randint(0, N), random.randint(0, N/2)
            while not ciesnina.isStateValid2(goal):
                goal[0], goal[1] = random.randint(0, N), random.randint(N/2, N)
    print("start: ", start[0], start[1])
    print("goal: ", goal[0], goal[1])
    rrt_path = plan(space, 'RRT', 1000, start, goal)
    plt.figure(1)
    if rrt_path:
        plot_path(rrt_path, 'r-', 0, N)
        print(print_path_txt(rrt_path))
        plt.plot(start[0], start[1], 'g*')
        plt.plot(goal[0], goal[1], 'y*')
        if naro == 1:
            naroznik.paint_obs(0, N)
        if cies == 1:
            ciesnina.paint_obs(0, N)
        if loso == 1:
            losoweprzeszkody.paint_obs(0, N)
        if lab == 1:
            labirynt.paint_obs(0, N)
        plt.legend(('RRT', 'start', 'goal'))
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig("path_RRT.png")
    a_path = plan(space, 'Astar', 1000, start, goal)
    plt.figure(2)
    if a_path:
        plot_path(a_path, 'b-', 0, N)
        print(print_path_txt(a_path))
        plt.plot(start[0], start[1], 'g*')
        plt.plot(goal[0], goal[1], 'y*')
        if naro == 1:
            naroznik.paint_obs(0, N)
        if cies == 1:
            ciesnina.paint_obs(0, N)
        if loso == 1:
            losoweprzeszkody.paint_obs(0, N)
        if lab == 1:
            labirynt.paint_obs(0, N)
        plt.legend(('A*', 'start', 'goal'))
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig("path_A_star.png")
    rrtconnect_path = plan(space, 'rrtconnect', 1000, start, goal)
    plt.figure(3)
    if rrtconnect_path:
        plot_path(rrtconnect_path, 'g-', 0, N)
        print(print_path_txt(rrtconnect_path))
        plt.plot(start[0], start[1], 'g*')
        plt.plot(goal[0], goal[1], 'y*')
        if naro == 1:
            naroznik.paint_obs(0, N)
        if cies == 1:
            ciesnina.paint_obs(0, N)
        if loso == 1:
            losoweprzeszkody.paint_obs(0, N)
        if lab == 1:
            labirynt.paint_obs(0, N)
        plt.legend(('RRT-Connect', 'start', 'goal'))
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig("path_RRT-Connect.png")
    est_path = plan(space, 'est', 1000, start, goal)
    plt.figure(4)
    if est_path:
        plot_path(est_path, 'm-', 0, N)
        print(print_path_txt(est_path))
        plt.plot(start[0], start[1], 'g*')
        plt.plot(goal[0], goal[1], 'y*')
        if naro == 1:
            naroznik.paint_obs(0, N)
        if cies == 1:
            ciesnina.paint_obs(0, N)
        if loso == 1:
            losoweprzeszkody.paint_obs(0, N)
        if lab == 1:
            labirynt.paint_obs(0, N)
        plt.legend(('EST', 'start', 'goal'))
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig("path_EST.png")
    plt.figure(5)
    if naro == 1:
        naroznik.paint_obs(0, N)
    if cies == 1:
        ciesnina.paint_obs(0, N)
    if loso == 1:
        losoweprzeszkody.paint_obs(0, N)
    if lab == 1:
        labirynt.paint_obs(0, N)
    plot_path(rrt_path, 'r-', 0, N)
    plot_path(a_path, 'b-', 0, N)
    plot_path(rrtconnect_path, 'g-', 0, N)
    plot_path(est_path, 'm-', 0, N)
    plt.legend(('RRT', 'A*', 'RRT-Connect', 'EST'))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.plot(start[0], start[1], 'g*')
    plt.plot(goal[0], goal[1], 'y*')
    plt.savefig("path.png")
    #plt.show()
