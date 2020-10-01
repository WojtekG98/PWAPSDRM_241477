#!/usr/bin/env python

import matplotlib.pyplot as plt
import WygladzSciezke
import math
from enum import Enum

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), "py-bindings"))
    from ompl import base as ob
    from ompl import geometric as og


class Node:
    """
    RRT_Connect Node
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.path = []

    def __eq__(self, other):
        return self.position.getX() == other.position.getX() and self.position.getY() == other.position.getY()

    def __str__(self):
        return str(self.position.getX()) + ", " + str(self.position.getY()) + ", " + \
               str(self.position.getYaw() * 180 / math.pi)


class GrowState(Enum):
    Trapped = 0
    Advanced = 1
    Reached = 2


# noinspection PyPep8Naming
class RRT_Connect(ob.Planner):

    def __init__(self, si):
        super(RRT_Connect, self).__init__(si, "RRT_Connect")
        self.treeA = []  # tree starting from start node
        self.treeB = []  # tree starting from end node
        self.states_ = []
        self.sampler_ = si.allocStateSampler()

    def expand(self, Tree, random_state):
        si = self.getSpaceInformation()
        # find nearest node
        nearest_node = self.near(random_state, Tree)
        # find new node based on step size
        new_node_xy = self.step(nearest_node, random_state)
        new_node_position = si.allocState()
        new_node_position.setXY(new_node_xy[0], new_node_xy[1])
        new_node_position.setYaw(new_node_xy[2]*180/math.pi)
        # new_node_position.setYaw(0)
        # connect the random node with its nearest node
        new_node = Node(nearest_node, new_node_position)
        # check the motion
        if si.checkMotion(new_node.position, Tree[-1].position):
            # add new node to the tree
            Tree.append(new_node)
            if si.distance(Tree[-1].position, random_state) < 1:
                return GrowState.Reached
            else:
                return GrowState.Advanced
        else:
            return GrowState.Trapped

    def near(self, random_state, Tree):
        si = self.getSpaceInformation()
        # find the nearest node
        dmin = si.distance(Tree[0].position, random_state)
        nearest_node_id = 0
        for i in range(0, len(Tree)):
            if si.distance(Tree[i].position, random_state) < dmin:
                dmin = si.distance(Tree[i].position, random_state)
                nearest_node_id = i
        return Tree[nearest_node_id]

    def step(self, nearest_node, random_state):
        (xnear, ynear) = (nearest_node.position.getX(), nearest_node.position.getY())
        (xrand, yrand) = (random_state.getX(), random_state.getY())
        (px, py) = (xrand - xnear, yrand - ynear)
        theta = math.atan2(py, px)
        (x, y) = (xnear + math.cos(theta), ynear + math.sin(theta))
        return x, y, theta

    def connect(self, Tree, q):
        S = self.expand(Tree, q)
        return S

    def solve(self, ptc):
        pdef = self.getProblemDefinition()
        si = self.getSpaceInformation()
        pi = self.getPlannerInputStates()
        goal = pdef.getGoal()
        st = pi.nextStart()
        while st:
            self.states_.append(st)
            st = pi.nextStart()
        start_state = pdef.getStartState(0)
        goal_state = goal.getState()
        self.treeA.append(Node(None, start_state))
        self.treeB.append(Node(None, goal_state))
        solution = None
        approxsol = 0
        approxdif = 1e6
        random_state = si.allocState()
        while not ptc():
            # new random node
            self.sampler_.sampleUniform(random_state)
            if self.expand(self.treeA, random_state) != GrowState.Trapped:
                if self.connect(self.treeB, self.treeA[-1].position) == GrowState.Reached:
                    if self.treeA[0].position == start_state:
                        start_tree = self.treeA
                        end_tree = self.treeB
                        print('A to start')
                    else:
                        start_tree = self.treeB
                        end_tree = self.treeA
                        print('B to start')
                    path_from_mid_to_start = []
                    path_from_mid_to_end = []
                    current = start_tree[-1]
                    while current.position != start_state:
                        path_from_mid_to_start.append(current.position)
                        current = current.parent
                    current = end_tree[-1]
                    while current is not None:
                        path_from_mid_to_end.append(current.position)
                        current = current.parent
                    path = path_from_mid_to_start[::-1] + path_from_mid_to_end
                    for i in range(0, len(path)):
                        self.states_.append(path[i])
                    solution = len(self.states_)
                    #if self.treeB[0].position == start_state:
                    #    print('B to start')
                    #    path_from_mid_to_start = []
                    #    path_from_mid_to_end = []
                    #    current = self.treeB[-1]
                    #    while current.position != start_state:
                    #        path_from_mid_to_start.append(current.position)
                    #        current = current.parent
                    #    current = self.treeA[-1]
                    #    while current is not None:
                    #        path_from_mid_to_end.append(current.position)
                    #        current = current.parent
                    #    path = path_from_mid_to_start[::-1] + path_from_mid_to_end
                    #    for i in range(0, len(path)):
                    #        self.states_.append(path[i])
                    #    solution = len(self.states_)
                    break
            self.treeA, self.treeB = self.treeB, self.treeA
            # print('treeA:', self.treeA)
            # print('treeB:', self.treeB)

        solved = False
        approximate = False
        if not solution:
            solution = approxsol
            approximate = True
        if solution:
            path = og.PathGeometric(si)
            for s in self.states_[:solution]:
                path.append(s)
            pdef.addSolutionPath(path)
            solved = True
        return ob.PlannerStatus(solved, approximate)

    def clear(self):
        super(RRT_Connect, self).clear()
        self.states_ = []


def isStateValid(state):
    return True


def plan():
    # create an ReedsShepp State space
    space = ob.ReedsSheppStateSpace(5)
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(100)
    space.setBounds(bounds)
    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    start = ob.State(space)
    start[0] = 10.
    start[1] = 90.
    goal = ob.State(space)
    goal[0] = 90.
    goal[1] = 10.
    ss.setStartAndGoalStates(start, goal, .05)
    # set the planner
    planner = RRT_Connect(ss.getSpaceInformation())
    ss.setPlanner(planner)

    result = ss.solve(100.0)
    if result:
        if result.getStatus() == ob.PlannerStatus.APPROXIMATE_SOLUTION:
            print("Solution is approximate")
        # try to shorten the path
        # ss.simplifySolution()
        # print the simplified path
        path = ss.getSolutionPath()
        path.interpolate(100)
        #print(path.printAsMatrix())
        path = path.printAsMatrix()
        plt.plot(start[0], start[1], 'g*')
        plt.plot(goal[0], goal[1], 'y*')
        WygladzSciezke.plot_path(path, 'b-', 0, 100)
        plt.show()


if __name__ == "__main__":
    plan()
