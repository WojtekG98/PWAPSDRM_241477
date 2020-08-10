#!/usr/bin/env python

import matplotlib.pyplot as plt
import PlanujSciezke
import math

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), "py-bindings"))
    from ompl import base as ob
    from ompl import geometric as og

class RRT_Connect(ob.Planner):
    def __init__(self, si):
        super(RRT_Connect, self).__init__(si, "RRT_Connect")
        self.states_ = []
        self.sampler_ = si.allocStateSampler()

    def solve(self, ptc):
        pdef = self.getProblemDefinition()
        goal = pdef.getGoal()
        si = self.getSpaceInformation()
        pi = self.getPlannerInputStates()
        st = pi.nextStart()
        while st:
            self.states_.append(st)
            st = pi.nextStart()
        solution = None
        approxsol = 0
        approxdif = 1e6
        last_dist = math.inf
        while not ptc():
            rstate = si.allocState()
            # pick a random state in the state space
            self.sampler_.sampleUniform(rstate)
            # check motion
            if si.checkMotion(self.states_[-1], rstate):
                dist = goal.distanceGoal(rstate)
                if dist < last_dist:
                    self.states_.append(rstate)
                    sat = goal.isSatisfied(rstate)
                    last_dist = dist
                    if sat:
                        approxdif = dist
                        solution = len(self.states_)
                        break
                    if dist < approxdif:
                        approxdif = dist
                        approxsol = len(self.states_)
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
    space = ob.ReedsSheppStateSpace(2)
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
    start[1] = 10.
    goal = ob.State(space)
    goal[0] = 90.
    goal[1] = 90.
    ss.setStartAndGoalStates(start, goal, .05)
    # set the planner
    planner = RRT_Connect(ss.getSpaceInformation())
    ss.setPlanner(planner)

    result = ss.solve(10.0)
    if result:
        if result.getStatus() == ob.PlannerStatus.APPROXIMATE_SOLUTION:
            print("Solution is approximate")
        # try to shorten the path
        # ss.simplifySolution()
        # print the simplified path
        path = ss.getSolutionPath()
        path.interpolate(100)
        print(path.printAsMatrix())
        path = path.printAsMatrix()
        plt.plot(start[0], start[1], 'g*')
        plt.plot(goal[0], goal[1], 'y*')
        PlanujSciezke.plot_path(path, 'b-')
        plt.show()


if __name__ == "__main__":
    plan()