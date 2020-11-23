from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
from math import sqrt
from math import pi
import Astar
import RRT
import random
def isStateValid(state):
    x = state.getX()
    y = state.getY()
    if (25 >= x >= 5 and 25 >= y >= 20) or (25 >= x >= 20 and 25 >= y >= 5):
        return False
    else:
        return True


def paint_obs(LowB, HighB):
    plt.axis([LowB, HighB, LowB, HighB])
    rec1 = plt.Rectangle([5, 20], 20, 5, color='k')
    rec2 = plt.Rectangle([20, 5], 5, 20, color='k')
    plt.gcf().gca().add_artist(rec1)
    plt.gcf().gca().add_artist(rec2)


if __name__ == '__main__':
    paint_obs(0, 100)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()