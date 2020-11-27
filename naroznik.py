import matplotlib.pyplot as plt

def isStateValid(state):
    x = state.getX()
    y = state.getY()
    if (26 >= x >= 4 and 26 >= y >= 19) or (26 >= x >= 19 and 26 >= y >= 4):
        return False
    else:
        return True

def isStateValid2(state):
    x = state[0]
    y = state[1]
    if (26 >= x >= 4 and 26 >= y >= 19) or (26 >= x >= 19 and 26 >= y >= 4):
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
