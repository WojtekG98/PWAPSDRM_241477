import matplotlib.pyplot as plt

def isStateValid(state):
    x = state.getX()
    y = state.getY()
    if (48 >= x >= 0 or 100 >= x >= 52) and 53 >= y >= 47:
        return False
    else:
        return True

def isStateValid2(state):
    x = state[0]
    y = state[1]
    if (48 >= x >= 0 or 100 >= x >= 52) and 53 >= y >= 47:
        return False
    else:
        return True

def paint_obs(LowB, HighB):
    plt.axis([LowB, HighB, LowB, HighB])
    rec1 = plt.Rectangle([0, 48], 47, 4, color='k')
    rec2 = plt.Rectangle([53, 48], 47, 4, color='k')
    plt.gcf().gca().add_artist(rec1)
    plt.gcf().gca().add_artist(rec2)

if __name__ == '__main__':
    paint_obs(0, 100)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
