import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D


def cube():
    with open("test3.txt") as handle:
        lst = list()
        for line in handle.readlines():
            lst.extend(line.rstrip().split(','))
    print("done", lst[0])
    p = (len(lst))/3
    xs = []
    ys = []
    zs = []
    j = 0
    for i in range(len(lst)):
        if j == 0:
            xs.append(float(lst[i]))
        if j == 1:
            ys.append(float(lst[i]))
        if j == 2:
            zs.append(float(lst[i]))
        j += 1
        if j > 2:
            j = 0
    fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
    ax.scatter(xs,ys,zs, s=0.0001)
    plt.show()


def something(t):
    with open("test.txt") as handle:
        lst = list()
        for line in handle.readlines():
            lst.extend(line.rstrip().split(','))
    print("done", lst[0])
    p = (len(lst))/3
    xs = []
    ys = []
    zs = []
    j = 0
    for i in range(t):
        if j == 0:
            xs.append(float(lst[i]))
        if j == 1:
            ys.append(float(lst[i]))
        if j == 2:
            zs.append(float(lst[i]))
        j += 1
        if j > 2:
            j = 0
    fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
    ax.scatter(xs,ys,zs, s=0.001)
    plt.show()

something(6000000)
#cube()