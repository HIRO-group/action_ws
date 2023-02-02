from numpy.linalg import norm
import numpy as np
import math

a = np.array([0.1, 0.5, 0.2])
b = np.array([0.8, -0.1, 0.2])

size = a.size
cost = np.zeros(size)
tcost = np.zeros(size)
dot = np.zeros(size)
l2 = np.zeros(size)
prev_cost = 0

for i in range(size):
    a1 = a[0:i+1]
    b1 = b[0:i+1]
    print("a1: ", a1)
    print("b1: ", b1)
    dot[i] = np.dot(a1, b1)
    l2[i] = math.pow(norm(b1), 2)
    cost[i] = l2[i] - dot[i] - prev_cost
    tcost[i] = l2[i] - dot[i]
    prev_cost += cost[i]


print("dot: ", dot)
print("l2: ", l2)
print("cost: ", cost)
print("tcost: ", tcost)
