from numpy.linalg import norm
import numpy as np

a = np.array([0., 0., 0., 0., 0.])
b = np.array([0.5, 0.5, 0., 0., 0.])

size = a.size
cost = np.zeros(size)
dot = np.zeros(size)
l2 = np.zeros(size)
prev_cost = 0

for i in range(size):
    a1 = a[0:i+1]
    b1 = b[0:i+1]
    dot[i] = np.dot(a1, b1)
    l2[i] = norm(b1)
    cost[i] = l2[i] - dot[i] - prev_cost
    prev_cost += cost[i]


print("dot: ", dot)
print("l2: ", l2)
print("cost: ", cost)
