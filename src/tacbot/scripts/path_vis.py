from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('solution_path.txt')
fig = plt.figure()
ax = fig.gca()
ax.plot(data[:, 0], data[:, 1], '.-')
plt.show()
