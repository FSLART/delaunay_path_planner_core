import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

points = np.array([[-2,1], [-2,3], [-2,5], [2,1], [2,3], [2,5], [0,1], [0,2], [0,3], [0,4], [0,5],
[-2,2], [-2,4], [2,2], [2,4], [0,0]])

tri = Delaunay(points)

plt.triplot(points[:,0], points[:,1], tri.simplices)
plt.plot(points[:,0], points[:,1], 'o')
plt.show()