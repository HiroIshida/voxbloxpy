import numpy as np

from voxbloxpy import Grid, get_test_esdf

esdf = get_test_esdf(0.05, 1, 640, 480)
print("finish creating esdf")

N = 60
lb = np.array([-3.0, -3.0, -2.0])
ub = np.array([3.0, 3.0, 4.0])
grid = Grid(lb, ub, (N, N, N))
gridsdf = esdf.get_grid_sdf(grid)
gridsdf.render_volume()
