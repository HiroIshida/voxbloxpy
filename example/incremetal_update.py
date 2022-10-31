import numpy as np
from _voxbloxpy import EsdfMap


quat_wxyz = [0.0, 1.0, 0.0, 0.0]
pos = [-1.0, 0.0, 0.0]
camera_pose = pos + quat_wxyz

r_sphere = 0.5
ylin = np.linspace(-1.0, 1.0, 100)
zlin = np.linspace(-1.0, 1.0, 100)
y_grid, z_grid = np.meshgrid(ylin, zlin)
pts_yz_plane = np.array(list(zip(y_grid.flatten(), z_grid.flatten())))
inside_circle = np.sum(pts_yz_plane ** 2, axis=1) < r_sphere - 0.02
pts_yz_plane_inner = pts_yz_plane[inside_circle]
pts_x = -np.sqrt(1.0 - np.sum(pts_yz_plane_inner ** 2, axis=1))

pts = np.zeros((len(pts_x), 3))
pts[:, 0] = pts_x
pts[:, 1:] = pts_yz_plane_inner

esdf = EsdfMap(0.01, 16)
esdf.update(camera_pose, pts)

#import matplotlib.pyplot as plt
#fig = plt.figure()
#ax = fig.add_subplot(projection='3d')
#ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
#plt.show()
